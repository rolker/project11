// Copyright 2026 Center for Coastal and Ocean Mapping & NOAA-UNH Joint
// Hydrographic Center, University of New Hampshire
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// [uma-ADR-0010 D9 / uma-ADR-0011 / uma-ADR-0013] Batch overview-pyramid builder
// for a depth store layer — production path (level discovery, per-level fold,
// level loop, argument parsing). The `build_depth_overviews` CLI is a thin
// main() over these.
//
// Folds the layer's native tiles into coarser parent tiles, level by level, into
// the layer's `overviews/` sidecar (flat dir, `<level>_<row>_<col>.tif` — level
// rides in the filename, same as the native layer). Overviews are DERIVED +
// REGENERABLE: each run rebuilds the sidecar (idempotent; safe to re-run after
// every ingest).
//
// Fold policy (depth, ADR-0010 D9): SHALLOWEST-PRESERVING. Among the valid
// contributors that land in one coarse cell, the one with the MAXIMUM ellipsoidal
// height (band 0 — most positive / least negative, i.e. shoalest and most
// hazardous to navigation) is selected and its whole {depth, σ} pair is carried
// through. Never a mean: a coarse corridor query must plan around the rock, not
// average it away, and the σ must stay coherent with the depth it describes, so
// the pair travels together (the fold operates on whole cells, all bands at once
// — see overview_builder.hpp CellFoldPolicy). VALID means band 0 (depth) is not
// NaN — NaN is the store's per-band no-data sentinel (see bathymetry_tile.hpp).
//
// Layer scope (uma-ADR-0010 D9, as amended by #331): draft, processed AND
// reference get a generated pyramid; chart is exempt — its ENC scale ladder is a
// cartographer-curated, shoal-biased native pyramid. No upsampling: only parent
// levels are built (the min_level < discovered-finest check plus the
// fold-toward-apex loop enforce it).
//
// NATIVE-WINS, and the display inverts it. On disk a derived tile is written
// only where no native tile occupies that (level, index): nothing compiled is
// overwritten, and no merge policy is needed or implied. On screen a consumer
// composites every level <= its selection with finer over coarser, so a derived
// level 7 folded from harbour-band data draws OVER a native level 6 compiled at
// a coarser scale wherever both exist. Both halves are the intended behaviour
// (uma-ADR-0013 D3's ECDIS-consistent corollary) and must be read together.
//
// Safety (uma-ADR-0013 D8): nothing in the query path reads this sidecar or its
// manifest. shallowestReliable() keeps scanning native tiles to the finest
// available level for a region, so declining to merge finer data into a compiled
// coarse tile carries no operational risk.
//
// Memory: tiles are grouped by parent FROM GRID INDICES and loaded <=4 children
// at a time (a whole-level in-memory fold would grow without bound with store
// size; this path peaks around one parent tile's contributor buckets — see
// overview_builder.hpp — and that peak does not grow with store size). Each level
// is built from the level below it — the native tiles there plus the derived
// tiles just written there — not by re-reading the finest data.

#include "marine_bathymetry_store/overview_pyramid.hpp"

#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_autonomy/gggs/index_math.h"
#include "marine_bathymetry_store/bathymetry_tile.hpp"
#include "marine_tiled_raster_store/coverage_manifest.hpp"
#include "marine_tiled_raster_store/overview_builder.hpp"
#include "marine_tiled_raster_store/tile_io.hpp"

namespace marine_bathymetry_store
{

// Depth-band indices and the fold policy live in `detail` (declared in the header)
// so the fold's determinism is directly unit-testable — feeding the same
// contributors in two orders must yield one {depth, σ}. Everything else is
// file-local (anonymous namespace below).
namespace detail
{

using Cell = marine_tiled_raster_store::CellValues<double>;

// The on-disk depth value tile is 2-band Float64: depth (band 0, ellipsoidal
// height in metres) + uncertainty (band 1, σ in metres). NaN is the per-band
// no-data sentinel (bathymetry_tile.hpp).
constexpr std::size_t kDepthBand = 0;   // shoalest-selection + no-data band
constexpr std::size_t kSigmaBand = 1;   // uncertainty (σ): the equal-depth tie-break key

// Total order for shallowest-preserving selection: true when candidate @p c should
// displace the current best @p best. The order is lexicographic and therefore
// TOTAL:
//   1. depth (band 0) DESCENDING — the larger ellipsoidal height (shoaler, most
//      hazardous to navigation) wins; both depths are non-NaN here (validCell gate);
//   2. on an EXACT depth tie, a finite σ beats a NaN σ (prefer the pair that
//      carries a real uncertainty); then
//   3. still tied, the SMALLER σ wins — the more reliable of two equally-shoal
//      pairs (ADR-0010 D9 "shoalest-reliable").
// When depth AND σ are both equal (or both σ NaN), the two cells carry identical
// {depth, σ}, so keeping the first is result-identical. The fold is thus a function
// of the contributor SET, not its order — buckets fill in filesystem-iteration
// order (overview_builder.hpp), which is not guaranteed, so an order-sensitive
// tie-break would make the sidecar non-idempotent.
inline bool shoalerThenMoreReliable(const Cell & c, const Cell & best)
{
  if (c[kDepthBand] != best[kDepthBand]) {
    return c[kDepthBand] > best[kDepthBand];
  }
  const bool c_sigma_nan = std::isnan(c[kSigmaBand]);
  const bool best_sigma_nan = std::isnan(best[kSigmaBand]);
  if (c_sigma_nan != best_sigma_nan) {
    return best_sigma_nan;   // finite σ preferred over NaN σ
  }
  if (c_sigma_nan) {
    return false;   // both σ NaN: identical {depth, σ}, keep the first
  }
  return c[kSigmaBand] < best[kSigmaBand];   // both finite: smaller σ wins
}

// Shallowest-preserving fold (ADR-0010 D9). Among the valid contributors, select
// the shoalest — MAXIMUM ellipsoidal height (band 0) — and return its WHOLE
// {depth, σ} pair, so the coarse cell's uncertainty stays coherent with the depth
// it describes. Never a mean: a coarse corridor query must not average a rock away,
// and a mixed depth/σ pair would describe a cell that never existed. Equal-depth
// ties break deterministically by σ (see shoalerThenMoreReliable), so the result
// depends only on the contributor SET, never enumeration order — the sidecar stays
// idempotent. Called only with >=1 contributor (the engine gates on validCell
// first), each carrying a non-NaN depth.
Cell depthShallowestFold(const std::vector<Cell> & contributors)
{
  const Cell * best = &contributors.front();
  for (const Cell & c : contributors) {
    if (shoalerThenMoreReliable(c, *best)) {
      best = &c;
    }
  }
  return *best;   // the whole pair, depth AND its paired uncertainty
}

// Saturated conservative per-tile geometric error (uma-ADR-0013 D1/D2).
//
// D2 makes error nesting a PRODUCER obligation — a tile's error must be at least
// the maximum of its descendants' — and requires a producer that cannot compute
// a meaningful error to "record a conservative upper bound rather than omit the
// field". An "unknown" sentinel would be that same omission wearing a hat, so
// this records the bound: max(level GSD, max child ε).
//
// It is computable from this producer alone. GGGS's nominal cell size halves
// with each level, so a child whose own ε is unrecorded contributes at most its
// level's GSD — strictly smaller than the parent's — and saturation holds even
// across an edge where no native ε exists. Where nothing records a finer error
// the value degenerates to exactly the level's ground sample distance, which is
// the level-as-resolution behaviour consumers already fall back to, so its
// arrival changes nothing for them and its later refinement (once the other
// three D2 writers record real errors) is purely additive.
//
// nominal_cell_size is the equatorial cell size; away from the equator the true
// ground sample is smaller in longitude, so the equatorial figure is itself an
// upper bound — which is the direction a conservative error must err.
double saturatedGeometricError(
  int level, int child_level,
  const std::vector<std::optional<double>> & child_errors)
{
  const double child_gsd = gggs::levels[child_level].nominal_cell_size;
  double error = gggs::levels[level].nominal_cell_size;
  for (const std::optional<double> & child_error : child_errors) {
    const double value = child_error.value_or(child_gsd);
    if (value > error) {
      error = value;
    }
  }
  return error;
}

}  // namespace detail

namespace
{

namespace fs = std::filesystem;

/// Atomically exchange two directory entries.
///
/// Wraps renameat2(RENAME_EXCHANGE) through syscall() rather than the glibc
/// wrapper: the wrapper only appeared in glibc 2.28, and RENAME_EXCHANGE lives in
/// <linux/fs.h> on some toolchains and <stdio.h> (under _GNU_SOURCE) on others.
/// Going through SYS_renameat2 directly keeps this working across both without a
/// feature-test dance.
///
/// @return 0 on success, -1 with errno set otherwise. errno is ENOSYS on a kernel
///   or platform without the call, and EINVAL/EOPNOTSUPP on a filesystem that
///   cannot do it — the caller treats all three as "fall back", not "fail".
int exchangeDirEntries(const char * a, const char * b)
{
#if defined(__linux__) && defined(SYS_renameat2)
#ifndef RENAME_EXCHANGE
#define RENAME_EXCHANGE (1 << 1)
#endif
  return static_cast<int>(
    ::syscall(SYS_renameat2, AT_FDCWD, a, AT_FDCWD, b, RENAME_EXCHANGE));
#else
  (void)a;
  (void)b;
  errno = ENOSYS;
  return -1;
#endif
}

using marine_tiled_raster_store::TiledRasterTile;
using Cell = marine_tiled_raster_store::CellValues<double>;

// Full band count of the on-disk depth tile (depth + uncertainty).
constexpr std::size_t kBands = BathymetryTile::value_band_count;   // 2

// Contributor gate. NaN in the DEPTH band (band 0) is the no-data sentinel
// (bathymetry_tile.hpp initialises empty cells to {NaN, NaN}). A cell with a real
// depth participates even if its uncertainty happens to be NaN — the depth is the
// navigable quantity and the pair is carried as-is; gating on band 0 alone
// matches how the store itself distinguishes surveyed from unsurveyed cells.
bool validCell(const Cell & cell) {return !std::isnan(cell[detail::kDepthBand]);}

// One level's tile counts: how many child tiles were read, how many parents were
// written, and how many parents were left to a native tile. The IN count is the
// diagnostic one for a partial store — an operator seeing "40 in" for a
// 1000-tile layer knows immediately what is wrong.
struct LevelCounts
{
  std::size_t in = 0;
  std::size_t out = 0;
  std::size_t suppressed_by_native = 0;
};

// Where one contributor tile lives. Children at a given level come from two
// places at once in a mixed-level layer: the native tiles in the layer dir, and
// the derived tiles this run just wrote into staging. The two sets are disjoint
// by construction (a derived tile is never written where a native one exists),
// so a flat list with each tile's directory is enough — no precedence rule is
// needed here, because there is never a collision to resolve.
struct SourceTile
{
  gggs::GridIndex grid;
  const fs::path * dir;
};

// Build one coarser level. @p children are the contributor tiles at
// @p child_level; parents are written to @p out_dir. A parent whose
// `(level, index)` is already occupied by a NATIVE tile is skipped and counted —
// native data always wins on disk. Each written parent is added to
// @p derived with its saturated geometric error (uma-ADR-0013 D1/D2), read back
// from @p derived for children that were themselves derived.
LevelCounts buildLevel(
  const std::vector<SourceTile> & children, const fs::path & out_dir,
  uint8_t child_level,
  const marine_tiled_raster_store::CoverageManifest & native,
  marine_tiled_raster_store::CoverageManifest & derived)
{
  LevelCounts counts;
  std::map<gggs::GridIndex, std::vector<SourceTile>> by_parent;
  for (const SourceTile & child : children) {
    ++counts.in;
    const gggs::GridIndex parent_grid = gggs::parent(child.grid);
    if (parent_grid.valid()) {
      by_parent[parent_grid].push_back(child);
    }
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::optional<double>> nodata(
    kBands, std::optional<double>(nan));
  for (const auto & group : by_parent) {
    // NATIVE-WINS. Compiled data is never overwritten and never merged into, so
    // the "what should a fold of harbour data into an approach-band tile mean?"
    // question is not answered here — it is removed.
    //
    // Suppression is WHOLE-TILE, and under a mixed-level producer (uma#369's
    // depth-adaptive `processed`) that has a KNOWN consequence for the coarse
    // display tier. `reference`'s mixed levels come from disjoint S-102
    // footprints, but depth bands within one contiguous survey share parent
    // indices, and the depth ladder GUARANTEES it: a 217 m level-12 tile deep on
    // one half and shallow on the other yields native 12 beside native 13/14
    // under one parent. Where that happens the shallow band's fold is dropped at
    // that level and every level coarser.
    //
    // This is NOT a writer obligation — "never emit two native levels over the
    // same ground" is unsatisfiable for a depth-adaptive writer, and stating it
    // as one (as an earlier draft of the ADR-0010 D9 amendment did) would be
    // telling cube_bathymetry#143 not to be depth-adaptive across a tile
    // boundary. Safety is unaffected: navigation reads the region-aware NATIVE
    // query and never an LOD level (uma-ADR-0013 D8). What is affected is the
    // coarse display tier — a design input for the coarse-tier work, recorded in
    // the ADR-0010 D9 amendment. The pyramid itself needs no change.
    if (native.contains(group.first)) {
      ++counts.suppressed_by_native;
      continue;
    }
    std::vector<TiledRasterTile<double>> child_tiles;
    child_tiles.reserve(group.second.size());
    std::vector<const TiledRasterTile<double> *> child_ptrs;
    std::vector<std::optional<double>> child_errors;
    child_errors.reserve(group.second.size());
    for (const SourceTile & child : group.second) {
      const fs::path path =
        *child.dir / marine_tiled_raster_store::tileFilename(child.grid);
      child_tiles.push_back(
        marine_tiled_raster_store::loadTile<double>(
          path.string(), gggs::Level(child_level), kBands));
      child_ptrs.push_back(&child_tiles.back());
      // nullopt for a native child: no producer records an error for those yet,
      // and saturatedGeometricError substitutes the child level's GSD.
      child_errors.push_back(derived.geometricError(child.grid));
    }
    const TiledRasterTile<double> parent_tile =
      marine_tiled_raster_store::buildParentTile<double>(
      group.first, child_ptrs,
      std::vector<double>(kBands, nan),
      validCell, detail::depthShallowestFold);
    marine_tiled_raster_store::saveTile<double>(
      parent_tile,
      (out_dir / marine_tiled_raster_store::tileFilename(group.first)).string(),
      nodata);
    derived.add(
      group.first,
      detail::saturatedGeometricError(
        group.first.level(), child_level, child_errors));
    ++counts.out;
  }
  return counts;
}

// Strict integer parse: rejects an empty, non-numeric or trailing-garbage value
// that std::atoi would silently read as 0 (`--min-level abc` must be a usage
// error, not a silent "build to the apex").
bool parseInt(const char * text, int & out)
{
  if (text == nullptr || text[0] == '\0') {
    return false;
  }
  try {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (text[used] != '\0') {
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

DepthArgStatus parseDepthOverviewArgs(
  int argc, char ** argv, DepthOverviewOptions & out)
{
  out = DepthOverviewOptions{};
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--min-level" && i + 1 < argc) {
      if (!parseInt(argv[++i], out.min_level)) {
        return DepthArgStatus::kError;
      }
    } else if (arg == "--dry-run") {
      out.dry_run = true;
    } else if (arg == "--help" || arg == "-h") {
      return DepthArgStatus::kHelp;
    } else if (out.layer_dir.empty() && !arg.empty() && arg[0] != '-') {
      out.layer_dir = arg;
    } else {
      return DepthArgStatus::kError;
    }
  }
  // The no-upsample guard (min_level strictly below the layer's finest native
  // level) can no longer be checked here: the finest level is DISCOVERED from the
  // layer, so the check moved into buildDepthOverviewPyramid, where it throws
  // std::invalid_argument. Parsing validates only what argv alone can decide.
  if (out.layer_dir.empty() || out.min_level < 0 || out.min_level > 20) {
    return DepthArgStatus::kError;
  }
  return DepthArgStatus::kOk;
}

DepthOverviewBuildResult buildDepthOverviewPyramid(
  const DepthOverviewOptions & opts, std::ostream * progress)
{
  if (opts.min_level < 0 || opts.min_level > 20) {
    throw std::invalid_argument(
      "buildDepthOverviewPyramid: min_level out of bounds");
  }
  const fs::path layer_dir(opts.layer_dir);
  if (!fs::is_directory(layer_dir)) {
    throw std::runtime_error("not a directory: " + opts.layer_dir);
  }

  // Level discovery (uma-ADR-0013 D3). One all-level scan yields the layer's
  // native coverage, which is both the guard below and the fold's input: a
  // mixed-level layer cannot be pyramided without knowing which regions hold data
  // at which level.
  //
  // The manifest is held IN MEMORY only and never written to <layer>/. This
  // builder does not own the native tiles, so a native coverage.json it wrote
  // would go stale on the next s102_import with nothing able to notice — the
  // scan fallback fires on a manifest's ABSENCE, not on its staleness. Only the
  // derived manifest, which this builder does own, is persisted (into
  // overviews.tmp/, so it rides uma-ADR-0011's rename-aside).
  std::size_t guard_skipped = 0;
  const marine_tiled_raster_store::CoverageManifest native =
    marine_tiled_raster_store::scanCoverage(layer_dir.string(), guard_skipped);

  // Guard: never wipe the sidecar for an empty or mis-pointed layer. Require at
  // least one usable native tile at SOME level before touching overviews/ — a
  // path typo must not destroy a previously-good build. This generalises the old
  // "no fine tiles at the declared level" refusal; there is no declared level to
  // mistype any more.
  if (native.empty()) {
    // Distinguish "nothing there" (a path typo) from "tiles are there but none
    // could be reconstructed" — the same message for both sends the operator
    // hunting a typo that does not exist.
    throw std::runtime_error(
      "no usable native tiles under " + opts.layer_dir +
            (guard_skipped > 0 ?
            " (" + std::to_string(guard_skipped) + " tile name(s) were present "
            "but failed grid reconstruction — see the warnings above; not a path typo)" :
            " (no tile of the form <level>_<row>_<col>.tif — check the path)") +
      "; refusing to replace overviews/");
  }

  const std::vector<uint8_t> native_levels = native.levels();
  const int finest = static_cast<int>(native_levels.back());
  // No-upsample invariant, now against the DISCOVERED finest level: the coarsest
  // level built must be strictly coarser than the layer's finest native data, so
  // the build only ever produces coarser tiles.
  if (opts.min_level >= finest) {
    throw std::invalid_argument(
      "buildDepthOverviewPyramid: min_level " + std::to_string(opts.min_level) +
      " is not below the layer's finest native level " + std::to_string(finest) +
      " (that would ask for an upsample)");
  }

  // The scan is filename-only. Another store's layer whose tiles happen to be
  // named <level>_<row>_<col>.tif would pass it and be rebuilt with the depth
  // shallowest-preserving policy (and its band semantics). Probe ONE TILE PER
  // DISCOVERED LEVEL — a mixed-level layer can hold a wrong-shape band at a level
  // the finest-level probe never opens.
  for (const uint8_t level : native_levels) {
    const std::vector<gggs::GridIndex> at_level = native.gridsAt(level);
    const std::string probe_path =
      (layer_dir /
      marine_tiled_raster_store::tileFilename(at_level.front())).string();
    const int probe_bands = marine_tiled_raster_store::tileRasterCount(probe_path);
    if (probe_bands != static_cast<int>(kBands)) {
      throw std::runtime_error(
        "not a depth layer: " + probe_path + " has " +
        std::to_string(probe_bands) + " band(s), expected " +
        std::to_string(kBands) + " (depth, uncertainty); refusing to replace "
        "overviews/ with a depth-policy pyramid");
    }
  }

  DepthOverviewBuildResult result;
  result.tiles_skipped = guard_skipped;
  for (const uint8_t level : native_levels) {
    result.native_levels.push_back(static_cast<int>(level));
  }

  // --dry-run stops here: the guards above are exactly the checks that catch a
  // mistyped path or wrong layer, and nothing below this point is reached without
  // writing. Report the discovered coverage — the report that replaces
  // --fine-level's mis-pointed-path guard — and touch nothing.
  if (opts.dry_run) {
    if (progress != nullptr) {
      *progress << "dry run: " << native.size() << " usable native tile(s) under " <<
        opts.layer_dir << " (" << guard_skipped << " unreconstructable)\n";
      for (const uint8_t level : native_levels) {
        *progress << "  native level " << static_cast<unsigned>(level) << ": " <<
          native.countAt(level) << " tile(s)\n";
      }
      *progress << "  would build levels " << (finest - 1) << "..." <<
        opts.min_level << " and replace " <<
        (layer_dir / "overviews").string() << "\n";
    }
    return result;
  }

  // Refuse the swap unless every native tile name reconstructed: a pyramid
  // missing a tile's coverage must never displace a previously-complete sidecar.
  //
  // This is decided from the guard scan alone, so it is settled BEFORE the run
  // lock is claimed and before a single tile is folded. Doing it after the fold
  // (as this originally did) burned a full staging copy and hours of I/O on a
  // layer that was already known unbuildable, and held the per-layer lock for the
  // whole futile run.
  if (result.tiles_skipped > 0) {
    return result;
  }

  const fs::path overviews = layer_dir / "overviews";
  const fs::path staging = layer_dir / "overviews.tmp";
  const fs::path retired = layer_dir / "overviews.old";

  // Crash-safe regeneration: build into a staging sibling and swap it over the
  // live sidecar only after every level succeeds, so an interrupted or throwing
  // run leaves the previous overviews/ intact rather than a truncated one a
  // consumer would read as complete.
  //
  // The staging directory doubles as the run lock: create_directory fails when it
  // already exists, so a second concurrent run over the same layer refuses instead
  // of interleaving its tiles with the first run's. A crashed run leaves the
  // directory behind as debris — the message says how to clear it.
  std::error_code create_ec;
  if (!fs::create_directory(staging, create_ec)) {
    throw std::runtime_error(
      "cannot claim staging directory " + staging.string() +
            (create_ec ?
            ": " + create_ec.message() :
            " (it already exists — another build is running over this layer, or a "
            "crashed run left it behind; remove it to retry)"));
  }

  // The derived coverage this run produces (uma-ADR-0013 D3), accumulated as the
  // levels are built. It is both the record written into the sidecar and the
  // lookup for a derived child's geometric error on the next level down.
  marine_tiled_raster_store::CoverageManifest derived;
  try {
    // Fold from just under the finest native level toward the apex. Contributors
    // at each child level are the NATIVE tiles there plus the DERIVED tiles this
    // run just wrote there — disjoint by construction, so no precedence rule is
    // needed between them.
    for (int level = finest - 1; level >= opts.min_level; --level) {
      const uint8_t child_level = static_cast<uint8_t>(level + 1);
      std::vector<SourceTile> children;
      for (const gggs::GridIndex & grid : native.gridsAt(child_level)) {
        children.push_back(SourceTile{grid, &layer_dir});
      }
      for (const gggs::GridIndex & grid : derived.gridsAt(child_level)) {
        children.push_back(SourceTile{grid, &staging});
      }

      const LevelCounts counts =
        buildLevel(children, staging, child_level, native, derived);
      const std::size_t native_here = native.countAt(static_cast<uint8_t>(level));
      if (progress != nullptr) {
        // child_level is uint8_t: without the cast it streams as a character.
        *progress << "level " << static_cast<unsigned>(child_level) << " -> " <<
          level << ": " <<
          counts.in << " tile(s) in, " << counts.out << " overview tile(s) out, " <<
          counts.suppressed_by_native << " left to native, " << native_here <<
          " native tile(s) already at level " << level << "\n";
      }
      result.tiles_suppressed_by_native += counts.suppressed_by_native;
      // Invariant: a level can never end up with no coverage at all. `children`
      // is non-empty at every iteration (the first reads the finest native level,
      // which the native.empty() guard proved non-empty; each later one reads a
      // level that itself just produced a derived tile or holds a native one), and
      // gggs::parent stays valid down to level 0, so by_parent is non-empty. Each
      // parent group is then either written (counts.out++) or suppressed, and
      // suppression requires a native tile AT THIS level (native_here >= 1).
      // Hence counts.out == 0 implies native_here >= 1.
      //
      // This used to be a runtime refusal with its own result flag and CLI exit
      // code. It cannot fire — see uma#331's review — so shipping it as a live
      // safety guard misrepresented what the builder actually protects against.
      // A broken tile chain surfaces through tiles_skipped instead.
      assert(counts.out > 0 || native_here > 0);
      (void)native_here;   // only read by the assert in NDEBUG builds
      if (counts.out > 0) {
        result.tiles_written += counts.out;
        result.derived_by_level[level] = counts.out;
        result.coarsest_level = level;
      }
    }

    // The derived manifest is written INTO STAGING, before the swap, so it rides
    // the rename-aside and is crash-consistent with the sidecar it describes
    // (uma-ADR-0011 §2). Reaching here means the build was not refused — the only
    // refusal, tiles_skipped, returned before staging was ever created.
    marine_tiled_raster_store::saveCoverageManifest(
      derived,
      (staging / marine_tiled_raster_store::coverageManifestFilename()).string(),
      "derived");
  } catch (...) {
    // Best-effort: cleanup must not throw here, or it would replace the original
    // exception with its own.
    std::error_code ec;
    fs::remove_all(staging, ec);   // never leave a partial staging dir behind
    throw;
  }

  // Swap staging over the live sidecar.
  //
  // PREFERRED PATH — renameat2(RENAME_EXCHANGE): atomically exchanges the two
  // directory entries, so <layer>/overviews/ resolves to a complete sidecar at
  // EVERY instant, and a concurrent reader either sees the old pyramid or the new
  // one, never a missing directory. This matters because consumers treat an
  // absent overviews/ as "this layer has no overviews" rather than retrying:
  // CAMP's GggsTileLayer skips the directory outright when it does not exist, so
  // a layer opened during a non-atomic swap would render with no overview tiles
  // at all until the operator reloaded it.
  //
  // FALLBACK — rename-aside, for filesystems without RENAME_EXCHANGE (it needs
  // Linux >= 3.15 and per-filesystem support; NFS and some overlay/FUSE mounts
  // return EINVAL/ENOSYS/EOPNOTSUPP). overviews/ -> overviews.old/, staging ->
  // overviews/, then drop overviews.old/. That path is crash-SAFE but NOT atomic:
  // the previous sidecar's contents are never destroyed before the new one is in
  // place, but the PATH overviews/ is briefly absent between the two renames.
  const bool had_previous = fs::exists(overviews);
  bool retired_moved = false;
  bool exchanged = false;
  if (had_previous) {
    // Both entries must exist for an exchange; staging always does here.
    if (exchangeDirEntries(staging.c_str(), overviews.c_str()) == 0) {
      // staging now holds the PREVIOUS sidecar; retire it under the usual name so
      // the cleanup below drops it.
      exchanged = true;
      std::error_code ec;
      fs::remove_all(retired, ec);
      fs::rename(staging, retired, ec);
      if (ec) {
        // The swap itself succeeded — the live sidecar is correct. Only the
        // leftover previous sidecar could not be moved aside; say where it is.
        std::cerr << "warning: swap succeeded but could not retire the previous "
          "sidecar from " << staging.string() << ": " << ec.message() <<
          "; remove it by hand before the next run" << std::endl;
      }
    } else {
      const int saved = errno;
      // Distinguish "this filesystem cannot exchange directory entries" (fall
      // back to rename-aside) from a real failure such as EACCES or EIO, which
      // the fallback would only hit again.
      const bool unsupported = saved == EINVAL || saved == ENOSYS ||
        saved == EOPNOTSUPP || saved == EPERM;
      if (!unsupported) {
        std::error_code ec;
        fs::remove_all(staging, ec);   // clear the run lock before reporting
        throw fs::filesystem_error(
          "atomic sidecar swap failed", staging, overviews,
          std::error_code(saved, std::generic_category()));
      }
    }
  }

  if (!exchanged) {
    try {
      // Retire the previous sidecar (overviews/ -> overviews.old/) and swap the
      // new one in under one guard: a throw from the retire step must clear the
      // staging lock too, or overviews.tmp/ blocks the next run.
      if (had_previous) {
        fs::remove_all(retired);
        fs::rename(overviews, retired);
        retired_moved = true;
      }
      fs::rename(staging, overviews);
    } catch (...) {
      // Clear the staging debris (best-effort, non-throwing — a throwing cleanup
      // would mask the original failure and skip the restore below), then
      // restore. Restore only if the retire rename actually completed —
      // otherwise overviews/ was never moved and is still in place.
      std::error_code ec;
      fs::remove_all(staging, ec);
      if (retired_moved) {
        // Non-throwing, unlike the rest of this block: a throw here would replace
        // the original swap diagnostic with a restore error, and the operator
        // would never learn WHY the swap failed. If the restore does fail, the
        // previous sidecar is sitting at overviews.old/ and nothing else would
        // say so.
        std::error_code restore_ec;
        fs::rename(retired, overviews, restore_ec);
        if (restore_ec) {
          std::cerr << "warning: could not restore the previous sidecar: " <<
            restore_ec.message() << "; it is intact at " << retired.string() <<
            " — rename it back to " << overviews.string() << " by hand" <<
            std::endl;
        }
      }
      throw;
    }
  }
  // Best-effort: the swap already succeeded — a throwing cleanup here would report
  // the build as failed with the new sidecar live, prompting a needless re-run.
  // Warn so the leftover overviews.old/ is still visible.
  std::error_code ec;
  fs::remove_all(retired, ec);
  if (ec) {
    std::cerr << "warning: could not remove retired sidecar " <<
      retired.string() << ": " << ec.message() << std::endl;
  }
  result.sidecar_replaced = true;
  return result;
}

}  // namespace marine_bathymetry_store
