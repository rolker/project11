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

/// @file
/// @brief Durable `processed` Tier-2 build (ADR-0006 D4/D5/D7, issue #184).
///
/// Reads a Tier-1 `.sst1` archive and composites a **best-source** mosaic: per
/// GGGS cell it keeps the highest-quality look's `{intensity, quality,
/// source-id}` (3-band `uint16` GeoTIFF tiles), and writes a `registry.json`
/// sidecar resolving the per-cell source index to its provenance (ADR-0005). This
/// is the durable layer the live draft is eventually promoted into.
///
/// v1 quality is a **flat-bottom grazing-angle score** peaking mid-swath
/// (`sin(2·grazing)`) — nadir and far-range are low, so a neighbouring line's
/// mid-swath look wins and fills the nadir gap automatically (ADR-0006 D5). The
/// full GeoCoder radiometry (beam pattern, slope, EGN) is a later phase; this
/// establishes the compositing + provenance contract.

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_tiled_raster_store/tile_io.hpp"

#include "marine_backscatter/processed_accumulator.hpp"
#include "marine_backscatter/quality.hpp"
#include "marine_backscatter/registry.hpp"
#include "marine_sidescan_mosaic/bathy_dem.hpp"
#include "marine_sidescan_mosaic/projection.hpp"
#include "marine_sidescan_mosaic/tier1.hpp"

using namespace marine_backscatter;       // NOLINT(build/namespaces) — shared engine.
using namespace marine_sidescan_mosaic;   // NOLINT(build/namespaces) — local tool.

namespace
{
/// @brief Sticky `projection_mode` for a store an operator deliberately mixed
///   (`--accumulate --allow-mixed-projection` across modes). It is never
///   downgraded back to `flat`/`dem`: the store holds samples placed both ways
///   for good, and a later run must be told so rather than reading a pure mode.
constexpr char kMixedMode[] = "mixed";

// Presence test for a valueless boolean flag (e.g. --accumulate).
bool hasFlag(int argc, char ** argv, const std::string & flag)
{
  for (int i = 1; i < argc; ++i) {
    if (flag == argv[i]) {
      return true;
    }
  }
  return false;
}

/// @brief The value following @p flag, or @p dflt when the flag is absent.
///
/// A flag in the **last** argv slot has no value: exit 2 rather than silently
/// falling through to the default. `... --bathy-store` with the path lost to a
/// shell slip must not run a full flat-bottom build and exit 0 (#297 review).
///
/// A flag followed by **another flag** is the same slip one slot along
/// (`--campaign --platform bizzy` would otherwise record the campaign as
/// `"--platform"` in `registry.json`), so a value beginning with `--` is refused
/// too. Single-dash tokens are left alone: those are negative numbers.
std::string argValue(int argc, char ** argv, const std::string & flag, const std::string & dflt)
{
  for (int i = 1; i < argc; ++i) {
    if (flag == argv[i]) {
      if (i + 1 >= argc) {
        std::cerr << "error: " << flag << " requires a value, but none followed it\n";
        std::exit(2);
      }
      const std::string value = argv[i + 1];
      if (value.rfind("--", 0) == 0) {
        std::cerr << "error: " << flag << " requires a value, but the next argument is "
                  << "another flag ('" << value << "').\n"
                  << "  A value beginning with '--' is not accepted: it is almost always a\n"
                  << "  dropped argument, and taking it literally would write the flag name\n"
                  << "  into the store's provenance.\n";
        std::exit(2);
      }
      return value;
    }
  }
  return dflt;
}

// Both parsers refuse TRAILING GARBAGE, not merely an unparseable value: `stoi`/
// `stod` stop at the first character they cannot use, so `--min-dem-coverage 0.9,`
// (a stray separator) silently became 0.9 and `--level 13x` became 13 — the same
// dropped/mistyped-argument class `argValue`'s flag-shaped-value refusal covers,
// and it would be recorded in the store's provenance (#297 round-3 review).
int toInt(const std::string & s, const std::string & flag)
{
  try {
    std::size_t used = 0;
    const int value = std::stoi(s, &used);
    if (used == s.size()) {
      return value;
    }
  } catch (const std::exception &) {
    // fall through to the same refusal.
  }
  std::cerr << "error: expected an integer for " << flag << ", got '" << s << "'\n";
  std::exit(2);
}

double toDouble(const std::string & s, const std::string & flag)
{
  try {
    std::size_t used = 0;
    const double value = std::stod(s, &used);
    if (used == s.size()) {
      return value;
    }
  } catch (const std::exception &) {
    // fall through to the same refusal.
  }
  std::cerr << "error: expected a number for " << flag << ", got '" << s << "'\n";
  std::exit(2);
}

/// @brief Every flag this tool accepts, split by whether it takes a value.
///
/// Kept beside `runTool`'s parsing so an added flag that misses this list fails
/// loudly in its own test rather than silently (see @ref refuseUnknownArguments).
const std::vector<std::string> & valueFlags()
{
  static const std::vector<std::string> flags{
    "--level", "--no-nadir-policy", "--tx-beamwidth-fallback-rad", "--source-id",
    "--platform", "--sensor", "--sensor-class", "--campaign", "--bathy-store",
    "--bathy-layers", "--min-dem-coverage", "--datum-check-warn-m", "--bathy-cache-tiles"};
  return flags;
}

const std::vector<std::string> & booleanFlags()
{
  static const std::vector<std::string> flags{"--accumulate", "--allow-mixed-projection"};
  return flags;
}

/// @brief Refuse any argument past the two positionals that is not a recognised
///   flag or that flag's value.
///
/// Unknown `--` tokens used to be ignored, which is the same dropped/mistyped
/// argument class the value and positional checks already cover — and the most
/// consequential instance of it: removing `--overwrite` in round 3 turned
/// `--accumulate --overwrite` from an explicit "opposites" refusal into a silent
/// accumulate at exit 0, and `--bathy-stor /path` runs a full FLAT build the
/// operator believes is DEM-corrected, recorded as `flat` in a store they will read
/// as orthorectified (#297 round-4 review).
///
/// A stray non-flag token is refused for the same reason: the two positionals come
/// first, so a third bare word is a mistyped flag or a lost value, never something
/// this tool has a use for.
///
/// @return exit code 2 to return immediately, or nullopt to proceed.
std::optional<int> refuseUnknownArguments(int argc, char ** argv)
{
  const auto known = [](const std::vector<std::string> & flags, const std::string & token) {
      return std::find(flags.begin(), flags.end(), token) != flags.end();
    };
  for (int i = 3; i < argc; ++i) {   // argv[1]/argv[2] are the checked positionals.
    const std::string token = argv[i];
    if (known(valueFlags(), token)) {
      // Consume the value — but only when it looks like one. A missing or
      // flag-shaped value is argValue's refusal to make, and it names the flag that
      // lost its argument; swallowing the next token here would report the slip as
      // an "unrecognised argument" one slot further along instead.
      if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        ++i;
      }
      continue;
    }
    if (known(booleanFlags(), token)) {
      continue;
    }
    if (token == "--overwrite") {
      std::cerr << "error: --overwrite is not a flag: this tool never deletes a store.\n"
                << "  Clearing a previous build is the operator's own explicit act — remove the\n"
                << "  *.tif tiles, registry.json, projection.json, and the derived overviews/\n"
                << "  directory by hand, or build into a fresh out_dir. Pass --accumulate to\n"
                << "  composite into the existing store instead.\n";
      return 2;
    }
    std::cerr << "error: unrecognised argument '" << token << "'.\n"
              << "  Everything after the two positional arguments must be a flag this tool\n"
              << "  knows (or that flag's value). An unknown token is a typo or a dropped\n"
              << "  argument — ignoring it would run something other than what was asked and\n"
              << "  record THAT in the store's provenance. Run with no arguments for the\n"
              << "  usage list.\n";
    return 2;
  }
  return std::nullopt;
}

/// @brief JSON string escape for the values written into the sidecar (store paths
///   and the layer list).
///
/// Control characters are escaped too, not merely unexpected: a path holding a
/// newline or a quote would otherwise emit a file that is not valid JSON at all —
/// which the reader answers with `kUnreadable`, freezing the store out of every
/// later `--accumulate` until it is repaired by hand.
std::string jsonEscape(const std::string & s)
{
  static const char * const kHex = "0123456789abcdef";
  std::string out;
  out.reserve(s.size());
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out += "\\u00";
          out.push_back(kHex[(static_cast<unsigned char>(c) >> 4) & 0xF]);
          out.push_back(kHex[static_cast<unsigned char>(c) & 0xF]);
        } else {
          out.push_back(c);
        }
        break;
    }
  }
  return out;
}

/// @brief One contributing run's DEM coverage: a fraction, or `null` (a flat run).
using CoverageElement = std::optional<double>;

/// @brief The elements of a recorded coverage list, VALIDATED.
///
/// @param text the array's inner text without its brackets ("0.99, null, 0.03"),
///   or a bare v1 scalar ("0.99" / "null"). Whitespace-only (or empty) is an empty
///   list, not a failure — `[]` is a well-formed record of "no run yet".
/// @return nullopt when ANY element is not `null` or a finite fraction in [0, 1].
///
/// Validated rather than carried forward verbatim, for two reasons the round-4
/// review found. An unparseable element used to be silently skipped by the
/// worst-of aggregation, which makes `dem_coverage` read *rosier* than the store
/// deserves — the opposite of the absolute floor the field is documented to be.
/// And a verbatim carry-forward re-emitted whatever it was handed, so a hand-edited
/// `[0.5,]` or `[ ]` stayed in the file forever, leaving a sidecar that `jq`,
/// `json.load`, and #179's registry merge all refuse. An invalid list now makes the
/// sidecar `kUnreadable`, which is the safe answer this reader already commits to.
std::optional<std::vector<CoverageElement>> parseCoverageElements(const std::string & text)
{
  const auto trim = [](const std::string & s) {
      std::size_t b = 0, e = s.size();
      while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) {++b;}
      while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {--e;}
      return s.substr(b, e - b);
    };
  std::vector<CoverageElement> out;
  if (trim(text).empty()) {
    return out;
  }
  std::istringstream elements(text);
  std::string element;
  while (std::getline(elements, element, ',')) {
    const std::string value = trim(element);
    if (value == "null") {
      out.emplace_back(std::nullopt);
      continue;
    }
    try {
      std::size_t used = 0;
      const double parsed = std::stod(value, &used);
      if (used != value.size() || !std::isfinite(parsed) || parsed < 0.0 || parsed > 1.0) {
        return std::nullopt;
      }
      out.emplace_back(parsed);
    } catch (const std::exception &) {
      return std::nullopt;   // empty element ("0.5,") or plain garbage.
    }
  }
  return out;
}

/// @brief Format a `[0, 1]` coverage figure for the sidecar at full round-trip
///   precision.
///
/// `operator<<`'s default 6-significant-digit stream precision rounds any
/// coverage above ~0.9999995 to the literal `1`, which reads back as PERFECT
/// DEM coverage — the exact "rosier than the store deserves" laundering the
/// worst-of aggregate exists to prevent. `std::to_chars` with the default
/// (general) format writes the SHORTEST decimal that reads back as the exact
/// same `double` — full precision where it matters (e.g. `0.9999995`) without
/// spelling ordinary figures like `0.03` as `0.029999999999999999` the way a
/// fixed `max_digits10` stream precision would (#297 round-5 review).
std::string formatCoverage(double value)
{
  std::array<char, 32> buf{};
  const auto result = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  return std::string(buf.data(), result.ptr);
}

/// @brief Record this run's projection mode next to `registry.json` (#297).
///
/// The mode cannot live in `registry.json` yet: `writeRegistry` is a fixed-shape
/// single-source writer in `marine_backscatter`, and widening it is a package API
/// change that belongs with #179's append-only registry merge. Until then the tool
/// writes its own sidecar, which every run (flat included) emits.
/// All string arguments are passed **already JSON-escaped** (`jsonEscape`), so a
/// value preserved verbatim from the previous sidecar can be written straight back
/// without an unescape/re-escape round trip.
///
/// @param dem_coverage_history the complete per-run coverage list for this store —
///   every earlier run's figure plus this run's, in order (`nullopt` = a flat run).
///   `dem_coverage` is derived from it as the WORST figure any contributing run
///   achieved, so a single-number reader can never get a rosier answer than the
///   store deserves.
/// @return true when the sidecar is on disk complete; false on any I/O failure
///   (open, write, the flush that close() performs — a full filesystem fails only
///   there, so the stream state is checked after the close, not before — or the
///   swap).
///
/// The write is STAGE-AND-SWAP: a sibling `projection.json.tmp` is written, closed,
/// checked, and only then renamed over the live sidecar (an atomic same-directory
/// rename). Truncating the live file in place put the store one failed `close()` —
/// exactly the full-filesystem case above — from having no readable sidecar at all,
/// and under `--accumulate` this file is the sole durable record of the whole
/// coverage history plus the bathy-store provenance. A store with an unreadable
/// sidecar is `kUnreadable` forever: every later `--accumulate` into it is refused.
/// `build_sidescan_overviews` stages-and-swaps its own sidecar for the same reason
/// (#297 round-4 review).
bool writeProjectionSidecar(
  const std::string & out_dir, const std::string & mode_json,
  const std::string & bathy_store_json, const std::string & bathy_layers_json,
  const std::vector<CoverageElement> & dem_coverage_history)
{
  // The worst figure across every run that contributed. An --accumulate used to
  // stamp the sidecar with THIS run's coverage, so folding a 0.99 bag into a 0.03
  // store made the store read 0.99 — laundering the exact hazard the figure exists
  // to record (#297 round-3 review).
  std::optional<double> worst;
  for (const auto & element : dem_coverage_history) {
    if (element) {
      worst = worst ? std::min(*worst, *element) : *element;
    }
  }

  const std::filesystem::path path = std::filesystem::path(out_dir) / "projection.json";
  const std::filesystem::path staging = std::filesystem::path(out_dir) / "projection.json.tmp";
  const auto discardStaging = [&staging]() {
      std::error_code ec;
      std::filesystem::remove(staging, ec);   // best-effort: the live file is intact.
    };
  {
    std::ofstream out(staging, std::ios::trunc);
    if (!out) {
      discardStaging();
      return false;
    }
    out << "{\n"
        << "  \"version\": 2,\n"
        << "  \"projection_mode\": \"" << mode_json << "\",\n"
        << "  \"bathy_store\": \"" << bathy_store_json << "\",\n"
        << "  \"bathy_layers\": \"" << bathy_layers_json << "\",\n"
        // The DEM coverage this store was built at. `--min-dem-coverage 0` is an
        // explicit opt-in to a partial run, and without a figure a 3 %- and a 99 %-
        // corrected store both read as plain "dem" downstream (#297 review). `null`
        // when no contributing run had one (a flat store).
        << "  \"dem_coverage\": ";
    if (worst) {
      out << formatCoverage(*worst);
    } else {
      out << "null";
    }
    // Per-run history, oldest first — one element per run that wrote this store
    // (`null` for a flat one). The aggregate above is a floor over it; the list is
    // what survives an --accumulate, so no run's figure is ever dropped.
    out << ",\n"
        << "  \"dem_coverage_history\": [";
    for (std::size_t i = 0; i < dem_coverage_history.size(); ++i) {
      if (i > 0) {
        out << ", ";
      }
      if (dem_coverage_history[i]) {
        out << formatCoverage(*dem_coverage_history[i]);
      } else {
        out << "null";
      }
    }
    out << "]\n"
        << "}\n";
    out.close();
    if (out.fail()) {
      discardStaging();
      return false;
    }
  }
  std::error_code ec;
  std::filesystem::rename(staging, path, ec);
  if (ec) {
    discardStaging();
    return false;
  }
  return true;
}

/// @brief What @p out_dir's sidecar says about how its store was projected.
///
/// `kMissing` (no sidecar at all) is a **pre-#297 store**, which is flat-built by
/// construction. `kUnreadable` (present but unopenable, truncated, or carrying no
/// parseable `projection_mode`) is an unknown mode and must never be silently
/// read as flat — a DEM store whose sidecar was truncated would otherwise accept
/// a flat `--accumulate`.
enum class SidecarState { kMissing, kUnreadable, kOk };

struct ProjectionSidecar
{
  SidecarState state = SidecarState::kMissing;
  std::string mode;           ///< only meaningful when `state == kOk`.
  std::string bathy_store;    ///< as recorded (still JSON-escaped); "" if absent.
  std::string bathy_layers;   ///< as recorded (still JSON-escaped); "" if absent.
  /// The recorded per-run coverage list, oldest first (`nullopt` = a flat run);
  /// empty when the store records none. Carried forward in full so no run's figure
  /// is ever dropped — including a **v1** sidecar's single scalar, which is seeded
  /// in as the first element (see @ref readProjectionSidecar).
  std::vector<CoverageElement> dem_coverage_history;
};

/// @brief The escaped inner text of the JSON string starting at @p i (which must
///   index its opening quote). @p i is left on the closing quote.
///
/// The text is returned **still escaped**, so a recorded value compares directly
/// against `jsonEscape(candidate)` without a round trip.
std::optional<std::string> scanJsonString(const std::string & text, std::size_t * i)
{
  std::string value;
  for (std::size_t j = *i + 1; j < text.size(); ++j) {
    if (text[j] == '\\') {
      if (j + 1 >= text.size()) {
        return std::nullopt;   // trailing backslash: truncated.
      }
      value.push_back(text[j]);
      value.push_back(text[j + 1]);
      ++j;
      continue;
    }
    if (text[j] == '"') {
      *i = j;
      return value;
    }
    value.push_back(text[j]);
  }
  return std::nullopt;   // unterminated.
}

/// @brief Parse the sidecar's flat JSON object into `key -> raw value text`.
///
/// A real (if small) parser rather than a per-line key search: searching for
/// `"projection_mode"` anywhere on a line finds it INSIDE a value too, so a
/// `--bathy-store` path holding a quoted `"projection_mode": "flat"` used to emit a
/// later `bathy_store` line whose spoofed mode overwrote the real one — a `dem`
/// store read back as `flat`. Scanning the object structurally cannot confuse a
/// value for a key, and it is also whitespace-agnostic: a hand-repaired sidecar
/// written on a single line parses all of its fields, not just the first (#297
/// round-3 review).
///
/// String values are returned still-escaped (see @ref scanJsonString); numbers,
/// literals, and arrays are returned as their verbatim source text. The FIRST
/// occurrence of a key wins, so an appended duplicate cannot override a recorded
/// value. Nested objects are refused — this sidecar is flat by contract, and an
/// unparseable sidecar is an unknown mode, which is the safe answer.
std::optional<std::map<std::string, std::string>> parseFlatJsonObject(const std::string & text)
{
  std::map<std::string, std::string> out;
  const auto skipWs = [&text](std::size_t i) {
      while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
        ++i;
      }
      return i;
    };
  std::size_t i = skipWs(0);
  if (i >= text.size() || text[i] != '{') {
    return std::nullopt;
  }
  i = skipWs(i + 1);
  if (i < text.size() && text[i] == '}') {
    return out;   // an empty object is well-formed, just modeless.
  }
  while (i < text.size()) {
    if (text[i] != '"') {
      return std::nullopt;
    }
    const auto key = scanJsonString(text, &i);
    if (!key) {
      return std::nullopt;
    }
    i = skipWs(i + 1);
    if (i >= text.size() || text[i] != ':') {
      return std::nullopt;
    }
    i = skipWs(i + 1);
    if (i >= text.size()) {
      return std::nullopt;
    }
    std::string value;
    if (text[i] == '"') {
      const auto parsed = scanJsonString(text, &i);
      if (!parsed) {
        return std::nullopt;
      }
      value = *parsed;
      ++i;
    } else if (text[i] == '[') {
      const std::size_t start = i;
      bool closed = false;
      for (++i; i < text.size(); ++i) {
        if (text[i] == '"') {
          if (!scanJsonString(text, &i)) {
            return std::nullopt;
          }
          continue;   // the loop's ++i steps past the closing quote.
        }
        if (text[i] == '[' || text[i] == '{') {
          return std::nullopt;   // flat by contract: no nesting inside the array.
        }
        if (text[i] == ']') {
          closed = true;
          break;
        }
      }
      if (!closed) {
        return std::nullopt;
      }
      value = text.substr(start, i - start + 1);
      ++i;
    } else if (text[i] == '{') {
      return std::nullopt;   // flat by contract.
    } else {
      const std::size_t start = i;
      while (i < text.size() && text[i] != ',' && text[i] != '}' &&
        !std::isspace(static_cast<unsigned char>(text[i])))
      {
        ++i;
      }
      value = text.substr(start, i - start);
      if (value.empty()) {
        return std::nullopt;
      }
    }
    out.emplace(*key, value);   // first occurrence wins.
    i = skipWs(i);
    if (i < text.size() && text[i] == ',') {
      i = skipWs(i + 1);
      continue;
    }
    if (i < text.size() && text[i] == '}') {
      return out;
    }
    return std::nullopt;
  }
  return std::nullopt;   // ran off the end: truncated.
}

/// @brief Read @p out_dir's `projection.json`.
///
/// Sidecar SCHEMA VERSIONS. v1 (an intermediate build of #297; `projection.json`
/// never shipped, so only stores built by such a build are v1) recorded a single
/// scalar `dem_coverage` and no history. v2 records the full per-run
/// `dem_coverage_history`, with the scalar derived from it as the worst element.
/// Reading only the array would have made a v2 `--accumulate` into a v1 store
/// rewrite the history as *this run alone* — folding a 0.99 run into a store
/// recorded at 0.03 would read 0.99, which is verbatim the laundering the history
/// exists to stop. So the scalar SEEDS the history whenever the array is absent
/// (#297 round-4 review). A version this build does not know is `kUnreadable`
/// rather than read with v2 assumptions: an unknown schema may mean fields whose
/// meaning has changed, and "unknown" is the answer this reader is built to give
/// safely.
ProjectionSidecar readProjectionSidecar(const std::string & out_dir)
{
  ProjectionSidecar out;
  const std::filesystem::path path = std::filesystem::path(out_dir) / "projection.json";
  std::error_code ec;
  const bool path_exists = std::filesystem::exists(path, ec);
  if (ec) {
    // The stat itself failed (EACCES/ELOOP/ENOTDIR/...), not "no file here" —
    // fail-safe as unreadable, never as missing: kMissing reads downstream as
    // "a pre-#297 flat build", which would let an unreadable `dem` store's
    // flat `--accumulate` through at exit 0 (#297 round-5 review).
    out.state = SidecarState::kUnreadable;
    return out;
  }
  if (!path_exists) {
    out.state = SidecarState::kMissing;
    return out;
  }
  out.state = SidecarState::kUnreadable;   // until a mode is actually parsed.
  // Only a regular file is read, and only up to a cap. A fifo left in the store's
  // place would otherwise hang the tool with no message, and a huge file would take
  // it out with bad_alloc; both are "the sidecar cannot be read", which this
  // function already has a safe answer for (#297 round-4 review).
  constexpr std::uintmax_t kMaxSidecarBytes = 1u << 20;   // 1 MiB; ours is ~200 B.
  if (!std::filesystem::is_regular_file(path, ec) || ec) {
    return out;
  }
  if (std::filesystem::file_size(path, ec) > kMaxSidecarBytes || ec) {
    return out;
  }
  std::ifstream in(path);
  if (!in) {
    return out;
  }
  std::ostringstream text;
  text << in.rdbuf();
  if (in.bad()) {
    return out;
  }
  const auto fields = parseFlatJsonObject(text.str());
  if (!fields) {
    // Truncated, corrupt, or not the flat object this tool writes: an unknown
    // mode, never "flat".
    return out;
  }
  const auto version = fields->find("version");
  if (version != fields->end()) {
    try {
      std::size_t used = 0;
      const int recorded = std::stoi(version->second, &used);
      if (used != version->second.size() || recorded < 1 || recorded > 2) {
        return out;   // a schema this build does not know: unknown, not flat.
      }
    } catch (const std::exception &) {
      return out;
    }
  }
  const auto store = fields->find("bathy_store");
  if (store != fields->end()) {
    out.bathy_store = store->second;
  }
  const auto layers = fields->find("bathy_layers");
  if (layers != fields->end()) {
    out.bathy_layers = layers->second;
  }
  const auto history = fields->find("dem_coverage_history");
  const auto scalar = fields->find("dem_coverage");
  if (history != fields->end()) {
    if (history->second.size() < 2 || history->second.front() != '[' ||
      history->second.back() != ']')
    {
      return out;   // present but not an array: unreadable, not empty.
    }
    const auto elements =
      parseCoverageElements(history->second.substr(1, history->second.size() - 2));
    if (!elements) {
      return out;
    }
    out.dem_coverage_history = *elements;
  } else if (scalar != fields->end()) {
    // v1 (or a v2 sidecar missing its array): the single figure IS the history so
    // far. Dropping it would let this run's figure stand alone as the store's whole
    // provenance — the laundering the history exists to stop.
    const auto elements = parseCoverageElements(scalar->second);
    if (!elements) {
      return out;
    }
    out.dem_coverage_history = *elements;
  }
  const auto mode = fields->find("projection_mode");
  // An empty (or absent) mode is present-but-unknown, not a flat store: leave the
  // state at kUnreadable.
  if (mode != fields->end() && !mode->second.empty()) {
    out.mode = mode->second;
    out.state = SidecarState::kOk;
  }
  return out;
}

/// @brief Does @p out_dir already hold value tiles from an earlier build?
///
/// The provenance guards must key on the **tiles**, not on `registry.json` alone:
/// the fold loop reads tile files, so a store whose registry never landed (an
/// interrupted or failed run) still has coverage that a later run would composite
/// into or overwrite (#297 round-2 review). Any read problem answers "no tiles" —
/// the caller's other checks still apply, and a directory that cannot be listed
/// fails loudly at `saveTiles` anyway.
bool hasTileFiles(const std::string & out_dir)
{
  std::error_code ec;
  if (!std::filesystem::is_directory(out_dir, ec) || ec) {
    return false;
  }
  std::filesystem::directory_iterator it(out_dir, ec);
  if (ec) {
    return false;
  }
  // Stepped by hand: a range-`for` uses the THROWING `operator++`, so a directory
  // that becomes unreadable mid-walk would escape as an exception instead of the
  // documented "any read problem answers no tiles" (#297 round-3 review).
  const std::filesystem::directory_iterator end;
  while (it != end) {
    const auto & entry = *it;
    if (entry.is_regular_file(ec) && !ec && entry.path().extension() == ".tif") {
      return true;
    }
    it.increment(ec);
    if (ec) {
      return false;
    }
  }
  return false;
}

/// @brief @p path in a form two spellings of the same location share.
///
/// `weakly_canonical` where the filesystem can answer (it resolves `..`, symlinks,
/// and a relative prefix against the CWD), else the purely lexical normalisation —
/// never a throw, and never an empty answer for a non-empty input.
std::string normalizedPath(const std::string & path)
{
  if (path.empty()) {
    return path;
  }
  std::error_code ec;
  const auto resolved = std::filesystem::weakly_canonical(std::filesystem::path(path), ec);
  if (ec || resolved.empty()) {
    return std::filesystem::path(path).lexically_normal().string();
  }
  return resolved.string();
}

/// @brief Everything the provenance guards need to know about `out_dir`.
struct OutputDirState
{
  bool has_tiles = false;
  bool has_registry = false;
  bool has_sidecar = false;
  bool has_overviews = false;
  bool populated = false;
};

/// @brief Inspect @p out_dir for the provenance guards (tiles + both sidecars).
///
/// `overviews/` counts as populated even though it is derived: the refusal message
/// names it among the files to delete, and an operator who follows those
/// instructions but misses the directory would otherwise get a fresh store beside a
/// stale pyramid built from the deleted tiles, at exit 0 (#297 round-4 review).
OutputDirState inspectOutputDir(const std::string & out_dir)
{
  OutputDirState state;
  const std::filesystem::path dir(out_dir);
  state.has_tiles = hasTileFiles(out_dir);
  state.has_registry = std::filesystem::exists(dir / "registry.json");
  state.has_sidecar = std::filesystem::exists(dir / "projection.json");
  std::error_code ec;
  state.has_overviews = std::filesystem::is_directory(dir / "overviews", ec) && !ec;
  state.populated =
    state.has_tiles || state.has_registry || state.has_sidecar || state.has_overviews;
  return state;
}

/// @brief What the sidecar this run writes carries, once the prior store has had
///   its say. A field this run has nothing to say about must keep the previous
///   run's value rather than blanking it (#297 round-3 review).
struct SidecarCarryForward
{
  std::string mode;                  ///< this run's mode, or `mixed` once promoted.
  std::string bathy_store_json;      ///< JSON-escaped; see @ref jsonEscape.
  std::string bathy_layers_json;     ///< JSON-escaped.
  /// Earlier runs' coverage elements, oldest first; empty if none (this run's is
  /// appended at write time).
  std::vector<CoverageElement> dem_coverage_history;
};

/// @brief The output-directory provenance guards, run before anything is decoded.
///
/// @param carry in/out: seeded with this run's own values, adjusted here for what
///   the existing store already records.
/// @return an exit code to return from the tool immediately, or `nullopt` to proceed.
std::optional<int> checkOutputDirGuards(
  const std::string & out_dir, const OutputDirState & state, bool accumulate,
  bool dem_mode, const std::string & projection_mode, bool allow_mixed_projection,
  const std::string & bathy_store, const std::string & bathy_layers, int source_id_arg,
  SidecarCarryForward * carry)
{
  const std::filesystem::path registry_file = std::filesystem::path(out_dir) / "registry.json";
  const bool out_dir_has_tiles = state.has_tiles;
  const bool out_dir_has_registry = state.has_registry;
  const bool out_dir_populated = state.populated;
  // Projection-mode guard (#297). ADR-0005 D2/D6 make per-cell provenance exactly
  // one interned source index resolved through the registry — nothing per cell
  // records HOW a sample was placed, so this is that contract applied to placement.
  // (D8 is the "both stores adopt this" decision, not a corruption rule.)
  // --accumulate folds this run into the tiles already on disk, and its only
  // provenance guard is the source_id match below — which a DEM-corrected re-run
  // with the same --source-id passes. Compositing corrected
  // and mis-placed samples into the same cells is unrecoverable: the source band
  // records the same source either way. So refuse the mix, fail-fast before
  // decoding. A store with no sidecar predates mode recording and is therefore
  // flat-built (every store built before this flag existed is).
  //
  // "Already a store" is decided by the TILES (plus either provenance file), not by
  // registry.json alone: a run interrupted between the tile write and the registry
  // write leaves coverage on disk that a later --accumulate would otherwise fold
  // into with both guards skipped, and then re-stamp with a pure mode (#297 round-2
  // review).

  // The same protection for a re-run WITHOUT --accumulate. saveTiles overwrites
  // only the tiles this run touches, so a plain re-run into a populated directory
  // leaves the previous build's untouched tiles in place — a store that is
  // materially mixed (this run's placement in some cells, the previous run's in
  // others) while the sidecar is re-stamped with this run's pure mode, at exit 0
  // and with no flag. Refuse (#297 round-2 review).
  //
  // The refusal is PLAIN — the tool has no store deleter, by design. A `--overwrite`
  // flag was written and then removed in round 3: every in-tool deletion path found
  // a way to destroy a store the operator did not mean to lose (a run that placed
  // nothing still cleared; a transposed path argument matched another store's tile
  // names). Stores are data-of-record here, so removing one is the operator's own
  // explicit act, outside this tool. Name the files instead (#297 round-3 review).
  if (!accumulate && out_dir_populated) {
    const std::filesystem::path dir(out_dir);
    std::cerr << "error: " << out_dir << " already holds a build (tiles/registry/"
              << "projection.json/overviews) and this run does not pass --accumulate.\n"
              << "  Writing into it would leave the previous build's untouched tiles beside\n"
              << "  this run's, so the store would hold samples placed two different ways\n"
              << "  while projection.json recorded a single pure mode. Per-cell provenance\n"
              << "  cannot tell them apart afterwards (ADR-0005 D2/D6). Choose one:\n"
              << "    --accumulate   composite into the existing store (mode-guarded), or\n"
              << "    a fresh out_dir, or\n"
              << "    delete the previous build yourself first — this tool never removes a\n"
              << "    store. It is these files, and nothing else in the directory:\n"
              << "      " << (dir / "*.tif").string() << "\n"
              << "      " << (dir / "registry.json").string() << "\n"
              << "      " << (dir / "projection.json").string() << "\n"
              << "    (a derived " << (dir / "overviews").string()
              << " sidecar is rebuilt from the tiles, so\n"
              << "    delete it too when you clear the store by hand).\n";
    return 2;
  }
  if (accumulate && out_dir_populated) {
    const ProjectionSidecar recorded = readProjectionSidecar(out_dir);
    if (recorded.state == SidecarState::kUnreadable) {
      std::cerr << "error: --accumulate: the store in " << out_dir << " has a projection.json "
                << "that could not be read or carries no projection_mode.\n"
                << "  Its projection mode is therefore UNKNOWN, and an unknown mode is not\n"
                << "  assumed to be flat: folding this run in could composite flat-bottom and\n"
                << "  DEM-orthorectified samples into the same cells, which per-cell provenance\n"
                << "  cannot tell apart afterwards. Repair or remove the sidecar (a store with\n"
                << "  NO projection.json is a pre-#297 flat build), or use a fresh out_dir.\n";
      return 2;
    }
    const std::string existing_mode =
      recorded.state == SidecarState::kOk ? recorded.mode : "flat";
    const std::string mode_note = recorded.state == SidecarState::kMissing ?
      " (no projection.json — a pre-#297 flat build)" : "";
    if (existing_mode != projection_mode) {
      std::cerr << (allow_mixed_projection ? "warning" : "error")
                << ": --accumulate: the store in " << out_dir << " was built with "
                << "projection_mode '" << existing_mode << "'" << mode_note
                << ", but this run uses '" << projection_mode << "'.\n"
                << "  Compositing flat-bottom and DEM-orthorectified samples into the same\n"
                << "  cells is unrecoverable: per-cell provenance records only the source id,\n"
                << "  which is identical either way. Regenerate into a fresh out_dir instead\n"
                << "  of accumulating, or pass --allow-mixed-projection to accept the mix.\n";
      if (!allow_mixed_projection) {
        return 2;
      }
      // The mix is accepted, so the store now holds samples placed BOTH ways and
      // stays that way forever: record `mixed`, not this run's mode. Rewriting it
      // as pure `dem`/`flat` would launder the provenance — every later
      // --accumulate would then pass the guard silently (#297 review).
      carry->mode = kMixedMode;
      std::cerr << "warning: " << out_dir << " is now a MIXED-projection store; its "
                << "projection.json records '" << kMixedMode << "' permanently, and every "
                << "later --accumulate into it needs --allow-mixed-projection.\n";
    } else if (existing_mode == kMixedMode) {
      // Unreachable while kMixedMode is neither "flat" nor "dem" (the branch above
      // fires first), but keep the stickiness explicit rather than incidental.
      carry->mode = kMixedMode;
    }

    // A DIFFERENT DEM, whatever the mode verdict was. Two DEM placements against
    // different bathy stores (or a different layer search order) are not the same
    // placement: another vintage, extent, or vertical datum moves the samples, and
    // the composited cells again record only the source id. The mode guard cannot
    // see this, so report it — as a warning, not a refusal: re-running against an
    // UPDATED store is a legitimate, common workflow, and only the operator knows
    // whether the two surfaces agree. Checked OUTSIDE the mode chain (a refused
    // mismatch has already returned above), so a `mixed` store — which is exactly
    // the store most likely to be built from several surfaces — is not the one
    // store that never gets the warning.
    //
    // It compares the store PATH, normalised (`./stores/bathy` and its absolute
    // form are the same store; without that the check cried wolf on every relative
    // invocation). What a path comparison cannot see is the same path RE-IMPORTED
    // with new content — a store fingerprint in the registry is what would catch
    // that, and it belongs with #179 (#297 round-3 review).
    const std::string recorded_store = recorded.bathy_store;
    const std::string recorded_layers = recorded.bathy_layers;
    if (dem_mode && !recorded_store.empty() &&
      (normalizedPath(recorded_store) != normalizedPath(jsonEscape(bathy_store)) ||
      recorded_layers != jsonEscape(bathy_layers)))
    {
      std::cerr << "warning: --accumulate: the store in " << out_dir << " was "
                << "DEM-orthorectified against '" << recorded_store << "' (layers '"
                << recorded_layers << "'), but this run uses '" << bathy_store
                << "' (layers '" << bathy_layers << "').\n"
                << "  The projection-mode guard cannot see this — but samples placed against\n"
                << "  two different bathymetric surfaces (a different vintage, extent, or\n"
                << "  vertical datum) composite into the same cells with nothing per-cell to\n"
                << "  tell them apart. Confirm the two surfaces agree, or build into a fresh\n"
                << "  out_dir. The sidecar will now name THIS run's store.\n";
    }

    // Carry the prior sidecar's provenance forward. Every earlier run's coverage
    // figure stays in the history list — an --accumulate that re-stamped the store
    // with only ITS OWN figure made a 0.99 bag folded into a 0.03 store read 0.99.
    carry->dem_coverage_history = recorded.dem_coverage_history;
    // A run with no bathy store of its own (a flat run promoted to `mixed`) must not
    // blank the store the DEM half was built against: that record is permanent and
    // erasing it silently disables the different-store cross-check for this store
    // forever (#297 round-3 review).
    if (carry->bathy_store_json.empty() && !recorded.bathy_store.empty()) {
      carry->bathy_store_json = recorded.bathy_store;
      carry->bathy_layers_json = recorded.bathy_layers;
      std::cerr << "note: this run names no --bathy-store, so projection.json keeps naming '"
                << recorded.bathy_store << "' (layers '" << recorded.bathy_layers
                << "') — the surface the store's DEM-placed samples were built against.\n";
    }
  }

  // Provenance guard (#253 review; ADR-0005 D2/D6 per-cell provenance, D7's
  // append-only registry merge / #179). foldTile preserves each
  // existing cell's original source band, but writeRegistry() is a write-once
  // single-source writer — so accumulating a DIFFERENT source-id into a store
  // leaves tiles carrying mixed source indices while registry.json names only
  // this run's source, silently corrupting provenance. Until the registry is an
  // append-only merge (#179), refuse the mismatch rather than corrupt it. Fail
  // fast, before decoding.
  if (accumulate) {
    const std::string reg_path = registry_file.string();
    if (out_dir_has_tiles && !out_dir_has_registry) {
      std::cerr << "error: --accumulate: " << out_dir << " holds value tiles but no "
                << "registry.json.\n"
                << "  The tiles' per-cell source indices are unresolvable without it, so this\n"
                << "  run cannot check that its --source-id matches theirs — and writing a\n"
                << "  fresh single-source registry over folded foreign coverage would assert a\n"
                << "  provenance the tiles do not have (ADR-0005 D2/D6). A store in this state\n"
                << "  is an interrupted build: remove it, or regenerate into a fresh out_dir.\n";
      return 2;
    }
    std::ifstream reg(reg_path);
    std::string line;
    while (std::getline(reg, line)) {
      const auto key = line.find("\"source_id\"");
      if (key == std::string::npos) {
        continue;
      }
      const auto colon = line.find(':', key);
      if (colon == std::string::npos) {
        continue;
      }
      const int existing_sid = std::atoi(line.c_str() + colon + 1);
      if (existing_sid > 0 && existing_sid != source_id_arg) {
        std::cerr << "error: --accumulate: existing store registry " << reg_path
                  << " has source_id " << existing_sid << ", but this run uses "
                  << source_id_arg << ".\n"
                  << "  A multi-source registry merge is not yet implemented "
                  << "(ADR-0005 D7 / #179); re-run with --source-id " << existing_sid
                  << " or use a fresh out_dir to avoid corrupting provenance.\n";
        return 2;
      }
      break;   // registry v1 is single-source: first source_id is authoritative.
    }
  }
  return std::nullopt;
}

/// @brief The tool body. `main` wraps it in a last-resort handler (below): the DEM
///   call sites catch their own faults, but `gggs::Level::gridIndex`'s
///   `std::out_of_range` and the throwing `std::filesystem` overloads used around
///   the store can still escape, and `std::terminate` with no diagnostic is not an
///   acceptable ending for a store writer (#297 review).
int runTool(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr <<
      "usage: sidescan_tier2_processed <tier1.sst1> <out_dir>\n"
      "       [--level N] [--no-nadir-policy drop|assume_zero]\n"
      "       [--tx-beamwidth-fallback-rad R]   (along-track footprint when the ping\n"
      "                                          lacks tx_beamwidths; 0=point-deposit)\n"
      "       [--source-id N] [--platform P] [--sensor S] [--sensor-class C] [--campaign X]\n"
      "       [--accumulate]   # composite INTO an existing store (reload+fold each\n"
      "                        # touched tile). A populated out_dir is refused\n"
      "                        # without it: this tool never deletes a store, so\n"
      "                        # clear the previous build by hand or use a fresh dir\n"
      "       [--bathy-store PATH]        # DEM-orthorectify against a bathy store\n"
      "                                   # (omitted = flat-bottom, unchanged)\n"
      "       [--bathy-layers CSV]        # layer search order\n"
      "                                   # (default processed,draft,reference;\n"
      "                                   # naming no layer at all is an error)\n"
      "       [--min-dem-coverage FRAC]   # abort (exit 3) below this DEM hit share\n"
      "                                   # (in [0, 1]; default 0.5; 0 = opt-in, warns)\n"
      "       [--datum-check-warn-m M]    # warn above this mean nadir-vs-DEM offset\n"
      "                                   # (positive metres; default 1.0)\n"
      "       [--bathy-cache-tiles N]     # resident bathy tiles, in [1, 1024]\n"
      "                                   # (default 8, ~14.7 MB each)\n"
      "       [--allow-mixed-projection]  # --accumulate across projection modes anyway\n";
    return 2;
  }
  // The same dropped-argument refusal `argValue` applies to flag VALUES, applied to
  // the two positional slots. `sidescan_tier2_processed in.sst1 --accumulate` would
  // otherwise create a store directory literally named `--accumulate` *and* turn
  // accumulate on, at exit 0 (#297 round-3 review).
  for (const int slot : {1, 2}) {
    if (std::string(argv[slot]).rfind("--", 0) == 0) {
      std::cerr << "error: the " << (slot == 1 ? "tier1.sst1 input" : "out_dir output")
                << " argument is '" << argv[slot] << "', which is a flag, not a path.\n"
                << "  Both positional arguments come first and neither may begin with '--':\n"
                << "  taking this literally would "
                << (slot == 1 ? "look for an archive" : "create a store directory")
                << " by that name while the\n"
                << "  flag itself still took effect.\n";
      return 2;
    }
  }
  if (const auto argument_exit = refuseUnknownArguments(argc, argv)) {
    return *argument_exit;
  }
  const std::string tier1_path = argv[1];
  const std::string out_dir = argv[2];
  const int level_n = toInt(argValue(argc, argv, "--level", "13"), "--level");
  const std::string no_nadir = argValue(argc, argv, "--no-nadir-policy", "drop");
  const double bw_fallback_raw = toDouble(
    argValue(argc, argv, "--tx-beamwidth-fallback-rad", "0.0"), "--tx-beamwidth-fallback-rad");
  // Same validation the live node applies to its fallback (shared helper): drop a
  // non-finite/negative value to 0 (point-deposit), warn on a degrees-for-radians slip.
  bool bw_fallback_suspicious = false;
  const double bw_fallback = sanitizeBeamwidthRad(bw_fallback_raw, &bw_fallback_suspicious);
  if (bw_fallback == 0.0 && bw_fallback_raw != 0.0) {
    std::cerr << "warning: --tx-beamwidth-fallback-rad " << bw_fallback_raw
              << " is not a finite non-negative radian value; ignoring it\n";
  } else if (bw_fallback_suspicious) {
    std::cerr << "warning: --tx-beamwidth-fallback-rad " << bw_fallback
              << " rad (~" << bw_fallback * 180.0 / M_PI << " deg) is implausibly wide for an "
              << "along-track beamwidth; did you pass degrees? expected radians (e.g. 0.00768)\n";
  }
  const int source_id_arg = toInt(argValue(argc, argv, "--source-id", "1"), "--source-id");
  if (source_id_arg < 1 || source_id_arg > 65535) {
    std::cerr << "error: --source-id must be in [1, 65535] (the per-cell band is the uint16 "
      "local index; the wide global id lives in the registry, ADR-0005 D4)\n";
    return 2;
  }
  const auto source_id = static_cast<std::uint16_t>(source_id_arg);
  if (no_nadir == "assume_zero") {
    std::cerr << "warning: --no-nadir-policy assume_zero collapses grazing (altitude 0 -> "
      "grazing 0), so quality floors and best-source degenerates to first-touch; "
      "prefer 'drop' for the processed layer.\n";
  }
  const std::string platform = argValue(argc, argv, "--platform", "bizzyboat");
  const std::string sensor = argValue(argc, argv, "--sensor", "garmin-gcv20");
  const std::string sensor_class = argValue(argc, argv, "--sensor-class", "sidescan");
  const std::string campaign = argValue(argc, argv, "--campaign", "unknown");
  const bool accumulate = hasFlag(argc, argv, "--accumulate");

  // DEM orthorectification (#297). Omitting --bathy-store keeps the unchanged
  // flat-bottom code path. ADR-0006 D6/D9 keep the LIVE draft node flat-bottom by
  // design ("no bathy live"); sidescan_tier2_flat is flat by its own scope — the
  // cheap no-bathy Tier-2 builder — not by those clauses, so only this tool gains
  // the option here.
  const std::string bathy_store = argValue(argc, argv, "--bathy-store", "");
  const std::string bathy_layers = argValue(argc, argv, "--bathy-layers", kDefaultBathyLayers);
  const double min_dem_coverage =
    toDouble(argValue(argc, argv, "--min-dem-coverage", "0.5"), "--min-dem-coverage");
  const double datum_check_warn_m =
    toDouble(argValue(argc, argv, "--datum-check-warn-m", "1.0"), "--datum-check-warn-m");
  const int bathy_cache_tiles = toInt(
    argValue(
      argc, argv, "--bathy-cache-tiles",
      std::to_string(BathyDem::kDefaultCacheTiles)), "--bathy-cache-tiles");
  // Bounded at BOTH ends. The LRU holds `N` × ~14.7 MB of resident tile, so an
  // unbounded N turns a typo into a `std::bad_alloc` mid-run — which the DEM call
  // sites would then report as "the store is corrupt or truncated", sending the
  // operator after the wrong fault (#297 review). 1024 tiles is already ~15 GB.
  constexpr int kMaxCacheTiles = 1024;
  if (bathy_cache_tiles < 1 || bathy_cache_tiles > kMaxCacheTiles) {
    std::cerr << "error: --bathy-cache-tiles must be in [1, " << kMaxCacheTiles
              << "]; a bathy tile is 960x960x2 doubles (~14.7 MB), so " << bathy_cache_tiles
              << " would ask for roughly "
              << (static_cast<std::int64_t>(bathy_cache_tiles) * 147 / 10) << " MB of "
              << "resident cache.\n";
    return 2;
  }
  const bool allow_mixed_projection = hasFlag(argc, argv, "--allow-mixed-projection");
  const bool dem_mode = !bathy_store.empty();
  // An argument refusal, not a store failure: the README assigns exit 2 to bad
  // arguments, and letting BathyDem's "no layer names requested" throw reach the
  // startup handler would report it as exit 1 (#297 review).
  if (dem_mode && splitCsv(bathy_layers).empty()) {
    std::cerr << "error: --bathy-layers names no layer (it is empty or all separators); "
              << "pass a comma-separated search order such as '" << kDefaultBathyLayers
              << "'\n";
    return 2;
  }
  const std::string projection_mode = dem_mode ? "dem" : "flat";
  // What the sidecar will record: this run's own values, adjusted by the guards
  // below for whatever the existing store already records (a deliberate mode mix
  // makes the store permanently `mixed`; a field this run says nothing about keeps
  // the previous run's value rather than being blanked).
  SidecarCarryForward carry;
  carry.mode = projection_mode;
  carry.bathy_store_json = jsonEscape(bathy_store);
  carry.bathy_layers_json = jsonEscape(dem_mode ? bathy_layers : "");
  if (!(min_dem_coverage >= 0.0 && min_dem_coverage <= 1.0)) {
    std::cerr << "error: --min-dem-coverage must be a fraction in [0, 1]\n";
    return 2;
  }
  if (!(datum_check_warn_m > 0.0)) {
    std::cerr << "error: --datum-check-warn-m must be a positive number of metres\n";
    return 2;
  }

  const OutputDirState out_dir_state = inspectOutputDir(out_dir);
  if (const auto guard_exit = checkOutputDirGuards(
      out_dir, out_dir_state, accumulate, dem_mode, projection_mode,
      allow_mixed_projection, bathy_store, bathy_layers, source_id_arg, &carry))
  {
    return *guard_exit;
  }

  std::ifstream in(tier1_path, std::ios::binary);
  if (!in) {
    std::cerr << "error: cannot open " << tier1_path << "\n";
    return 1;
  }
  std::uint32_t found_version = 0;
  switch (checkTier1Header(in, &found_version)) {
    case Tier1HeaderStatus::Ok:
      break;
    case Tier1HeaderStatus::BadVersion:
      std::cerr << "error: " << tier1_path << " is a Tier-1 stream but version "
                << found_version << " (this build expects v" << kTier1Version
                << "); re-run the importer to regenerate it\n";
      return 1;
    case Tier1HeaderStatus::BadMagic:
      std::cerr << "error: " << tier1_path << " is not a Tier-1 stream\n";
      return 1;
  }

  // Hard-fail on an unusable bathy store (absent/renamed layer dirs, zero tiles)
  // rather than degrade every sample to flat while exiting 0.
  std::unique_ptr<BathyDem> dem;
  if (dem_mode) {
    try {
      dem = std::make_unique<BathyDem>(
        bathy_store, splitCsv(bathy_layers), static_cast<std::size_t>(bathy_cache_tiles));
    } catch (const std::exception & e) {
      std::cerr << "error: " << e.what() << "\n";
      return 1;
    }
    for (const auto & warning : dem->warnings()) {
      std::cerr << "warning: " << warning << "\n";
    }
    std::cerr << "bathy DEM: " << dem->describe() << "\n";
  }

  const gggs::Level level(level_n);
  ProcessedAccumulator acc(level);

  Tier1Ping p;
  std::size_t n_in = 0, n_proj = 0, n_no_nadir = 0, n_placed = 0, n_bad_pose = 0;
  std::size_t n_dem_hit = 0, n_dem_no_coverage = 0, n_dem_degenerate = 0,
    n_dem_nonconverged = 0;
  // Datum cross-check: nadir_altitude_m (height above bottom, from the altimeter)
  // vs sensor ellipsoidal height − DEM height at the nadir point. Same physical
  // quantity by two independent paths, so a persistent offset means a datum
  // mismatch (orthometric store, lever-arm error, unexpected tide frame).
  std::size_t n_datum_check = 0;
  // The achieved DEM coverage, once the gate has computed it (flat runs: none).
  std::optional<double> dem_coverage;
  double datum_sum = 0.0, datum_sq_sum = 0.0;
  double bbox_min_lat = std::numeric_limits<double>::infinity();
  double bbox_max_lat = -std::numeric_limits<double>::infinity();
  double bbox_min_lon = std::numeric_limits<double>::infinity();
  double bbox_max_lon = -std::numeric_limits<double>::infinity();
  bool warned_wide_per_ping = false;   // throttle the degrees-slip warning to once.
  // A DEM lookup throws only when a tile the startup scan listed cannot be read —
  // a corrupt/truncated store. Abort the run (nothing has been written yet) rather
  // than silently finish with the rest of the samples placed flat. The handler is
  // installed around the DEM CALL SITES only: a catch-all around the whole ping
  // loop would also change the flat path (which this feature must leave untouched)
  // and would report a Tier-1 decode or accumulator failure as a DEM fault.
  const auto demLookupFailed = [](const std::exception & e) {
      std::cerr << "error: bathy DEM lookup failed: " << e.what() << "\n"
                << "  a tile the startup scan listed could not be read, so the store is\n"
                << "  corrupt or truncated. No tiles and no registry were written.\n";
    };
  // Running out of memory loading a tile is a SIZING fault, not a corrupt store —
  // reported separately so the operator adjusts --bathy-cache-tiles instead of
  // hunting a store that is fine (#297 review).
  const auto demAllocFailed = [bathy_cache_tiles]() {
      std::cerr << "error: out of memory loading a bathy tile.\n"
                << "  The resident budget is --bathy-cache-tiles " << bathy_cache_tiles
                << " x ~14.7 MB = ~"
                << (static_cast<std::int64_t>(bathy_cache_tiles) * 147 / 10) << " MB; lower it\n"
                << "  (the store itself is not implicated). No tiles and no registry were "
                << "written.\n";
    };
  while (readTier1Ping(in, p)) {
    ++n_in;
    if (p.sample_rate <= 0.0) {
      continue;
    }
    double altitude = 0.0;
    if (p.nadir_altitude_m > 0.0F) {
      altitude = p.nadir_altitude_m;
    } else if (no_nadir != "assume_zero") {
      ++n_no_nadir;
      continue;
    }

    const GeoBeam gb = ecefPoseToGeoBeam(p.tx, p.ty, p.tz, p.qx, p.qy, p.qz, p.qw);
    if (!gb.valid) {
      ++n_bad_pose;   // degenerate quaternion: don't project the ping due north.
      continue;
    }
    geographic_msgs::msg::GeoPoint origin;
    origin.latitude = gb.latitude_deg;
    origin.longitude = gb.longitude_deg;
    origin.altitude = 0.0;
    const double azimuth = gb.azimuth_rad;
    bbox_min_lat = std::min(bbox_min_lat, gb.latitude_deg);
    bbox_max_lat = std::max(bbox_max_lat, gb.latitude_deg);
    bbox_min_lon = std::min(bbox_min_lon, gb.longitude_deg);
    bbox_max_lon = std::max(bbox_max_lon, gb.longitude_deg);

    // Datum cross-check (see the counters above): only meaningful where the
    // altimeter actually returned, so it never runs under assume_zero.
    if (dem && p.nadir_altitude_m > 0.0F) {
      std::optional<double> nadir_bottom;
      try {
        nadir_bottom = dem->depthAt(gb.latitude_deg, gb.longitude_deg);
      } catch (const std::bad_alloc &) {
        demAllocFailed();
        return 1;
      } catch (const std::exception & e) {
        demLookupFailed(e);
        return 1;
      }
      if (nadir_bottom) {
        const double implied = gb.altitude_m - *nadir_bottom;
        const double discrepancy = static_cast<double>(p.nadir_altitude_m) - implied;
        ++n_datum_check;
        datum_sum += discrepancy;
        datum_sq_sum += discrepancy * discrepancy;
      }
    }

    // Along-track footprint splat (#208): per-sample beamwidth from Tier-1 v2, else
    // the CLI fallback (0 → point-deposit, unchanged). Run the stored width through
    // the shared validator too (non-finite/negative falls back; a degrees slip is
    // flagged once before the splat hard-caps it).
    bool per_ping_suspicious = false;
    const double per_ping_bw =
      sanitizeBeamwidthRad(static_cast<double>(p.tx_beamwidth_rad), &per_ping_suspicious);
    const double bw = per_ping_bw > 0.0 ? per_ping_bw : bw_fallback;
    if (per_ping_bw > 0.0 && per_ping_suspicious && !warned_wide_per_ping) {
      warned_wide_per_ping = true;
      std::cerr << "warning: stored per-ping tx_beamwidth " << per_ping_bw
                << " rad (~" << per_ping_bw * 180.0 / M_PI << " deg) is implausibly wide; "
                << "capping the splat (further such pings not warned)\n";
    }

    for (std::size_t j = 0; j < p.samples.size(); ++j) {
      const double slant =
        slantRange(static_cast<int>(j), p.sample0, p.sound_speed, p.sample_rate);
      const double flat_ground = groundRange(slant, altitude);
      if (flat_ground <= 0.0) {
        continue;   // nadir cone.
      }
      // DEM orthorectification (#297). The flat range stays the nadir-cone gate
      // and the iteration seed; on any degraded status the flat pair is used
      // unchanged, so the no-DEM path is bit-for-bit what it was.
      double ground = flat_ground;
      double vertical = altitude;
      if (dem) {
        DemCorrection corrected;
        try {
          corrected = correctedGroundRange(
            slant, gb.altitude_m, origin, azimuth, flat_ground,
            [&dem](double lat, double lon) {return dem->depthAt(lat, lon);});
        } catch (const std::bad_alloc &) {
          demAllocFailed();
          return 1;
        } catch (const std::exception & e) {
          demLookupFailed(e);
          return 1;
        }
        switch (corrected.status) {
          case DemCorrection::Status::kConverged:
            ++n_dem_hit;
            ground = corrected.ground_range;
            vertical = corrected.vertical_offset;
            break;
          case DemCorrection::Status::kNoCoverage:
            ++n_dem_no_coverage;
            break;
          case DemCorrection::Status::kDegenerate:
            ++n_dem_degenerate;
            break;
          case DemCorrection::Status::kNotConverged:
            ++n_dem_nonconverged;
            break;
        }
      }
      const auto intensity =
        static_cast<std::uint16_t>(std::clamp(static_cast<double>(p.samples[j]), 0.0, 65535.0));
      // grazingQuality derives the grazing angle from the (vertical, horizontal)
      // pair, so the DEM-corrected pair improves the best-source score with no
      // marine_backscatter API change. (Seabed-NORMAL incidence — ADR-0006 D4 —
      // stays out of scope; it belongs with the GeoCoder radiometry phase.)
      const std::uint16_t quality = grazingQuality(vertical, ground);
      const double footprint_m = footprintAlongTrack(slant, bw);
      splatAlongTrack(
        origin, gb.heading_rad, footprint_m, azimuth, ground, level,
        [&acc, intensity, quality, source_id](const gggs::CellIndex & cell) {
          acc.add(cell, intensity, quality, source_id);
        });
      ++n_placed;
    }
    ++n_proj;
  }

  // DEM coverage gate (#297). The startup hard-fail proves the store exists and
  // holds tiles; it cannot prove the store OVERLAPS this survey. A valid store for
  // another lake would yield n_dem_hit == 0 while every sample took the flat
  // fallback and the tool exited 0 with a normal-looking summary. Checked here,
  // BEFORE saveTiles/writeRegistry, so a failure writes nothing.
  if (dem) {
    // Datum cross-check, reported FIRST — before the coverage gate's own exits and
    // before saveTiles/writeRegistry — because it is the stronger signal of the two:
    // low coverage means samples were left uncorrected, while a datum offset means
    // they were CONFIDENTLY MIS-PLACED. Reporting it after the gate's `return 3`
    // would silence it on exactly the runs a wrong vertical datum produces — a
    // store in the wrong vertical frame drives coverage down (its heights push the
    // geometry degenerate or non-convergent) and would exit 3 with no mention of
    // the datum at all (#297 round-2 review). It still only warns (the offset can
    // be a legitimate known bias, and the tool cannot tell).
    if (n_datum_check > 0) {
      const double n = static_cast<double>(n_datum_check);
      const double mean = datum_sum / n;
      const double rms = std::sqrt(datum_sq_sum / n);
      std::cerr << "tier2-processed: datum cross-check (nadir altimeter vs sensor height "
                << "- DEM) over " << n_datum_check << " pings: mean " << mean
                << " m, rms " << rms << " m\n";
      if (std::abs(mean) > datum_check_warn_m) {
        std::cerr << "warning: mean datum discrepancy " << mean << " m exceeds "
                  << datum_check_warn_m << " m. The altimeter's height above bottom and the "
                  << "sensor-height-minus-DEM height are the same quantity by two independent "
                  << "paths, so a persistent offset means a datum mismatch (an orthometric "
                  << "bathy store, a lever-arm error, or an unexpected tide frame) — the "
                  << "samples may be confidently mis-placed, not merely uncorrected.\n";
      }
    } else {
      std::cerr << "warning: datum cross-check had no usable ping (no altimeter return with "
                << "DEM coverage), so the vertical datum agreement is unverified\n";
    }

    // Every non-converged status places the sample FLAT, so all of them belong in
    // the denominator — a run that is 40 % degenerate is 40 % flat-placed, and
    // scoring it on hits-vs-no-coverage alone would report coverage 1.0 and sail
    // through the very gate meant to stop it (#297 review).
    const std::size_t denominator =
      n_dem_hit + n_dem_no_coverage + n_dem_degenerate + n_dem_nonconverged;
    if (denominator == 0) {
      // No sample ever reached the lookup: an empty archive, or every ping dropped
      // upstream. That is a no-usable-input failure, not a coverage verdict —
      // reporting it as "coverage 0 (0 of 0)" would send the operator hunting for
      // the wrong problem.
      std::cerr << "error: no sample reached the DEM lookup, so this run placed nothing.\n"
                << "  read " << n_in << " ping(s), projected " << n_proj
                << "; dropped no-nadir=" << n_no_nadir << " bad-pose=" << n_bad_pose << "\n"
                << "  Check the Tier-1 archive rather than the bathy store: an empty .sst1,\n"
                << "  or pings without an altimeter return under --no-nadir-policy drop,\n"
                << "  produce this. Nothing was written.\n";
      return 3;
    }
    const double coverage =
      static_cast<double>(n_dem_hit) / static_cast<double>(denominator);
    dem_coverage = coverage;   // recorded in the sidecar; see writeProjectionSidecar.
    const bool below_gate = coverage < min_dem_coverage;
    if (below_gate || coverage < 0.5) {
      std::string gate_note;
      if (below_gate) {
        gate_note = " is below --min-dem-coverage " + std::to_string(min_dem_coverage);
      }
      std::cerr << (below_gate ? "error" : "warning") << ": DEM coverage " << coverage
                << " (" << n_dem_hit << " of " << denominator << " samples that reached the "
                << "lookup; the other " << (denominator - n_dem_hit)
                << " were placed flat)" << gate_note << "\n"
                << "  bathy store: " << dem->describe() << "\n"
                << "  layer search order: " << bathy_layers << "\n"
                << "  counters: hit=" << n_dem_hit << " no-coverage=" << n_dem_no_coverage
                << " degenerate=" << n_dem_degenerate
                << " non-converged=" << n_dem_nonconverged << "\n"
                << "  survey bounds: lat [" << bbox_min_lat << ", " << bbox_max_lat
                << "] lon [" << bbox_min_lon << ", " << bbox_max_lon << "]\n";
      if (below_gate) {
        std::cerr << "  refusing to write: the output would be a flat-bottom store wearing a\n"
                  << "  DEM run's name. Check --bathy-store / --bathy-layers, or pass\n"
                  << "  --min-dem-coverage 0 to accept a deliberately partial run.\n";
        return 3;
      }
    }
  }

  // A run that placed NOTHING must not touch the store's provenance. The DEM
  // branch's "no sample reached the lookup" refusal above covers only `if (dem)`,
  // so a FLAT --accumulate --allow-mixed-projection over an empty (or
  // altimeter-less) .sst1 contributed no sample yet still rewrote the sidecar —
  // permanently relabelling a `dem` store `mixed` and appending a `null` history
  // element, at exit 0. Same failure class as the run-that-placed-nothing-still-
  // cleared path that got `--overwrite` removed, so gate on the tiles themselves:
  // no cell painted, nothing recorded (#297 round-4 review).
  if (acc.tiles().empty()) {
    std::cerr << "error: this run painted no cell, so it has nothing to record.\n"
              << "  read " << n_in << " ping(s), projected " << n_proj
              << "; dropped no-nadir=" << n_no_nadir << " bad-pose=" << n_bad_pose << "\n"
              << "  Check the Tier-1 archive: an empty .sst1, or pings without an altimeter\n"
              << "  return under --no-nadir-policy drop, produce this. Nothing was written and\n"
              << "  no existing store was touched — in particular its projection.json keeps\n"
              << "  the mode and coverage history it already had.\n";
    return 3;
  }

  // Accumulate into an existing store: reload each tile this run touched and fold
  // it back in (best-source) before saving, so bag-by-bag ingestion composites
  // instead of overwriting (issue #253) — bounded RAM/disk vs one whole-campaign
  // pass. Without --accumulate a populated out_dir is refused outright (the guard
  // above), so saveTiles only ever writes into an empty or fresh directory here.
  if (accumulate) {
    std::vector<gggs::GridIndex> grids;
    grids.reserve(acc.tiles().size());
    for (const auto & kv : acc.tiles()) {
      grids.push_back(kv.first);
    }
    std::size_t folded = 0;
    for (const auto & grid : grids) {
      const std::string path =
        (std::filesystem::path(out_dir) /
        marine_tiled_raster_store::tileFilename(grid)).string();
      if (!std::filesystem::exists(path)) {
        continue;   // new coverage — nothing on disk to merge.
      }
      try {
        const auto existing = marine_tiled_raster_store::loadTile<std::uint16_t>(
          path, level, ProcessedAccumulator::kBands);
        acc.foldTile(existing);
        ++folded;
      } catch (const std::exception & e) {
        // Never destroy prior coverage on a read hiccup. If we cannot reload an
        // existing tile, saving would OVERWRITE it with this run's partial
        // composite and silently drop its accumulated coverage — so abort before
        // any tile is written. (Nothing has been saved yet at this point.)
        std::cerr << "error: --accumulate could not reload existing tile " << path
                  << ": " << e.what() << "\n"
                  << "  refusing to continue: saving now would OVERWRITE this tile and lose its\n"
                  << "  prior coverage. Inspect and repair the tile, or regenerate the store\n"
                  << "  into a fresh out_dir. No tiles were written.\n";
        return 1;
      }
    }
    std::cerr << "accumulate: folded " << folded << " existing tile(s) from " << out_dir << "\n";
  }

  // The projection sidecar is written FIRST — before any tile — so there is no
  // window in which tiles exist without a mode record. Written last, a crash or a
  // full filesystem between the tile write and the sidecar write would leave a
  // DEM-orthorectified store with no projection.json, which every later run reads
  // as a pre-#297 FLAT build and happily accumulates flat samples into (#297
  // round-2 review). The failure mode is now the harmless one instead: a sidecar
  // with no tiles beside it, which the guards treat as an interrupted build.
  // `saveTiles` creates out_dir lazily (and not at all when nothing is dirty), so
  // create it here.
  std::error_code mkdir_ec;
  std::filesystem::create_directories(out_dir, mkdir_ec);
  if (mkdir_ec && !std::filesystem::is_directory(out_dir)) {
    std::cerr << "error: cannot create output directory " << out_dir << ": "
              << mkdir_ec.message() << "\n";
    return 1;
  }
  // This run's element appended to the history the prior sidecar recorded, so the
  // list holds one figure per run that ever wrote this store.
  std::vector<CoverageElement> coverage_history = carry.dem_coverage_history;
  coverage_history.push_back(dem_coverage);
  if (!writeProjectionSidecar(
      out_dir, jsonEscape(carry.mode), carry.bathy_store_json, carry.bathy_layers_json,
      coverage_history))
  {
    std::cerr << "error: could not write " << out_dir << "/projection.json.\n"
              << "  It records how this run placed its samples, and it is written before the\n"
              << "  tiles precisely so no tile can exist without it. This run wrote no tile\n"
              << "  and no registry, and the write is staged-and-swapped, so any sidecar the\n"
              << "  store already had is intact: fix the output directory's permissions/space\n"
              << "  and re-run.\n";
    return 1;
  }

  std::size_t written = 0;
  try {
    const std::vector<std::optional<std::uint16_t>> nodata(
      ProcessedAccumulator::kBands, std::optional<std::uint16_t>(0));
    written = marine_tiled_raster_store::saveTiles<std::uint16_t>(acc.tiles(), out_dir, nodata);
  } catch (const std::exception & e) {
    std::cerr << "saveTiles failed: " << e.what() << "\n";
    return 1;
  }
  const std::string registry_path =
    (std::filesystem::path(out_dir) / "registry.json").string();
  writeRegistry(registry_path, source_id, platform, sensor, sensor_class, campaign);
  // `writeRegistry` returns void, and BOTH --accumulate provenance guards key on
  // registry.json being present — an interrupted or failed write would leave tiles
  // that every later accumulate folds into unguarded. Verify the file landed.
  std::error_code registry_ec;
  if (!std::filesystem::exists(registry_path) ||
    std::filesystem::file_size(registry_path, registry_ec) == 0 || registry_ec)
  {
    std::cerr << "error: " << registry_path << " was not written (or is empty) after the "
              << "tiles were saved.\n"
              << "  The tiles carry per-cell source indices that only the registry resolves,\n"
              << "  and both --accumulate provenance guards look for this file — without it a\n"
              << "  later run would fold into the store unchecked. Regenerate the store.\n";
    return 1;
  }

  std::cerr << "tier2-processed: projected " << n_proj << "/" << n_in << " pings ("
            << n_placed << " samples; dropped no-nadir=" << n_no_nadir
            << " bad-pose=" << n_bad_pose << "), best-source into "
            << written << " 3-band tiles + registry.json in " << out_dir
            << " [projection=" << projection_mode
            << (carry.mode == projection_mode ? "" : " store=" + carry.mode) << "]\n";
  if (dem) {
    std::cerr << "tier2-processed: dem hit=" << n_dem_hit
              << " no-coverage=" << n_dem_no_coverage
              << " degenerate=" << n_dem_degenerate
              << " non-converged=" << n_dem_nonconverged
      // Deliberately NOT called "hits": these count DEM PROBES that returned a
      // value (up to one bilinear stencil per iteration per sample, plus one per
      // ping for the datum check), so they are not comparable with hit= above.
              << "; per-layer store lookups that returned data (probes, not samples)";
    for (const auto & kv : dem->lookupsByLayer()) {
      std::cerr << " " << kv.first << "=" << kv.second;
    }
    std::cerr << "\n";
    if (dem_coverage) {
      std::cerr << "tier2-processed: achieved DEM coverage " << *dem_coverage
                << ", recorded in projection.json\n";
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return runTool(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "error: unhandled exception: " << e.what() << "\n"
              << "  The run aborted. If the store had already begun writing, treat it as\n"
              << "  incomplete and regenerate it into a fresh output directory.\n";
    return 1;
  } catch (...) {
    std::cerr << "error: unhandled non-standard exception; the run aborted. Treat any\n"
              << "  partially written store as incomplete and regenerate it.\n";
    return 1;
  }
}
