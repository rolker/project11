# Plan: write down the sonar processing chain

## Issue

[#373](https://github.com/rolker/unh_marine_autonomy/issues/373) — one page
that says what the sonar processing chain is, which component owns each stage,
and what each stage must be fed.

Sequenced deliberately ahead of the units fix
([cube_bathymetry#144](https://github.com/rolker/cube_bathymetry/issues/144))
and ahead of moving the survey explorer onto the shared projector, so both have
something to conform to.

## Context

Two sonar documents already exist in this repo and neither is the chain:

- `docs/sonar_ecosystem.md` — the big-picture *status* map, organised by arcs
  and by where to direct effort. Currently being rewritten by the open
  [#368](https://github.com/rolker/unh_marine_autonomy/pull/368), so this plan
  must not touch it.
- `docs/sonar_reference.md` — durable hardware, protocol and data-location
  facts that do not change per PR.

The new page is the third kind: the processing chain itself. It is a reference
document, not an ADR — it records what the chain IS and what it needs, and
cites the decisions that already govern each stage rather than restating them.

## Approach

### 1. Verify every stage against source before writing a word of it

The documentation-accuracy rule governs this whole plan: no parameter, topic,
message field, type or unit goes in unread. The reading list, by stage:

| Stage | Read |
|---|---|
| Acquisition | `marine_tools/kongsberg_em_bridge`, `SonarDetections.msg`, `PingInfo.msg`, `uma-ADR-0009` |
| Projection | `cube_bathymetry/src/detections_projector.cpp`, `marine_perception_tools/src/mbes_geometry.hpp` |
| Uncertainty | `cube_bathymetry/{include,src}/…/error_model.{h,cpp}`, `marine_perception_tools/src/sounding_uncertainty.hpp` |
| Georeferencing | `detections_projector.cpp` TF lookups, `marine_perception_tools/src/tf_lift.hpp` |
| Estimation | `cube_bathymetry/src/{node,grid,map_sheet}.cpp`, `marine_perception_tools/src/cube_lab.cpp` |
| Store write | `cube_bathymetry/src/store_import.cpp`, `marine_bathymetry_store`, `marine_perception_tools/src/cube_export.cpp` |

Also read `ssp_ray_tracer` and `sonar_info_curve`, which are chain members the
first survey only skimmed.

### 2. Write `docs/sonar_processing_chain.md`

One section per stage. Each section states, in this order: what the stage does;
the one component that owns it, by package and file; what it must be fed, with
units and with the behaviour when an input is absent; and where the stage is
currently implemented more than once, with a judgement on whether that
duplication is defensible.

A units table is a first-class part of the uncertainty stage: beamwidths in
radians everywhere on the wire, `Device` fields in degrees, angles in radians,
`cube::Sounding` errors as variances not standard deviations. The chain's known
defects are recorded as defects with their issue links, not silently corrected
in prose.

### 3. Cross-link

Add a link from `docs/sonar_reference.md`. Leave `docs/sonar_ecosystem.md`
alone and record the link as a follow-up, because #368 is rewriting it.

## Files to Change

| File | Change |
|---|---|
| `docs/sonar_processing_chain.md` | new |
| `docs/sonar_reference.md` | one link line |
| `.agent/work-plans/issue-373/plan.md` | this plan |

## Principles Self-Check

- **Verify, never assume** — the whole of step 1 exists for this. The survey
  that motivated the issue already found one claim in a filed issue that was
  wrong because nobody had read the message definition.
- **Do the whole thing** — every stage gets the same treatment, including the
  two the first survey only skimmed. A chain document with a gap in it invites
  the next re-implementation to land in the gap.
- **Do not document from assumptions about ownership either** — "the one
  component that owns this stage" is a claim about the code, so it is checked
  against the code, and where no single component owns a stage the document
  says so rather than nominating one.

## ADR Compliance

Cites and does not restate: `uma-ADR-0002` (bathymetry store), `uma-ADR-0007`
(MBES backscatter store), `uma-ADR-0009` (sonar info message), `uma-ADR-0010`
(geospatial world model), `uma-ADR-0011` (overview pyramid). No ADR is amended
by this work.

## Consequences

- No code changes, so no tests and no rebuild.
- Docs-only PR, so the PR body omits the test-plan section.
- Follow-ups this document is expected to sharpen, not resolve:
  [cube_bathymetry#144](https://github.com/rolker/cube_bathymetry/issues/144)
  (units, and its scope needs correcting),
  [cube_bathymetry#145](https://github.com/rolker/cube_bathymetry/issues/145)
  (no offline vessel/device configuration), and
  [marine_tools#82](https://github.com/rolker/marine_tools/issues/82) (drivers
  publish no beamwidths).

## Documentation & Instruction Impact

The chain document is the deliverable, so the impact is the cross-links above.
No instruction file changes. If the verification pass turns up a pattern worth
capturing for other agents, it is raised as a candidate rather than written
into `.agent/knowledge/` as a side effect.

## Open Questions

- ~~Whether the explorer's stand-in uncertainty is recorded as a defensible
  duplication or as debt with an issue behind it.~~ **Resolved by the
  verification pass**: on the default branch the explorer does not run a
  stand-in of the error model at all, but a depth-only placeholder with no
  angle term. The angle-aware stand-in exists only on the branch in review
  (marine_perception_tools#50). Recorded as what ships, with the branch state
  marked, rather than as a duplication.

## What the verification pass changed

The first draft attributed stage 4 to `DetectionsProjector` and described the
explorer's unmerged work in the present tense. An accuracy review against
source corrected both, and turned up three defects the draft had not found:

- **Stage 4 has no owner.** The projector returns sonar-frame soundings; the
  world lift is hand-rolled four times inside `cube_bathymetry` and once in the
  explorer. Filed as
  [cube#146](https://github.com/rolker/cube_bathymetry/issues/146) and now the
  document's headline structural finding.
- **A zero-filled beamwidth array is believed.** `norbit_driver` resizes the
  arrays without assigning them, so the per-beam branch is selected on length
  and the angular term vanishes. Added to
  [cube#144](https://github.com/rolker/cube_bathymetry/issues/144): the fix
  must validate, not only convert.
- **`horizontal_error` IS a clean variance.** The draft repeated the header's
  claim that one term is doubled to approximate a 95% bound. The code does no
  such thing and neither does Calder's; the comment is inherited text that was
  never true of this port. Also added to #144.

## Estimated Scope

One new document, one link line. Verification is the bulk of the work.
