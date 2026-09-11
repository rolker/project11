---
issue: 373
---

# Issue #373 — docs: write down the sonar processing chain — one owner per stage, and what each stage must be fed

## Integrated Review
**Status**: complete
**When**: 2026-09-11 11:07 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #374 at `9993fc7`
**Sources**: 3 (rolker review @ `9993fc7` — 16 inline comments; Copilot R1–R5 — 3 inline comments across `c753a0c`/`e69d6cb`/`9993fc7`; integrator read of current merge state)
**Cross-source confirmations**: 0 (no `progress.md` timeline existed for this issue — this entry creates it; the GitHub reviews are the only prior source)
**CI**: all-pass (`build`, `copilot-pull-request-reviewer`)

### Findings

Scope and structure — the largest block:
- [ ] (must-fix, rolker L1) The document is multibeam-only but is named and framed as the general sonar chain. Either rename it to say multibeam, or add placeholders for single-beam and split-beam echosounders, sidescan, water column (multibeam, single/split beam, and the sidescan water-column portion), and raw element data needing beamforming — the last plausibly folding into acquisition — `docs/sonar_processing_chain.md`
- [ ] (must-fix, rolker L93) The three arrival scenarios do not cover everything that will arrive: systems that have already applied a sound-speed profile and computed uncertainties, soundings with missing metadata, and element-level data needing beamforming — `docs/sonar_processing_chain.md`
- [ ] (must-fix, rolker L113) There must be a pathway for data missing the required fields. Worked example: data was collected with no reported beamwidths and must still be processable via an override at processing time. Decide between a transformation node that fills defaults and a parameter-based fill-in, and apply one uniform approach — `docs/sonar_processing_chain.md`

Conventions and naming — overlaps already-filed cube issues:
- [ ] (should-fix, rolker L61 + L63) Revisit positive-down for `Platform.heave` and the ray tracer's depths. Operator's stated rule: under-the-hood follows ROS conventions, user-facing values such as parameters follow what the literature and spec sheets use, e.g. degrees. Was positive-down inherited from original CUBE? Feeds [cube#152](https://github.com/rolker/cube_bathymetry/issues/152) — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L64) Consider renaming the `_error` fields to variance. Adjacent to [cube#142](https://github.com/rolker/cube_bathymetry/issues/142), where `depth_var` holds a standard deviation — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L56) Beamwidth meaning is ambiguous in the message definition's own comments; file an issue against the message format to state what the beamwidth means — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L500) Record the accepted justifications for diverging from Calder: device independence with no built-in device tables, conformance to ROS standards as a reason to choose different units, and evolving technology such as RTK GPS replacing tide, draft and heave — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L257) Rename `kongsberg_em_bridge` to reflect that it consumes `.all` data, and drop "bridge" from the name. Still useful beyond the M3 for older EM-series `.all` files; the EM line now uses `.kmall` — `marine_tools`

Technical expansion — most become new issues rather than doc edits:
- [ ] (should-fix, rolker L343) Any deviation from Calder must be documented and justified, RTK ellipsoidal referencing removing tide/draft/heave among them. Expand the error model for vessel speed: check that a stationary pinging vessel produces uniform, artifact-free data below the boat. Determine whether yaw rate needs its own term or whether IMU uncertainties already cover turn noise — if turn uncertainty is estimated well, turn data should not need cutting — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L183) Flesh out angular-response curves. The lake test succeeded where water properties varied little; determine whether a curve can be parameterized so a sound-speed and possibly temperature profile lets one calibration transfer between water bodies — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L286) `SspRayTracer` should become a shared module serving real sound-speed profiles as well as estimated ones. Profiles exist for the Appledore survey. A sound-speed branch of the pipeline is likely needed; look at sound speed manager for inspiration — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L381) Decide whether georeferencing needs two variants, real-time and post-processing, differing only in TF buffer handling, and whether this is a plain TF lookup or warrants a shared library. Feeds [cube#146](https://github.com/rolker/cube_bathymetry/issues/146) — `docs/sonar_processing_chain.md`
- [ ] (should-fix, rolker L409) Determine whether `cube::GeoMapSheet` must also carry backscatter, and revisit whether both the Cartesian and geographic spaces are still needed — Cartesian or map space is probably acceptable for temporary or cube-lab work. Adjacent to [cube#130](https://github.com/rolker/cube_bathymetry/issues/130) — `docs/sonar_processing_chain.md`
- [ ] (nice-to-have, rolker L391) The survey explorer's cube lab should document what every `cube::Parameters` field means, as a pop-out with diagrams — `marine_perception_tools`
- [ ] (nice-to-have, rolker L430) Think through a live sidescan store and sidescan coverage transmission to CAMP. May not be implemented, but should guide generalizing the current live multibeam coverage; the same live coverage recording could serve onboard coverage planners — `docs/sonar_processing_chain.md`

Staleness found by the integrator, not by any reviewer:
- [ ] (must-fix, integrator) "None are fixed as of 2026-09-10" is now false. `cube_bathymetry` PR#153 merged 2026-09-10 11:19 -0700, after this branch's last commit, closing [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) and [cube#147](https://github.com/rolker/cube_bathymetry/issues/147) and fixing five of the ten defect-table rows — every stage-3 entry — `docs/sonar_processing_chain.md:434`
- [ ] (must-fix, integrator) The same merge dates the "Why the M3 bridge leaves beamwidths empty" section. Its "do not fix the driver first" sequencing was conditioned on the consumer unit fix, which has now landed — `docs/sonar_processing_chain.md:246`

Bot findings, all trivial and all still present at head:
- [ ] (nice-to-have, Copilot @ `c753a0c`) The glance table names `cube::ErrorModel` as stage 3's owner while the paragraph below calls stages 2 and 3 one component; one clarifying clause removes the stumble — `docs/sonar_processing_chain.md:40`
- [ ] (nice-to-have, Copilot @ `e69d6cb`) `cube#81` is the only unlinked issue reference in the file; verified by scanning every reference — `docs/sonar_processing_chain.md:177`
- [ ] (nice-to-have, Copilot @ `9993fc7`) "not yet filed" does not say what is unfiled; name the missing tracking issue explicitly — `docs/sonar_processing_chain.md:191`

### False positives
- None. All three bot comments describe conditions verified present in the file at head, and all sixteen human comments are design direction rather than claims to check.

## Implementation
**Status**: complete
**When**: 2026-09-11 (afternoon, -04:00)
**By**: Claude Code Agent (Claude Fable 5.1)

**Branch**: `feature/issue-373` (PR #374)

All sixteen operator comments from the Integrated Review were discussed to a
decision one at a time before editing; the page was then rewritten in one pass
(906 lines, from 517). What changed, mapped to the review findings:

- [x] L1 scope → shared spine + per-product middles; sidescan / water column / single-beam as contract-only branches
- [x] L93 + L113 → two-axis arrival framing (entry point × completeness); five-tier missing-fields rule; recording rule; rented stages (beamforming → M3, bottom detection → DeltaT; Imagenex obfuscation + patent recorded); IHO f(z) recommendation withdrawn (class empty)
- [x] L61/L63/L64/L500 → three-category convention rule, divergence justifications, defensibility rule; wire + struct variance rename filed as cube#158; retrofit as unh_echoboats_project11#489
- [x] L56 → "full −3 dB" sourced to sonar_image_proc + the v1 migration rule; rviz half-angle bug filed as rviz_sonar_image#9; message path tracked in #380 (operator opens the upstream doc issue)
- [x] L257 → `kongsberg_dotall` rename, marine_tools#84, sequenced after #82
- [x] L343 → cube#156 (RTK half-job), cube#155 (attitude-rate coupling, data check first); stationary case recorded as an independence question; offline zero-speed budget noted beside cube#145
- [x] L183 + L286 → angular-response decomposition (GeoCoder) as direction; water-body model section (`water/` theme); ray tracer moved under it; scope note posted on #300
- [x] L381 → georeferencing = one function + policy switch; the three differences posted on cube#146
- [x] L409 → Cartesian `Grid`/`MapSheet` to be deleted, GeoGrid the one grid, equal-angle cell property recorded; scope note posted on cube#129
- [x] L391 → cube#157 (parameter reference) + marine_perception_tools#54 (pop-out)
- [x] L430 → live transport already source-agnostic; sidescan publisher gap filed as #379
- [x] integrator: defect table refreshed against cube PR#153; M3-bridge section updated; explorer state updated to merged
- [x] Copilot ×3: owner clause reworded, cube#81 linked, "not yet filed" replaced by #378
- [x] DeltaT correction (found during review): imagenex_deltat#2 filed; page rewritten to describe what the driver actually publishes
- [x] plan.md revision section added

**Not done here**: nothing on the page was left describing unmerged work; the
stores rebuild after the pipeline changes is the operator's decision, recorded.

### Follow-ups filed by this review
cube#155, #156, #157, #158; marine_tools#84; imagenex_deltat#2;
rviz_sonar_image#9; marine_perception_tools#54; unh_echoboats_project11#489;
#378, #379, #380. Scope notes: cube#129, cube#146, #300.
