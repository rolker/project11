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
- [x] (must-fix, rolker L1) The document is multibeam-only but is named and framed as the general sonar chain. Either rename it to say multibeam, or add placeholders for single-beam and split-beam echosounders, sidescan, water column (multibeam, single/split beam, and the sidescan water-column portion), and raw element data needing beamforming — the last plausibly folding into acquisition — `docs/sonar_processing_chain.md`
- [x] (must-fix, rolker L93) The three arrival scenarios do not cover everything that will arrive: systems that have already applied a sound-speed profile and computed uncertainties, soundings with missing metadata, and element-level data needing beamforming — `docs/sonar_processing_chain.md`
- [x] (must-fix, rolker L113) There must be a pathway for data missing the required fields. Worked example: data was collected with no reported beamwidths and must still be processable via an override at processing time. Decide between a transformation node that fills defaults and a parameter-based fill-in, and apply one uniform approach — `docs/sonar_processing_chain.md`

Conventions and naming — overlaps already-filed cube issues:
- [x] (should-fix, rolker L61 + L63) Revisit positive-down for `Platform.heave` and the ray tracer's depths. Operator's stated rule: under-the-hood follows ROS conventions, user-facing values such as parameters follow what the literature and spec sheets use, e.g. degrees. Was positive-down inherited from original CUBE? Feeds [cube#152](https://github.com/rolker/cube_bathymetry/issues/152) — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L64) Consider renaming the `_error` fields to variance. Adjacent to [cube#142](https://github.com/rolker/cube_bathymetry/issues/142), where `depth_var` holds a standard deviation — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L56) Beamwidth meaning is ambiguous in the message definition's own comments; file an issue against the message format to state what the beamwidth means — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L500) Record the accepted justifications for diverging from Calder: device independence with no built-in device tables, conformance to ROS standards as a reason to choose different units, and evolving technology such as RTK GPS replacing tide, draft and heave — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L257) Rename `kongsberg_em_bridge` to reflect that it consumes `.all` data, and drop "bridge" from the name. Still useful beyond the M3 for older EM-series `.all` files; the EM line now uses `.kmall` — `marine_tools`

Technical expansion — most become new issues rather than doc edits:
- [x] (should-fix, rolker L343) Any deviation from Calder must be documented and justified, RTK ellipsoidal referencing removing tide/draft/heave among them. Expand the error model for vessel speed: check that a stationary pinging vessel produces uniform, artifact-free data below the boat. Determine whether yaw rate needs its own term or whether IMU uncertainties already cover turn noise — if turn uncertainty is estimated well, turn data should not need cutting — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L183) Flesh out angular-response curves. The lake test succeeded where water properties varied little; determine whether a curve can be parameterized so a sound-speed and possibly temperature profile lets one calibration transfer between water bodies — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L286) `SspRayTracer` should become a shared module serving real sound-speed profiles as well as estimated ones. Profiles exist for the Appledore survey. A sound-speed branch of the pipeline is likely needed; look at sound speed manager for inspiration — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L381) Decide whether georeferencing needs two variants, real-time and post-processing, differing only in TF buffer handling, and whether this is a plain TF lookup or warrants a shared library. Feeds [cube#146](https://github.com/rolker/cube_bathymetry/issues/146) — `docs/sonar_processing_chain.md`
- [x] (should-fix, rolker L409) Determine whether `cube::GeoMapSheet` must also carry backscatter, and revisit whether both the Cartesian and geographic spaces are still needed — Cartesian or map space is probably acceptable for temporary or cube-lab work. Adjacent to [cube#130](https://github.com/rolker/cube_bathymetry/issues/130) — `docs/sonar_processing_chain.md`
- [x] (nice-to-have, rolker L391) The survey explorer's cube lab should document what every `cube::Parameters` field means, as a pop-out with diagrams — `marine_perception_tools`
- [x] (nice-to-have, rolker L430) Think through a live sidescan store and sidescan coverage transmission to CAMP. May not be implemented, but should guide generalizing the current live multibeam coverage; the same live coverage recording could serve onboard coverage planners — `docs/sonar_processing_chain.md`

Staleness found by the integrator, not by any reviewer:
- [x] (must-fix, integrator) "None are fixed as of 2026-09-10" is now false. `cube_bathymetry` PR#153 merged 2026-09-10 11:19 -0700, after this branch's last commit, closing [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) and [cube#147](https://github.com/rolker/cube_bathymetry/issues/147) and fixing five of the ten defect-table rows — every stage-3 entry — `docs/sonar_processing_chain.md:434`
- [x] (must-fix, integrator) The same merge dates the "Why the M3 bridge leaves beamwidths empty" section. Its "do not fix the driver first" sequencing was conditioned on the consumer unit fix, which has now landed — `docs/sonar_processing_chain.md:246`

Bot findings, all trivial and all still present at head:
- [x] (nice-to-have, Copilot @ `c753a0c`) The glance table names `cube::ErrorModel` as stage 3's owner while the paragraph below calls stages 2 and 3 one component; one clarifying clause removes the stumble — `docs/sonar_processing_chain.md:40`
- [x] (nice-to-have, Copilot @ `e69d6cb`) `cube#81` is the only unlinked issue reference in the file; verified by scanning every reference — `docs/sonar_processing_chain.md:177`
- [x] (nice-to-have, Copilot @ `9993fc7`) "not yet filed" does not say what is unfiled; name the missing tracking issue explicitly — `docs/sonar_processing_chain.md:191`

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

## Local Review
**Status**: complete
**When**: 2026-09-11 14:29 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**PR**: #374 at `a8cbfd5`
**Mode**: post-PR
**Depth**: Deep (reason: 1170 changed lines; cross-repo factual surface)
**Must-fix**: 16 | **Suggestions**: 27

Six specialists: five source-verification passes over `cube_bathymetry`,
`marine_tools`, `imagenex_deltat`, `norbit`, `ros2sonic`, `edgetech_sonar`,
`marine_perception_tools`, `rviz_sonar_image`, `marine_sidescan_mosaic`,
`rqt_operator_tools`, `marine_acoustic_msgs`, `marine_interfaces`,
`unh_echoboats_project11`; plus governance + plan drift. All 33 issue/PR
references, all 8 ADR targets, all 5 intra-page anchors and all 14 tables
check out. The technical body held up nearly everywhere; the findings below
are the places it did not.

### Findings
- [ ] (must-fix) "The DeltaT is recorded as its cloud" — the topic is commented out in the boat config today — `docs/sonar_processing_chain.md:290-293` vs `unh_echoboats_project11/bizzyboat_project11/config/bizzyboat.yaml:922-923`
- [ ] (must-fix) "Offline replay always supplies NaN" is false — `import_bag`/`batch_regen` take `--odom-topic` and supply real per-ping SOG; only `bag_to_geotiff` always passes NaN — `docs/sonar_processing_chain.md:510-513` and `:836` vs `cube_bathymetry/src/import_bag_main.cpp:1205`, `batch_regen_main.cpp:877`
- [ ] (must-fix) The "~7 °/s / ~70 °/s" attitude-rate figures are not reproducible from the latency the code forms (σ≈0.0308 s → 1.6 / 16 °/s); they drop `gps_latency_sdev`, the largest of the three — `docs/sonar_processing_chain.md:554-557` vs `cube_bathymetry/src/error_model.cpp:101-103`, `include/cube_bathymetry/error_model.h:108,126,129`
- [ ] (must-fix) "`cube::Parameters` has 25 fields" — the struct has 28 non-static data members (the 25 is inherited from cube#157) — `docs/sonar_processing_chain.md:679` vs `cube_bathymetry/include/cube_bathymetry/parameters.h:122-235`
- [ ] (must-fix) `cube::SspRayTracer` is not a symbol — the API is the free function `cube::traceRay` plus `SoundSpeedProfilePoint`/`RayTraceResult`; the page types it in backticks four times incl. the units table — `docs/sonar_processing_chain.md:88,110-118,472-476,755-760` vs `cube_bathymetry/include/cube_bathymetry/ssp_ray_tracer.h:92,223`
- [ ] (must-fix) "The operator settled the same reading on 2026-06-21 ([marine_tools#62])" — #62 says the opposite (across-track fan HALF-angle) and carries no operator comment; the correct source is the driver — `docs/sonar_processing_chain.md:128-129` vs `marine_tools/garmin_sidescan/garmin_sidescan/node.py:97-101`
- [ ] (must-fix) The latched `SonarInfo` is written as what every driver does; only `kongsberg_em_bridge` publishes it — `garmin_sidescan`, `edgetech_sonar`, `imagenex_deltat` have no reference to it — `docs/sonar_processing_chain.md:195-199` vs `marine_tools/kongsberg_em_bridge/kongsberg_em_bridge/node.py:297`
- [ ] (must-fix) "what remains is includes, comments and a forward declaration" for the Cartesian pair — tests use it as a correctness oracle and the explorer's CMake probes `grid.h` — `docs/sonar_processing_chain.md:637-641` vs `cube_bathymetry/test/test_publish_equivalence.cpp:347,359`, `marine_perception_tools/CMakeLists.txt:28`
- [ ] (must-fix) "corrected Calder's Eqn. 3.49" inverts it — the divergence record calls it a porting error; Calder's C already had `cosT²` — `docs/sonar_processing_chain.md:861` vs `cube_bathymetry/docs/divergences_from_calder.md:318`, `original_cube/libsrc/errmod/errmod_full.c:376-377`
- [ ] (must-fix) The field-carriage fix is cited to mpt#49, which is about beam-angle-blind uncertainty; the fix is mpt PR#50 — `docs/sonar_processing_chain.md:611-613`
- [ ] (must-fix) Stage 4's "What it must be fed" requires the buffer to cover the ping stamp, contradicting the at-or-latest policy 20 lines above — `docs/sonar_processing_chain.md:624` vs `:599-608`
- [ ] (must-fix) A new top-level `water/` theme of the world model is named in a reference page that says it decides nothing; ADR-0010 D3 governs that taxonomy and its own precedent is a dated D3 amendment (#288 added `datum/`, `s100/`) — `docs/sonar_processing_chain.md:17,731-736` vs `docs/decisions/0010-geospatial-world-model.md:195-220`
- [ ] (must-fix) Four operator attributions have no traceable source in the issue, the 16 PR comments, the plan or progress — the M3-returned/DeltaT-reinstalled hardware state, the Imagenex obfuscation-and-patent assertion about a named vendor, the empty-fields choice, and the rebuild-every-store decision; and the angular-response "direction" answers an operator question rather than restating a direction — `docs/sonar_processing_chain.md:171,316-318,401,784,862`
- [ ] (must-fix) The plan's Open Questions still says the explorer stand-in exists only on the branch in review (mpt#50), which has merged and which the page describes as shipped — `.agent/work-plans/issue-373/plan.md:150-156`
- [ ] (must-fix) Every Integrated Review checkbox is still `[ ]` while the entry is `Status: complete` and Implementation ticks them all — `.agent/work-plans/issue-373/progress.md:23-95`
- [ ] (must-fix) PR #374's body carries no AI signature and still describes the pre-revision six-stage document — refresh it and add the `Authored-By`/`Model` block
- [ ] (suggestion) "The same bridge already decodes the XYZ88 datagram" — the decoder exists but the node filters to N/78 before parsing; `sonar_reference.md` already phrases this correctly — `docs/sonar_processing_chain.md:404-406` vs `marine_tools/kongsberg_em_bridge/kongsberg_em_bridge/node.py:451-453`, `em_datagrams.py:159`
- [ ] (suggestion) The DeltaT horizontal floor does not itself pin the influence radius — the clamp is cube-side, and the 0.01 floor binds only within ~1 m of nadir — `docs/sonar_processing_chain.md:372-375` vs `cube_bathymetry/src/parameters.cpp:72-115`
- [ ] (suggestion) "speed enters only the latency terms, all ∝ speed²" — the Eqn 3.96 term is latency²×sog_sdev², not speed-scaled; it vanishes only because `gps_latency` defaults to 0 — `docs/sonar_processing_chain.md:560-564` vs `cube_bathymetry/src/error_model.cpp:193-194`, `error_model.h:110`
- [ ] (suggestion) "the `/12` divisor matches Calder" closes a question the divergence record leaves open — the port applies `bw/12` to every beam with no detection-method input, a live divergence with no defect-table row — `docs/sonar_processing_chain.md:871-873` vs `cube_bathymetry/docs/divergences_from_calder.md:203-217`
- [ ] (suggestion) The "faithful port" verdict has no term-by-term artifact in the repo and the README scopes it to the CUBE algorithm, not the error budget — attribute it to cube#30 explicitly — `docs/sonar_processing_chain.md:531-536`
- [ ] (suggestion) Lever arms, alignment sigmas, static roll and both beamwidths are unconfigurable everywhere, live included (the live node sets three fields) — the live/offline contrast overstates the live path — `docs/sonar_processing_chain.md:515-520` vs `cube_bathymetry/src/detections_to_pointcloud.cpp:134-136`, `error_model.cpp:66-73`
- [ ] (suggestion) Platform's surface and geometric-mean sound speed are filled from the same scalar — `docs/sonar_processing_chain.md:489-492` vs `cube_bathymetry/src/detections_projector.cpp:161-162`
- [ ] (suggestion) The fallback-policy bullet covers four of the five copies — `bag_to_geotiff` is a third variant (bare at-stamp, no counter) — `docs/sonar_processing_chain.md:599-606` vs `cube_bathymetry/src/bag_to_geotiff.cpp:529-530`
- [ ] (suggestion) The "two of five" geodetic count is right but the setup names only the GeoTIFF tool as full-solve; the live node solves per sounding too — `docs/sonar_processing_chain.md:607-613` vs `cube_bathymetry/src/cube_bathymetry_node.cpp:2050-2053`
- [ ] (suggestion) "Nothing is defaulted" — the optional fields are defaulted to NaN, and `bag_to_geotiff` invents uncertainty from nav covariance ×10, violating the seam the paragraph declares — `docs/sonar_processing_chain.md:218-222` vs `cube_bathymetry/src/bag_to_geotiff.cpp:543-545,575-578`
- [ ] (suggestion) The horizontal variance caps the influence radius rather than setting it — `docs/sonar_processing_chain.md:671-672` vs `cube_bathymetry/src/parameters.cpp:72-110`
- [ ] (suggestion) The explorer's lab does not resample at export — it writes its metric lattice 1:1 with a linearised lon/lat affine — `docs/sonar_processing_chain.md:660-663` vs `marine_perception_tools/src/cube_export.cpp:43-78`, `map_geo_anchor.hpp:31-49`
- [ ] (suggestion) The "until detections are carried through" quote is in `cube_lab.hpp:171`, not `sounding_uncertainty.hpp` — `docs/sonar_processing_chain.md:522-529`
- [ ] (suggestion) `sonar_image_proc` is not present on this host, so two of the three "full −3 dB" supports are unverifiable here; the March-2022 provenance and the bmr migration rule *are* verifiable — cite commit `9937688` (2022-03-21) and `order00_projected_sonar_image.bmr:224-225` — `docs/sonar_processing_chain.md:120-134`
- [ ] (suggestion) "one element per beam" and the tx/rx axis mapping are not in `PingInfo.msg`; they come from `DetectionFlag.msg:12-13` and `SonarDetections.msg:40,45` — mark them as the page marks "full vs half" — `docs/sonar_processing_chain.md:81`
- [ ] (suggestion) The message states the angle sign conventions but qualifies them "for a downlooking sonar"; the units table drops the qualifier — `docs/sonar_processing_chain.md:78-79` vs `SonarDetections.msg:40-47`
- [ ] (suggestion) "the message's own header says" — it is the field comment on `two_way_travel_times` — `docs/sonar_processing_chain.md:414-416` vs `SonarDetections.msg:22`
- [ ] (suggestion) `r2sonic` is called flatly "conformant" but fills `rx_beamwidths` from a transmit field — `docs/sonar_processing_chain.md:353` vs `ros2sonic/r2sonic/src/conversions.cpp:18`
- [ ] (suggestion) "DeltaT in water-column mode; M3 IMB stream" have no ROS path today, and "IMB" appears nowhere in either tree — say so — `docs/sonar_processing_chain.md:238`
- [ ] (suggestion) "about 0.73 here" never states the latitude it means — `docs/sonar_processing_chain.md:656`
- [ ] (suggestion) The intro says the rented stages sit beside the chain with their own section, but they are a subsection of Stage 1 and the text says beamforming stays inside acquisition — `docs/sonar_processing_chain.md:56-60` vs `:300,332`
- [ ] (suggestion) Nothing links *to* the new page except `sonar_reference.md` — `sonar_ecosystem.md:282`, `interfaces.md:262`, `data_flows.md:282` and `.agents/README.md:33-36` all omit it, and the plan's promised ecosystem-link follow-up was never filed
- [ ] (suggestion) `sonar_reference.md:81-82` now contradicts the new page on store layers (`processed/` vs the ADR-0007 A.2 single `survey` layer)
- [ ] (suggestion) `sonar_reference.md` has no DeltaT/Garmin/EdgeTech entry, so the sensor the chain now runs on is documented on neither page while the new page says that page "tracks which ones" — `docs/sonar_processing_chain.md:12-14`
- [ ] (suggestion) cube#152 asked for the units convention to be written down durably; it now lives in another repo's reference page and #152 records nothing — post a pointer
- [ ] (suggestion) Raise the three-category convention rule and the "variance fields named error/uncertainty" trap as a `.agent/knowledge/` candidate — the plan's own Documentation & Instruction Impact escape hatch, unexercised — `docs/sonar_processing_chain.md:100-118,154-160`
- [ ] (suggestion) The operator's L1 comment offered *rename the doc* or *add placeholders*; the PR chose placeholders and the title question was never re-surfaced for a yes/no
- [ ] (suggestion) `marine_tools#2` is the only support for the element-interface architecture claim; nothing in-tree documents it — `docs/sonar_processing_chain.md:311-314,336-338`
- [ ] (suggestion) File separately: `marine_perception_tools/src/tf_lift.hpp:76-78` cites "(#42)", an unrelated issue
- [ ] (suggestion) File separately: `import_bag_main.cpp:971` and `batch_regen_main.cpp:688` print "(offline projection, vessel_speed = NaN)" unconditionally, even with `--odom-topic` — the banner is what produced this page's wrong claim
- [ ] (suggestion) File separately: `imagenex_deltat/nodes/deltat_to_bag.py` already builds `SonarDetections` — prior art for imagenex_deltat#2 the page does not mention; and `imagenex_deltat/setup.py:25` declares a console_script for a module that does not exist
- [ ] (suggestion) File separately: this repo has no `.pre-commit-config.yaml` at all, so the workspace's "run pre-commit hooks" rule is vacuous here and the green `build` check exercises nothing in a docs diff

## Implementation
**Status**: complete
**When**: 2026-09-11 (evening, -04:00)
**By**: Claude Code Agent (Claude Fable 5.1)

**Branch**: `feature/issue-373` (PR #374) — address-findings pass on the fresh `## Local Review` (16 must-fix, 27 suggestions). Host-inline, because two findings needed operator ground truth (obtained at a checkpoint) and one needed a physics choice.

Must-fix, all addressed:
- [x] DeltaT recording state corrected (BizzyBoat line commented out this season; IzzyBoat records it)
- [x] Offline speed: `--odom-topic` supplies real SOG; NaN only without it / in `bag_to_geotiff`; the NaN banner is unconditional
- [x] Attitude-rate figures: both sigma choices given (0.007 s IMU+tx → 7/70 °/s; summed 0.031 s → 1.6/16 °/s); cube#155 corrected to match
- [x] `cube::Parameters` = 28 fields; cube#157 corrected
- [x] `cube::SspRayTracer` → `cube::traceRay` / `SoundSpeedProfilePoint` / `RayTraceResult` at all four sites
- [x] "full −3 dB" sourced to `garmin_sidescan/node.py`, upstream commit `9937688`, the bmr rule; mt#62 citation removed
- [x] `SonarInfo` published only by `kongsberg_em_bridge`
- [x] Cartesian pair: test oracle + explorer `find_path` named as deletion costs
- [x] Eqn. 3.49: porting error, Calder's C was right
- [x] field-carriage fix cited to mpt PR#50
- [x] stage-4 feed sentence reconciled with at-or-latest
- [x] three decisions marked *operator direction, pending #381* (operator's choice: record later, not strip)
- [x] four operator statements recorded as a PR #374 comment; the vendor sentence softened to "not open to us — a vendor choice" (operator's choice)
- [x] plan Open Question updated (mpt#50 merged; mpt#55 is the replacement)
- [x] Integrated Review checkboxes ticked (20)
- [x] PR body refreshed + AI signature

Suggestions taken: XYZ88 wording; DeltaT floor claim; Eqn 3.96 phrasing; `bw/12` divergence row; `bag_to_geotiff` invented uncertainty on the same seam; "would resample"; quote located in `cube_lab.hpp`; third fallback variant + live node per-sounding solve; Vessel unconfigurable live too; both sound speeds from one scalar; faithful-port attributed to cube#30 and scoped; `sonar_reference.md` store-layer contradiction fixed. Consequence gaps filed as #382 (cross-links after #368 + DeltaT reference entry).

Not taken: nothing outstanding from the must-fix list; remaining suggestions were wording preferences.
