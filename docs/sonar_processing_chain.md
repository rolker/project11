# Sonar Processing Chain

How sonar data becomes a stored, shareable product: the stages, which component
owns each, what each stage must be fed, and where the chain is still rented
from someone else's software.

This is the third of three sonar pages, and the one to read when you are about
to write code that touches soundings:

- [sonar_ecosystem.md](sonar_ecosystem.md) — the big-picture *status* map: what
  is built, what is next, where to direct effort.
- [sonar_reference.md](sonar_reference.md) — durable *hardware and protocol*
  facts: sensor identities, wire formats, where the data of record lives. Sonars
  come and go; that page tracks which ones, and this one deliberately does not.
- **this page** — the *processing chain*: stages, ownership, inputs, units.

It records what the chain is and what it needs. It decides nothing; where a
decision governs a stage it is cited, not restated. Three directions the
operator set during this page's review — the `water/` theme, the units rule,
and the Calder extension rule — have no decision record yet; each is marked
*operator direction, 2026-09-11, pending [#381](https://github.com/rolker/unh_marine_autonomy/issues/381)*,
and the record, not this page, will decide them. The four operator statements
of fact this page rests on (hardware state, the M3 device-table choice, the
store rebuild, the angular-response direction) are recorded as a comment on
[PR #374](https://github.com/rolker/unh_marine_autonomy/pull/374) so they have
a source outside a conversation.

**What "is" means here.** This page describes what ships on the default
branches. Where a stage is being changed on an open branch, that is marked
explicitly rather than written in the present tense, because a chain document
that describes unmerged work is a chain document that lies to whoever reads it
next. Unbuilt branches of the chain are named with the contract they would
consume and produce, and nothing more — a placeholder that speculates about an
implementation is a map of things that do not exist. Everything below was read
out of source on 2026-09-11; where code and its own comments disagree, the
disagreement is recorded rather than smoothed over.

## The chain at a glance

The chain has a **spine** that every product shares, and a **middle** that
differs per product.

| | Stage | Owner | In | Out |
|---|---|---|---|---|
| spine | Acquisition | one driver per sonar — the contract is the message | sensor wire protocol, or vendor software output | `SonarDetections` / `RawSonarImage` / `ProjectedSonarImage` + latched `SonarInfo` |
| bathymetry | Projection | `cube::DetectionsProjector` | ping + TF + SOG | **sonar-frame** soundings |
| bathymetry | Uncertainty | `cube::ErrorModel` (inside the projector) | ping + `Platform` + `Vessel` + `Device` | per-sounding TPU |
| spine | Georeferencing | **no single owner — five copies** ([cube#146](https://github.com/rolker/cube_bathymetry/issues/146)) | soundings + TF | geographic soundings |
| bathymetry | Estimation | `cube::GeoMapSheet` / `cube::GeoGrid` / `cube::Node` | **`PointCloud2` on `soundings`** + `cube::Parameters` | per-node depth, uncertainty, backscatter |
| sidescan | Decode → project → mosaic | `marine_sidescan_mosaic` | `RawSonarImage` + TF + a nadir altitude (`sensor_msgs/Range`) + (optionally) a DEM | backscatter mosaic tiles |
| spine | Store write | `cube::store_import`, the mosaic node | nodes / mosaic | GGGS tiles in the world model |

Three stages generalize across products and are the spine: acquisition is a
message contract, not a multibeam idea; georeferencing is the same lift for
every product, which is exactly why its duplication spans both the multibeam
tools and the explorer's sidescan-facing helper; store write lands GGGS tiles
either way, depths in one store and backscatter in another. What does not
generalize is the middle. Projection, uncertainty and CUBE estimation turn
bottom detections into a depth surface. Decode, projection and mosaic tiers turn
sample data into a backscatter image. Water column and single/split beam are
[named branches](#the-other-branches) with no middle built yet.

Two further pieces sit beside the chain rather than in it, and have their own
sections: the [water-body model](#the-water-body-model) that every stage
needing sound speed, temperature or salinity consults, and the two
[stages we rent](#the-stages-we-rent-beamforming-and-bottom-detection) from
vendor software.

Projection and uncertainty are **one component**: `DetectionsProjector` exists
so that projection and the error model happen together and once, and its
header forbids node-bound includes so bag replay can reuse it. Georeferencing
is **not** in it, and that is the chain's weakest joint. See stage 4.

Projection and uncertainty are also **skippable**. The estimator's input is a
point cloud carrying per-sounding variances, so a sensor that already delivers
that can publish it directly. See stage 1.

## Units and conventions

Get these wrong and the numbers stay plausible while being wrong, which is how
the defects below survived. This table is the contract.

| Quantity | Unit | Convention |
|---|---|---|
| `SonarDetections.tx_angles` | radians | transmit **steering** angle at the array; positive forward |
| `SonarDetections.rx_angles` | radians | receive **steering** angle at the array; positive to starboard. `Sounding::beam_angle` is a copy of it |
| `SonarDetections.two_way_travel_times` | seconds | |
| `PingInfo.tx_beamwidths`, `rx_beamwidths` | **radians** | **full** −3 dB width, not a half-angle; `tx` = along-track (elevation, for an imaging sonar), `rx` = across-track (azimuth); one element per beam; may be empty. See the note below for where "full" comes from |
| `PingInfo.frequency`, `sound_speed` | Hz, m/s | **0 means unavailable**, not zero |
| `cube::Device` beamwidths | **degrees** | user-facing; datasheets quote degrees |
| `cube::Platform` roll, pitch | radians | roll positive port-side-up; pitch positive bow-up, negated from the REP-103 producer at the boundary |
| `cube::Platform.heave` | metres | **positive down** today (Calder's convention; negated from TF at the boundary) — see the rule below |
| `cube::Vessel` alignments, angular sdevs, static roll | degrees | user-facing |
| `cube::Sounding.depth` | metres | **elevation, positive up** |
| ray-tracer depths (`ssp_ray_tracer.h`: `cube::traceRay` over `SoundSpeedProfilePoint`s) | metres | **positive down** today, below a shared surface datum — see the rule below |
| `cube::Sounding.vertical_error`, `horizontal_error` | m², **variances** | named "error", hold variances — rename pending, [cube#158](https://github.com/rolker/cube_bathymetry/issues/158) |
| `PointCloud2` fields `vertical_uncertainty`, `horizontal_uncertainty` | m², **variances** | named "uncertainty", hold variances — the same rename, [cube#158](https://github.com/rolker/cube_bathymetry/issues/158) |

Angles on the wire are radians; angles in the `Vessel` and `Device`
configuration are degrees. That split is the single most common source of
defects in this chain, and every one of the units defects below is an instance
of it. [cube#152](https://github.com/rolker/cube_bathymetry/issues/152) opened
the question of a stated convention after the fifth instance; the rule below is
the operator's answer (2026-09-11), and lives here because it spans
`marine_tools`, `cube_bathymetry` and the explorer.

**The convention rule, in three categories** (operator direction, 2026-09-11,
pending [#381](https://github.com/rolker/unh_marine_autonomy/issues/381)).

1. **Internal interfaces between our own components follow ROS** (REP-103:
   radians, metres, z up, ENU). `Platform` is filled by our projector and typed
   by nobody; its roll and pitch already went to radians in the September fix,
   and heave should follow to positive-up, which costs nothing numerically
   because heave enters the budget only squared.
2. **User-facing values follow the literature.** `Device` and `Vessel`
   configuration stay in degrees because that is what a spec sheet and a patch
   test report say.
3. **External data formats keep their native convention only inside their
   reader.** Sound-speed casts are tabulated positive-down in every
   oceanographic dataset, so that sign lives in the cast loader and dies there.
   The ray tracer talks to the projector, an internal interface, so it should
   speak elevation; today it speaks depth-below-surface and names its result
   field the long way round so that a missing negation reads wrong at the call
   site. That naming is a mitigation for a crossing that does not need to
   exist; the tracer is not yet in the production path, so this is the cheap
   moment to change it.

**Where "full −3 dB width" comes from.** The message comment ("Sonar reported
-3db beamwidths, reported in radians") entered `PingInfo` in March 2022 in a
commit titled "committing changes from meeting" and says neither full nor half.
The answer is in the co-maintainer's own consumers and in the migration rule:
`sonar_image_proc`'s Python metadata sets the elevation extent to
`±0.5 × tx_beamwidth`, its C++ interface compares against
`tan(tx_beamwidths[0] / 2)`, and the v1→v2 bag migration copies the old
vendor-defined `azimuth_beamwidth` / `elevation_beamwidth` into the arrays with
no factor of two. The one driver here that publishes beamwidths agrees:
`garmin_sidescan`'s node states its table holds "the FULL -3 dB widths, NOT
half-angles" (`garmin_sidescan/node.py`). The March-2022 provenance is commit
`9937688` in the upstream repository; the migration rule is
`bmr/order00_projected_sonar_image.bmr`. (The `sonar_image_proc` sources are
cited from the upstream repository, not from a local checkout.)
One consumer still reads it as a half-angle and draws every fan twice as wide:
[rviz_sonar_image#9](https://github.com/rolker/rviz_sonar_image/issues/9).
Getting that sentence into the message definition is a documentation-only
change upstream; the tracking issue for the message update path as a whole is
[#380](https://github.com/rolker/unh_marine_autonomy/issues/380).

**Two further traps.** `Sounding::depth` and the ray tracer's
`depth_below_surface` have **opposite signs** today. And the variance fields
hold variances, so a consumer that squares them is wrong by a square — and a
producer that writes a standard deviation into them is wrong by a square root,
which is exactly what the DeltaT driver did (stage 1).

**Diverging from Calder** (operator direction, 2026-09-11, pending
[#381](https://github.com/rolker/unh_marine_autonomy/issues/381)). The estimator is a port of Brian Calder's CUBE and
error model, which ship in `cube_bathymetry/original_cube/` and are known in the
hydrographic world. Every divergence is recorded in
`cube_bathymetry/docs/divergences_from_calder.md`, and the operator has set the
three justifications a divergence may cite (2026-09-11):

- **device independence** — no built-in device tables; drivers and parameters
  carry the variation;
- **conforming to ROS standards** — a reason to choose different units or
  signs;
- **evolving technology** — RTK GPS replacing tide, draft and heave, for one.

And one rule for *extending* rather than diverging: an added or substituted
error-model term must be defensible in the same terms as Calder's. Locate it in
the published error-budget literature his equation numbering follows, or derive
it with a citation and back it with data, and record it in the divergence
document as an extension, never as Calder. Dropping positive-down *inside* the
port is not a divergence from Calder's intent, since his own development notes
keep it as an inside-only convention; it moves where the boundary sits.

## Stage 1 — Acquisition

**What it does.** Gets a sensor onto a contract the rest of the chain can
consume.

**Owner: one driver per sonar, and the right adapter for what that sonar
delivers.** This is the only stage expected to be rewritten repeatedly, because
the sonar changes with the platform and with what is available. BizzyBoat
carried a Kongsberg M3 through the 2026 season; the M3 has been **returned** and
the Imagenex DeltaT **reinstalled** (operator, 2026-09-11), and the next
power-up needs this stage adapted to it — see [the DeltaT today](#the-deltat-today).
Ben or DriX may carry an EM2040 or similar. Nothing downstream of this stage
should name a sonar, and where the rest of this page does, that is a defect in
the page.

**Read the rest of this page with one caveat.** The bathymetry branch was
developed and refined while the M3 was the sonar on the boat, so its assumptions
are M3-shaped in places nobody has had reason to test. A change of sonar is a
work item with real content, not a configuration change, and the first such
change is now scheduled rather than hypothetical.

### The message contract

Acquisition is a message contract, not a driver. `marine_acoustic_msgs` carries
one shared metadata block, `PingInfo` (frequency, sound speed, beamwidths), and
three raw message types that share it:

| Message | Carries | Produced today by |
|---|---|---|
| `SonarDetections` | per-beam travel times, steering angles, flags, intensities — bottom detections | `kongsberg_em_bridge` (M3); `r2sonic`, `norbit_driver` (external) |
| `RawSonarImage` | per-beam sample data — water column, sidescan, single/split beam | `garmin_sidescan`, `edgetech_sonar` |
| `ProjectedSonarImage` | imaging-sonar fans with beam directions and range bins | (external imaging-sonar drivers) |

Alongside its raw message a driver should publish a latched
`marine_interfaces/SonarInfo` ([ADR-0009](decisions/0009-sonar-info-message.md))
carrying what `PingInfo` lacks: acquisition settings, intensity semantics, and
the correction state of the stream. `SonarInfo` is prototyped here on purpose,
outside the upstream message package, as the candidate for upstreaming
([#380](https://github.com/rolker/unh_marine_autonomy/issues/380)). **Only
`kongsberg_em_bridge` publishes it today**; `garmin_sidescan`, `edgetech_sonar`
and `imagenex_deltat` do not reference the message at all.

### What the estimator actually consumes

Not `SonarDetections`. The live CUBE node subscribes to a **`PointCloud2` on
`soundings`**, in the sensor frame, with named fields:

| Field | Required | Meaning |
|---|---|---|
| `x`, `y`, `z` | yes | position relative to the sonar head, metres |
| `vertical_uncertainty` | yes | **variance**, m²; must be finite and positive — to be renamed `vertical_variance` ([cube#158](https://github.com/rolker/cube_bathymetry/issues/158)) |
| `horizontal_uncertainty` | yes | **variance**, m²; must be finite and non-negative — to be renamed `horizontal_variance` |
| `intensity` | no | backscatter; NaN means not reported, not zero |
| `beam_angle` | no | radians; needed for the angular-response corrections |

The two optional fields are probed by name, so a producer may omit them. The
two variance fields are not: a cloud without them throws while the iterators
are built, and the ping is dropped with a throttled warning. A point whose
position or variance is non-finite, or whose vertical variance is non-positive,
is dropped and counted. **Nothing is defaulted on this path.** A sensor that
cannot say how good its soundings are does not get to have them guessed at. One
offline tool breaks that rule on the same seam: `bag_to_geotiff` invents a
sounding uncertainty from the navigation covariance scaled by ten, which is a
substitution the product does not record.

That contract is the seam. Everything above it is per-sensor; everything below
it is shared.

### Two axes, not one list

Data can join the chain at more than one point, and it can arrive with or
without everything the next stage needs. Those are different questions with
different answers, and conflating them is how the DeltaT's invented
uncertainties went unnoticed for two seasons.

**Entry point** is where the data joins. It says which stages are skipped, and
it is a routing question, solved by having the right adapter.

| Entry point | Arrives as | Then | Sensors |
|---|---|---|---|
| element data, before beamforming | vendor raw interface | beamform → detect → the full chain | M3, DeltaT — both rent this today; see [the stages we rent](#the-stages-we-rent-beamforming-and-bottom-detection) |
| beam samples | `RawSonarImage` | detect → the full chain (bathymetry); or the sidescan / water-column branches | DeltaT in water-column mode; M3 IMB stream; sidescans |
| bottom detections | `SonarDetections` | project (2) → uncertainty (3) → cloud → the rest | M3 via `kongsberg_em_bridge`; the DeltaT once [imagenex_deltat#2](https://github.com/rolker/imagenex_deltat/issues/2) lands |
| sonar-frame soundings with variances | the `PointCloud2` contract | georeference (4) → estimate (5) | EM2040 and similar, via the manufacturer's own model; the DeltaT today, with invented variances |
| georeferenced soundings | `cube::GeoSounding` (offline) | estimate (5) | bag import, batch regeneration |

The richest path is the detections one, because the error model then has the
geometry that produced each sounding. The soundings-with-variances path is not a
special case bolted on; it is why the seam is a point cloud rather than a
detections message, and an early real-time CUBE test ingested exactly such a
cloud. Two obligations come with it: the variances must be **variances in m²**,
not the standard deviations or 95 % bounds a manufacturer is more likely to
quote; and the provenance should be recorded, because a surface built from
vendor uncertainty and one built from our error model are not the same product
even where the numbers agree.

**Completeness** is whether the arrival can proceed unaided, and it is a policy
question about what may be substituted and by whom. The policy is the same no
matter where the data entered:

1. **Publish at the richest entry point the sensor supports.** Most sensors can
   do better than their driver currently does; the DeltaT is the worked
   example.
2. **A missing device constant** (a beamwidth) belongs in configuration read by
   the stage that needs it — the device table
   [marine_tools#82](https://github.com/rolker/marine_tools/issues/82) adds —
   not in a per-ping node, which adds a hop and hides the origin.
3. **A missing per-sounding estimate** (an uncertainty) is an error-model
   choice and belongs in the error model as a **named, selectable** model.
   Never an invented constant, anywhere. Fill in as early as possible: every
   stage discards information, and a node sitting on a point cloud cannot see
   beam angles or travel times, so it is structurally limited to a depth-based
   guess even for sensors that could support the full model.
4. **A whole message class** (the sensor speaks one contract, the consumer
   wants another) is a transformation node. Two exist: `detections_to_pointcloud`
   and `marine_sonar_to_pointcloud`.
5. **Anything substituted is stamped into the product**, not only recorded in a
   configuration file. The DeltaT soundings in the store today carry no trace
   that their variances were invented, so it cannot be recovered after the fact.
   [ADR-0005](decisions/0005-multi-platform-provenance-registry.md) covers this ground for
   cross-store sources and the same shape applies here.

**The positions-only class is empty.** An earlier revision of this page
described a fifth arrival, `x, y, z` and nothing else, and recommended porting
Calder's depth-based IHO error model for it. Every sensor we own can supply
geometry, so the class has no members, and the model stays unported for three
reasons: its horizontal term is a constant independent of depth and angle, and
horizontal variance caps the CUBE influence radius, so it would smear a shallow
survey uniformly; its coefficients are a survey *acceptance* limit, so feeding
them in as measured uncertainty writes a policy number into the store where a
physical one belongs; and nothing needs it. The honest answer for a sensor that
truly cannot give geometry is to say so, not to substitute a standard.

**Recording follows the same rule as publishing: record the earliest entry
point the sensor supports, never the derived cloud.** BizzyBoat's logging
launch records the M3 as detections plus the sonar-info message, so every M3
bag survives a change to the cloud contract untouched — the cloud regenerates.
The DeltaT, where it is recorded, is recorded as its cloud, because the driver
has nothing earlier to record — BizzyBoat's launch has that topic commented out
this season (the M3 replaced it) and IzzyBoat's records it; those are the only bags [cube#158](https://github.com/rolker/cube_bathymetry/issues/158)
touches, and a one-off retrofit script handles them
([unh_echoboats_project11#489](https://github.com/rolker/unh_echoboats_project11/issues/489))
rather than compatibility code in the pipeline.

### The stages we rent: beamforming and bottom detection

Both sonars we own produce **element-level data**, and for both, the
beamforming is done today by the vendor's Windows application. Both ROS
acquisition paths are therefore downstream of that software: `kongsberg_em_bridge`
consumes a `.all` UDP stream *exported by* the M3's own application, and the
DeltaT driver reads the head's profile output. Neither talks to an array. The
Raw Range and Angle datagram the bridge decodes is post bottom-detection, so
two algorithms sit between the array and our first message, and both are
rented. They fail differently, and they point at opposite sonars.

**Beamforming points at the M3.** Kongsberg documents the raw element
interface, so owning this stage is decoding a described protocol
([marine_tools#2](https://github.com/rolker/marine_tools/issues/2) records the
architecture: the head sends elements over Ethernet to the Windows software,
which performs all beamforming). **The DeltaT's element interface is not open
to us** — a vendor choice, recorded outside this page (operator, 2026-09-11;
see the PR #374 comment). That path is closed, and the judgement about it is
the operator's, not a technical estimate.

**Bottom detection points at the DeltaT.** Its software outputs water column
*or* soundings, never both, so wanting water column forces us to detect. The M3
hands over its water-column imagery and its `.all` bathymetry simultaneously,
so it forces nothing — which makes it the one platform where our own detector
could be developed against the vendor's detections as a reference, then run on
DeltaT water column where none exist. This is also the stage where we already
have defects we cannot fix: the shallow-water flyers of
[cube#110](https://github.com/rolker/cube_bathymetry/issues/110) and the M3
false-detection outliers are bottom-detection defects, and the detector lives
in the vendor's box, so the only remedies have been downstream gates.

Beamforming stays inside acquisition rather than becoming a stage of its own,
because its output is the same message contract and nothing downstream changes.
What earns it this subsection is that it needs inputs nothing else in the chain
needs — array geometry, per-element calibration, and a shading choice that
*determines the beamwidth*. If we beamform, we know the beamwidths exactly,
which retires the device-table guesswork in marine_tools#82, where the M3 is
deliberately left uncharacterised because no datasheet figure exists.

Neither capability is scheduled. The operator's direction is that the pipeline
must be able to accommodate both so they can be tried; no issue is filed until
the shape is settled.

### Where each multibeam driver stands

Sidescan sensors are **not** in this table. They feed the sidescan branch, not
the CUBE error model, so `garmin_sidescan` is out of scope here however similar
its beamwidth handling looks.

| Driver | Arrives as | Beamwidths | State |
|---|---|---|---|
| `marine_tools/kongsberg_em_bridge` (M3) | raw detections | empty, deliberately | conformant apart from beamwidths; a device table is on a branch, not yet published ([marine_tools#82](https://github.com/rolker/marine_tools/issues/82)); to be renamed `kongsberg_dotall` ([marine_tools#84](https://github.com/rolker/marine_tools/issues/84)) |
| `r2sonic` | raw detections | assigned | conformant |
| `norbit_driver` | raw detections | `resize()`d, never assigned — **an array of zeros** | the consumer now rejects non-positive widths and falls back ([cube#144](https://github.com/rolker/cube_bathymetry/issues/144)); the driver still publishes zeros |
| `imagenex_deltat` | **a cloud with invented variances** | none | not conformant; to become a detections source ([imagenex_deltat#2](https://github.com/rolker/imagenex_deltat/issues/2)) |

### The DeltaT today

`imagenex_deltat/nodes/deltat.py` receives the head's 83P records over UDP and
publishes the `PointCloud2` contract directly — **with both variance fields
present**, computed as:

```
v_uncertainty = max(0.1, depth * 0.01)
h_uncertainty = max(0.01, abs(yoffset * 0.01))
```

The commit that added them is titled "Add hacked uncertainty to be able to work
with cube", and it predates the detections-to-pointcloud step: it was the only
way to reach CUBE at the time, not a shortcut past an existing path. Read cold,
those lines look like carelessness; they are an artifact of ordering. They are
nonetheless wrong in four ways now. A one-percent-of-depth figure is shaped
like a standard deviation and the estimator reads the field as a variance; the
horizontal floor of 0.01 read as a variance is σ = 0.1 m; the floor binds only
within about a metre of nadir, and outboard the one-percent term understates
the horizontal budget by the same square-root confusion, so the CUBE influence
radius is wrong everywhere and smallest where the beams are worst. The intensity field is named `i`, so
backscatter never arrives. There is no `beam_angle` field, so the angular
corrections cannot run. And the head's tilt angle is parsed and ignored, with
every point placed at zero along-track.

The 83P record carries per-beam ranges, a start angle and increment, the sound
speed and the range resolution — everything needed to publish `SonarDetections`
and let the real projector and error model run. That moves the DeltaT from the
weakest arrival class to the strongest and deletes the hack; it needs a DeltaT
entry in the device table for beamwidths. That is the acquisition work for the
next BizzyBoat power-up, [imagenex_deltat#2](https://github.com/rolker/imagenex_deltat/issues/2),
together with recording the detections topic and moving the boat's vessel
configuration off the returned M3.

### Why the M3 bridge leaves beamwidths empty

`tx_beamwidths` and `rx_beamwidths` are left empty **on purpose**, with a
comment saying so: at the time, the error model treated those arrays as degrees
while the message says radians, so leaving them empty avoided the mismatch. The
consumer fix has now landed (cube_bathymetry PR#153, closing
[cube#144](https://github.com/rolker/cube_bathymetry/issues/144) and
[cube#147](https://github.com/rolker/cube_bathymetry/issues/147)): the boundary
normalises units and validates each per-beam width as finite, positive and
below π before believing it. The driver-side device table
([marine_tools#82](https://github.com/rolker/marine_tools/issues/82)) is
implemented on a branch and not yet published; the M3 is **deliberately
uncharacterised** in it — no datasheet figure exists anywhere, and the operator
chose empty fields plus the device fallback over inventing one.

The M3's own datagram stream does not carry beamwidth either, so for any
Kongsberg unit the figure has to come from a device table rather than the wire.
The same bridge carries a decoder for the XYZ88 datagram — the node filters to
the Raw Range and Angle datagram before parsing, so it is not used — which the
M3 exports empty but an EM2040 would populate. The package is to be renamed **`kongsberg_dotall`**
([marine_tools#84](https://github.com/rolker/marine_tools/issues/84)) — it
decodes a format and bridges nothing — after the #82 branch lands and riding the
same boat-config change as the DeltaT swap.

### What the beam angle means

`tx_angles` and `rx_angles` are documented as the *steering* angles applied to
the transmit and receive beams, on a message whose own header says truly raw
multibeam data uses travel times rather than ranges. They are array-side
quantities by definition, before any ray tracing and before any seabed
geometry. The contract is not ambiguous; what differs between sonars is what a
consumer can *do* with the angle:

- The **M3 does not use a sound-speed profile.** It works from surface sound
  speed alone, so its reported receive angle and its geometry are consistent by
  construction — in our chain the position is *computed from* that angle and
  one sound speed, so the two cannot disagree.
- An **EM2040 ray-traces through a profile.** Its soundings are refracted, so
  the angle it reports is still a steering angle at the array while the
  position has been bent on the way down. The array-side angle, the apparent
  angle from the head to the sounding, and the angle at which the sound met the
  seabed are then three different numbers.

**The discrepancy is worth measuring, not just avoiding.** When a sensor
supplies both a beam angle and an `x, y, z`, the apparent angle implied by the
position can be compared with the reported launch angle. Under a uniform water
column they agree; the difference is the accumulated refraction — a cheap,
self-contained check on whether the profile in use matches the water, and a
signal the [water-body model](#the-water-body-model) can consume
([#300](https://github.com/rolker/unh_marine_autonomy/issues/300),
[marine_perception_tools#28](https://github.com/rolker/marine_perception_tools/issues/28)).

What consumes the angle is the angular-response correction
([ADR-0007](decisions/0007-mbes-backscatter-store.md),
[cube#81](https://github.com/rolker/cube_bathymetry/issues/81)), which indexes
an empirical per-sonar curve by `|beam angle|` in degrees. **`SonarInfo` does
not declare which angle its curve was built against** — steering, apparent or
incidence — which is empty for a straight-line sonar and not empty for a
ray-tracing one. That declaration is
[#378](https://github.com/rolker/unh_marine_autonomy/issues/378). It matters
less than it did, because the direction for the curve itself has changed: see
[the angular-response decomposition](#the-angular-response-decomposition).

## Stage 2 — Projection

**What it does.** Turns each beam's travel time and steering angles into a point
**in the sonar frame**. It does not georeference; that is stage 4.

**Owner.** `cube::DetectionsProjector`, which delegates the per-beam arithmetic
to the `cube::Sounding` detections constructor and writes
`Sounding::sonar_relative_position`.

**What it must be fed.** `two_way_travel_times`, `tx_angles`, `rx_angles` and
`ping_info.sound_speed`. Range is `twtt * sound_speed / 2`, so a ping whose
`sound_speed` is the "unavailable" 0 projects every beam to the origin. There is
a range gate: a sounding is kept only when its slant range is within
`[minimum_range, maximum_range]`, and the count dropped is reported in the
diagnostics rather than logged by the projector itself.

**Implemented more than once.** `marine_perception_tools/src/mbes_geometry.hpp`
re-implements the same projection for the survey explorer, with the same formula
and a single sound speed. Its header calls itself a QC-grade projection, which
is honest, but it is a second copy of stage 2.

**Straight-line projection is what ships.** Re-projection through a real
sound-speed profile is the job of the ray tracer (`ssp_ray_tracer.h`, the free
function `cube::traceRay`), which is a consumer of the
[water-body model](#the-water-body-model) and is described there. An earlier
revision of this page said the tracer was consumed by the sound-speed inversion
work; it is not consumed by anything outside its own source and tests.

## Stage 3 — Uncertainty

**What it does.** Attaches a per-sounding vertical and horizontal error budget.
This decides how much CUBE trusts each sounding, and it is the least validated
stage in the chain.

**Owner.** `cube::ErrorModel::compute(detections, platform)`, constructed with a
`Vessel` and a `Device`.

**What it must be fed.**

- *Per ping, via `cube::Platform`*: timestamp, roll, pitch, heave, surface sound
  speed, geometric-mean sound speed, and vessel speed over ground. The projector
  draws roll, pitch and heave from TF, fills both sound speeds from the ping's
  single value, and takes speed over ground as an argument.
- *Per beam, via `SonarDetections`*: travel times, tx and rx steering angles,
  detection flags, and from `ping_info` the sound speed, frequency and
  beamwidths — the last validated at the boundary, with a per-beam width that
  is non-finite, non-positive or ≥ π falling back to the `Device` figure.
- *As configuration, via `Vessel`*: lever arms to the GPS and IMU, alignment and
  latency standard deviations, draft, static roll, and the measurement standard
  deviations for roll, pitch, gyro, sound-speed profile, heave and draft — none
  of which is configurable anywhere today, live included; the live node sets
  three fields and leaves the rest at the defaults. Also
  `ellipsoidal_referenced`, which when true (the default) omits the tide terms
  from the vertical budget — and only the tide terms; see below.
- *As configuration, via `Device`*: across-track and along-track beamwidths in
  degrees, range error as a fraction, and an absolute range-error floor.

**What happens when an input is absent.** A missing attitude transform leaves
roll and pitch NaN, so the resulting uncertainty is NaN, and the projector
reports it in `diagnostics.missing_attitude`. A missing heave transform defaults
heave to zero, which the projector documents as non-critical because heave
enters the budget only squared. Speed over ground may be passed as NaN, and is
floored to zero. Bag import and batch regeneration take an odometry topic and
supply a real per-ping speed when it is given, and NaN when it is not (both
print the NaN banner unconditionally, which misled an earlier revision of this
page); the GeoTIFF tool always passes NaN. A run without odometry therefore
carries **zero latency error** in its horizontal budget while the live node
carries the full term, and nothing in the product says which it was.

**Neither `Vessel` nor `Device` has an offline configuration path.** The offline
tools set only the frames and the range gate and leave both structs at their
defaults — zero lever arms, nominal standard deviations, 2° beamwidths — so an
archive cannot be reprocessed with the real geometry. That is
[cube#145](https://github.com/rolker/cube_bathymetry/issues/145), and the
zero-speed point above belongs beside it.

**What the survey explorer runs instead.** The explorer's CUBE lab does not run
this model. It computes an angle-aware placeholder
(`marine_perception_tools/src/sounding_uncertainty.hpp`, stored as variances
per the `Sounding` contract, seeded from `cube::Device`'s defaults), and the
lab's header (`cube_lab.hpp`) says it lasts "until detections are carried
through" — now [marine_perception_tools#55](https://github.com/rolker/marine_perception_tools/issues/55). The explorer's bag
reader holds the ping and the transform buffer at the same moment, so carrying
detections through to `DetectionsProjector` is practical rather than
aspirational.

**Validation status.** The estimator core was compared term by term against
Calder's original C in [cube#30](https://github.com/rolker/cube_bathymetry/issues/30).
The verdict there, as recorded on that issue, is worth repeating: the
**estimator is a faithful port** — feed it the same soundings with the same
uncertainty and the grid matches Calder — a verdict on the CUBE algorithm, not
on the error budget. The divergences are concentrated in this stage, the upstream error
budget.

**What the model does not contain.** Read against Calder's original on
2026-09-11, with the operator's questions in hand:

- **RTK referencing is half-done.** `ellipsoidal_referenced` removes the two
  tide terms and nothing else. Draft, dynamic draft and loading are still summed
  into the static vertical budget unconditionally, and heave still enters on
  every ping. Under RTK the antenna height gives the transducer height through
  the lever arm, so all four are *replaced* by the GNSS vertical uncertainty
  plus the lever-arm attitude terms. Today the model charges for both worlds at
  once and gets neither GNSS term.
  [cube#156](https://github.com/rolker/cube_bathymetry/issues/156).
- **No attitude rate anywhere.** The three timing sigmas the vessel carries are
  summed into one latency variance (σ ≈ 0.031 s with the defaults, dominated
  by the 0.03 s GPS term) and used in exactly one place, the along-track jitter
  term scaled by speed². Roll and heading uncertainty are static. So a ping timestamped δt from its attitude sample carries an attitude
  error of `rate × δt` that the model never sees — the mechanism behind turn
  noise, and reachable on a straight line in a chop. Which timing sigma the
  coupling should use is part of the derivation: attitude-to-ping timing is the
  IMU and transmit terms (≈ 0.007 s together), on which roll rate × latency
  matches the static roll sigma at about 7 °/s and the heading half needs
  about 70 °/s; using the code's summed variance including the GPS term gives
  about 1.6 °/s and 16 °/s. Either way the heading half is expected to be
  small and the roll half is not. The term is to be
  located in the literature or derived and cited, and sized against Massabesic
  and Isles of Shoals bags before it is added:
  [cube#155](https://github.com/rolker/cube_bathymetry/issues/155).
- **A stationary pinging vessel** is not a missing speed term: speed enters only
  the latency terms, three of which scale with speed² and vanish at rest (the
  fourth, Calder's Eqn. 3.96, is latency² × speed-over-ground sigma², and is
  zero only because the GPS latency defaults to zero).
  The artifacts under a stationary boat come from CUBE assuming soundings are
  independent while hundreds of the same geometry land in the same nodes — a
  node-level question, to be looked at in a dockside bag before modelling.
- **Detection flags are never read**
  ([cube#154](https://github.com/rolker/cube_bathymetry/issues/154)); the
  `1/cos(angle)` beamwidth widening is omitted and is device-dependent in
  Calder ([cube#148](https://github.com/rolker/cube_bathymetry/issues/148));
  `static_roll` still gates in degrees against an angle in radians
  ([cube#150](https://github.com/rolker/cube_bathymetry/issues/150)).

## Stage 4 — Georeferencing

**What it does.** Lifts sonar-frame soundings into a geographic frame, by
looking up `earth <- <sonar frame>` at the ping stamp and converting through
ECEF to latitude and longitude.

**Owner: nobody.** This is the finding that most deserves your attention. The
lift is written out five times:

| Where | File |
|---|---|
| live CUBE node | `cube_bathymetry/src/cube_bathymetry_node.cpp` |
| bag to GeoTIFF | `cube_bathymetry/src/bag_to_geotiff.cpp` |
| store import | `cube_bathymetry/src/import_bag_main.cpp` |
| batch regeneration | `cube_bathymetry/src/batch_regen_main.cpp` |
| survey explorer | `marine_perception_tools/src/tf_lift.hpp` |

The node's own comment says it is "mirroring `bag_to_geotiff.cpp`", which is the
duplication admitting itself in a code comment. Four of the five live in one
package, so this is not even a cross-repo problem.

**It is not a plain lookup, and it is one function, not two variants.** The
lookup call is the same in all five. What differs is everything around it, and
each difference has already produced a defect:

- **Fallback policy.** The live node and the explorer fall back to the latest
  transform when the stamp is uncovered; import and batch regeneration use a
  bounded buffer and drop-and-count; the GeoTIFF tool is a third variant, a
  bare at-stamp lookup with no counter. Both are right for their setting. A shared
  function takes the policy explicitly — at-stamp-only or at-or-latest — over
  the same `tf2::BufferCore` the projector already takes, which offline tools
  already fill from bag transforms. The difference between real-time and
  post-processing is that argument, nothing else.
- **The geodetic conversion.** The GeoTIFF tool solves the full ECEF→geodetic
  conversion per sounding. Import and batch regeneration solve it once per ping
  at the sensor origin and linearize each sounding through a local ENU tangent
  plane (the import performance work); the live node, like the GeoTIFF tool,
  solves per sounding. Two of the five copies therefore compute a slightly
  different number from the other three, by an amount stated nowhere.
- **Field carriage.** The explorer's copy dropped beam angle and slant range
  when it rebuilt the sounding
  ([marine_perception_tools#49](https://github.com/rolker/marine_perception_tools/issues/49),
  fixed in [marine_perception_tools PR#50](https://github.com/rolker/marine_perception_tools/pull/50):
  the two copies became one helper with copy-then-overwrite semantics, so a
  field added later rides along).

The shared function belongs beside `DetectionsProjector` under the same header
rule — no node-bound includes — which is the extraction that was done for
projection and the error model and then stopped one stage short. That is
[cube#146](https://github.com/rolker/cube_bathymetry/issues/146), with the three
points above recorded on it.

**What it must be fed.** A TF buffer containing the `earth` frame at the ping
stamp — or, under the at-or-latest policy, the latest transform it holds. A
ping with no `earth` transform at all is dropped, and on the live node the grid
simply does not update.

## Stage 5 — Estimation

**What it does.** Runs CUBE: each sounding is spread over an influence radius
— Calder's capture distance, computed from the ratio of the depth-dependent
maximum allowed variance to the sounding's vertical variance, scaled by the
node spacing, **capped** at the 99 % horizontal bound (2.576 × the horizontal
sigma) and never below one node spacing — and each node maintains competing
depth hypotheses with Kalman updates, monitoring and intervention.

**Owner.** `cube::GeoMapSheet` over `cube::GeoGrid` over `cube::Node`, tuned by
`cube::Parameters`. The node — the hypotheses, the median pre-queue, the
Bayes-factor and run-length decision — is space-agnostic and shared; the grid's
job is spreading and indexing.

**One grid, geographic.** The package also holds a Cartesian pair, `Grid` and
`MapSheet`, in metres, from Calder's mapsheet. Nothing in production calls it:
the live node, import, batch regeneration and the GeoTIFF tool all migrated to
the geographic sheet. What remains is includes, comments, a forward
declaration, one test that uses the Cartesian sheet as the correctness oracle
for the geographic one (`test_publish_equivalence.cpp`), and a `find_path` for
its header in the explorer's build — so deletion ports that test and touches
that build file. Calder's mapsheet was in metres because his soundings arrived
projected; ours arrive georeferenced, and that is not a reason to keep an
equivalent. The Cartesian pair is to be **deleted**, the single library
`estimate()` entry point ([cube#129](https://github.com/rolker/cube_bathymetry/issues/129))
built on the geographic grid, and the explorer's cube lab — which today bypasses
both sheets and drives nodes directly with its own copy of the insert loop,
because the plain grid could not give it backscatter
([cube#130](https://github.com/rolker/cube_bathymetry/issues/130)) — moved onto
it. That takes the insert loop from three copies to one.

**What a GGGS cell is.** A quadtree level fixes a grid span in degrees; a cell
is that span divided by the rows per grid, and the same angular span is used
for latitude and longitude everywhere below 72° latitude. So the lattice is
**equal-angle, not equal-length**: north-south spacing is close to the level's
nominal size everywhere, and east-west spacing is the nominal size times
cos(latitude) — about 0.73 here. A level chosen for one-metre cells gives nodes
about one metre apart north-south and about 73 cm apart east-west. The
estimator is unaffected: `GeoGrid` computes every influence and capture
distance in metres (an equirectangular approximation, sub-millimetre against
the geodesic at these radii). What is not metric is the spacing of the nodes,
which is why a product's cell size is two numbers (stage 6) and why a lab
comparing against a reference implementation on a square lattice would have to
resample at export (the explorer does not today).

**What it must be fed.** On the live path, the `PointCloud2` contract from
stage 1, which this node georeferences itself before accumulating (that is the
stage-4 duplication). Offline, the same content arrives as `cube::GeoSounding`.
Either way the estimator needs depth, a vertical variance and a horizontal
variance, plus intensity, beam angle and slant range when the backscatter
product is wanted ([ADR-0007](decisions/0007-mbes-backscatter-store.md)). The
horizontal variance caps the influence radius, so an under-stated horizontal
budget pins every radius to the cell size.

**Backscatter is already carried.** The node co-estimates intensity alongside
the winning depth hypothesis, the geographic sheet configures the angular
correction, and the geographic grid exposes both through its node records. The
gap was only ever on the Cartesian side.

**Parameters.** `cube::Parameters` has 28 fields and, today, header comments as
their only description anywhere. A reference page is
[cube#157](https://github.com/rolker/cube_bathymetry/issues/157), rendered by
the explorer's lab as its parameter pop-out
([marine_perception_tools#54](https://github.com/rolker/marine_perception_tools/issues/54))
rather than copied. One thing that page must say: the two IHO fields feed
`maxVarianceAllowed()`, Calder's use of the survey-order allowance as an
acceptance gate on the *estimate* — legitimate, and not the unported IHO error
model of stage 1.

## Stage 6 — Store write

**What it does.** Turns estimated nodes, or a mosaic, into GGGS tiles on disk.

**Owner.** `cube::store_import`, writing through `marine_bathymetry_store` and
`marine_mbes_backscatter_store`; the sidescan mosaic node, writing through
`marine_tiled_raster_store`. The live CUBE node persists into the `draft/` layer
directly, with a flush-and-close-checked write that is explicitly **not
crash-atomic**.

**What it must be fed.** Estimated nodes plus the store layout rules from
[ADR-0002](decisions/0002-bathymetric-data-store.md),
[ADR-0010](decisions/0010-geospatial-world-model.md) and
[ADR-0011](decisions/0011-overview-pyramid.md). Layer naming is a live trap: the
MBES backscatter store was collapsed to a single `survey` layer by ADR-0007 A.2,
and imagery layer names are not renamed to match the depth theme. A tile's cell
size is **two numbers**, north-south and east-west, for the reason given in
stage 5.

**Live transport is already source-agnostic.**
[ADR-0008](decisions/0008-live-sonar-coverage-transport-and-render.md) defines one tile message
with named self-describing bands, and a render interface with the file store,
the live cache and plain rasters as peers, so a consumer dequantizes anything
without per-source knowledge. The producers that exist are the CUBE node and
the bag importer. The live sidescan mosaic node writes its tiles to disk and
publishes nothing, so live sidescan coverage in CAMP is one missing publisher
on an existing transport ([#379](https://github.com/rolker/unh_marine_autonomy/issues/379)),
and an onboard coverage planner is a later consumer of the same tile catalog.

**A different product, not a duplicate stage.** The explorer exports a single
GeoTIFF for sharing through `cube_export`, rather than tiles. Two places
nonetheless know how to turn nodes into rasters.

## The water-body model

Operator direction, 2026-09-11, pending a decision record
([#381](https://github.com/rolker/unh_marine_autonomy/issues/381); ADR-0010 D3
governs the store taxonomy and its precedent is a dated amendment). Sound speed ingestion and inference belong with
temperature and salinity in one **water-body model** — "ocean model" in prose,
where ocean means any water body, rivers and lakes included — whose fidelity
ranges from a pure unknown represented by the default 1500 m/s up to a full
time-varying 3-D model, populated by whatever is available and refined by the
sonar data when it can be.

**It is a theme of the world model, not a store owned by one algorithm.** The
geospatial world model ([ADR-0010](decisions/0010-geospatial-world-model.md))
organizes the collection by source class, and its fourth decision says layers
encode process while σ encodes trust. The theme is named **`water/`**, a
sibling of `depths/`, `charts/`, `datum/` and `imagery/`. Each property — sound
speed, temperature, salinity — gets provenance layers named for how the values
entered: cast, surface sensor, climatology, inverted. Fidelity is then a
property of the data, not the code:

| Rung | What populates it | σ |
|---|---|---|
| nothing known | the 1500 m/s default | declared large |
| a surface value | `sound_speed_bridge` on the boat | small at the face, unbounded below |
| one cast | the Appledore profiles; Sound Speed Manager for ingest and QC | grows with distance and time from the cast |
| a field | interpolated casts; climatology where it exists | per cell |
| refined | inversion from crossing lines, [#300](https://github.com/rolker/unh_marine_autonomy/issues/300) | per profile, from misfit |

Every consumer asks one question — value and σ at a position and time — and
gets an honest answer at whatever rung the data supports, without knowing which
rung answered. The consumers: the ray tracer for re-projection and for
the launch-to-incidence mapping below; the error model's sound-speed σ;
absorption for transmission loss, from temperature and salinity; sidescan slant
range; single-beam reduction; simulator replay.

**What exists today.** The ray tracer (`cube_bathymetry/.../ssp_ray_tracer.h`:
`cube::traceRay` over a `SoundSpeedProfilePoint` profile, returning a
`RayTraceResult`; a constant-gradient tracer, the forward model agreed on #300) and the surface
sound-speed bridge. Nothing handles a profile beyond the tracer's own input;
the Appledore casts are the first real profile data. The
[#300](https://github.com/rolker/unh_marine_autonomy/issues/300) epic's
"water-column store" is the sound-speed layer of this theme, recorded there.

**Backing.** Sound Speed Manager (HydrOffice, CCOM/NOAA) for ingest and QC; the
Kammerer / TU Delft lineage #300 cites for inversion; Beaudoin (2004) for the
array-face sound-speed term; the standard absorption formulae for turning
temperature and salinity into loss.

### The angular-response decomposition

The backscatter correction in the chain today is one empirical CSV per sonar,
absolute angle in degrees against dB relative to nadir
([cube#81](https://github.com/rolker/cube_bathymetry/issues/81); a second tier
removes transmission loss with an absorption coefficient,
[cube#87](https://github.com/rolker/cube_bathymetry/issues/87)). That curve
lumps four things: the system beam pattern, transmission loss, the mapping from
launch angle to seabed incidence, and the seafloor's own angular response. Only
the first is a property of the sonar. Transmission loss is a function of
frequency, temperature, salinity and depth. The incidence mapping is the ray
tracer's job. The seafloor term is the sediment, site-specific by definition.

So a lumped curve cannot transfer between water bodies, and parameterizing it
by water properties would not fix that, because the seafloor term is not a water
property — transferring the Massabesic curve to the Shoals would subtract the
lake's sediment response from the ocean's, which is the signal a backscatter
product exists to preserve. **The direction (operator, 2026-09-11, confirmed on
PR #374) is the decomposition**: a system term that travels with the sonar, an absorption term
from the water-body model, an incidence mapping from the ray tracer, and the
seafloor term left in the data as the product. That is what GeoCoder's angular
range analysis does (Fonseca & Calder), and ADR-0007 already points there. It
is also the model for how advanced features are added to this chain generally:
backed by existing work from CCOM and beyond, cited, rather than invented.

## The other branches

**Sidescan.** Consumes `RawSonarImage` from `garmin_sidescan` or
`edgetech_sonar`, plus TF, a timestamped nadir altitude (`sensor_msgs/Range`,
used for the slant-to-ground correction — the live node drops a ping whose
altitude is missing or stale unless configured to assume zero), and optionally
a DEM from the depths store; produces
backscatter mosaic tiles through `marine_sidescan_mosaic` (decode, per-ping
projection with a nadir altitude, tier-1 flat and tier-2 DEM-draped mosaics,
overview pyramids) into the sidescan store
([ADR-0006](decisions/0006-multi-platform-backscatter-store.md)). It shares the
spine with bathymetry and none of the middle. Its live node has no tile
publisher ([#379](https://github.com/rolker/unh_marine_autonomy/issues/379)).
The stage detail is deliberately not written here while the mosaic work is
still moving; the umbrella is
[#171](https://github.com/rolker/unh_marine_autonomy/issues/171).

**Water column.** Consumes `RawSonarImage` — the same entry point as sidescan —
from a multibeam's water-column stream (the M3's IMB, the DeltaT in water-column
mode) or a single/split-beam echosounder. `marine_interfaces` already carries
`AcousticSlice` / `AcousticLayer` for a layer of interest in water-column data,
used in past deployments though not this season on BizzyBoat; nothing in the
current tree produces or consumes them. This branch shares the **bottom
detection** stage with bathymetry (see the stages we rent): on the DeltaT,
wanting water column means detecting the bottom ourselves. No middle is built.

**Single and split beam.** Arrives as `RawSonarImage` with one beam;
`rqt_sonar_waterfall` carries a single-beam extractor for display. Reduction to
a sounding is the water-body model's consumer list, not a built stage.

## Known defects in the chain

Recorded here because a chain document that hides them is worse than none.
State as of 2026-09-11.

| Where | Defect | Direction | Status |
|---|---|---|---|
| Stage 3 | Beamwidth fallback used a degrees field as radians | 57× too large | **fixed**, cube PR#153 ([cube#144](https://github.com/rolker/cube_bathymetry/issues/144)) |
| Stage 3 | Per-beam branch converted a radians field as degrees | 57× too small | **fixed**, cube PR#153 |
| Stage 3 | A zero-filled beamwidth array was accepted as a measurement | angular term vanished | **fixed** at the consumer, cube PR#153; `norbit_driver` still publishes zeros |
| Stage 3 | `Platform` roll/pitch fed radians, read as degrees; pitch sign inverted | attitude switched off | **fixed**, cube PR#153 ([cube#147](https://github.com/rolker/cube_bathymetry/issues/147)) |
| Stage 3 | A doc comment claimed a 95 % doubling the code did not do | reader misled | **fixed**, cube PR#153 |
| Stage 3 | `static_roll` bounds in degrees against a beam angle in radians | gate misplaced (latent at 0) | open, [cube#150](https://github.com/rolker/cube_bathymetry/issues/150) |
| Stage 3 | Angular term omits Calder's device-dependent beam-footprint widening | no angle dependence | open, [cube#148](https://github.com/rolker/cube_bathymetry/issues/148) |
| Stage 3 | Detection flags never read | an invalid beam becomes seafloor | open, [cube#154](https://github.com/rolker/cube_bathymetry/issues/154) |
| Stage 3 | `ellipsoidal_referenced` removes only tide terms | draft/heave over-charged, GNSS term absent | open, [cube#156](https://github.com/rolker/cube_bathymetry/issues/156) |
| Stage 3 | No attitude-rate × latency coupling | turn and seaway error charged to nothing | open, [cube#155](https://github.com/rolker/cube_bathymetry/issues/155) |
| Stage 3 | No `Vessel`/`Device` configuration path; offline speed is NaN → 0 without an odometry topic | archive cannot be reprocessed with real geometry; latency budget silently zero | open, [cube#145](https://github.com/rolker/cube_bathymetry/issues/145) |
| Stage 4 | Five copies of the world lift, four in one package | drift, and it has already happened | open, [cube#146](https://github.com/rolker/cube_bathymetry/issues/146) |
| Stage 5 | A second, Cartesian grid with its own insert loop; a third in the explorer | drift | open, [cube#129](https://github.com/rolker/cube_bathymetry/issues/129) |
| Stage 3 | `bw/12` applied to every beam with no detection-method input | Calder switches per device | recorded divergence, no issue |
| Stage 1 | M3 publishes no beamwidths | device fallback on every ping | open; fix on a branch, not yet published, [marine_tools#82](https://github.com/rolker/marine_tools/issues/82) |
| Stage 1 | DeltaT publishes invented variances, `i` not `intensity`, no beam angle | radius pinned; no backscatter | open, [imagenex_deltat#2](https://github.com/rolker/imagenex_deltat/issues/2) |
| Stage 1 | Variance fields named "uncertainty" / "error" | producers write σ | open, [cube#158](https://github.com/rolker/cube_bathymetry/issues/158) |
| Stage 1 | `SonarInfo` does not say which angle its curve is built against | a ray-traced sonar's curve cannot be reused | open, [#378](https://github.com/rolker/unh_marine_autonomy/issues/378) |
| Display | rviz fan uses the full beamwidth as a half-angle | fans 2× too wide | open, [rviz_sonar_image#9](https://github.com/rolker/rviz_sonar_image/issues/9) |

### The beamwidth units defect, and what fixed it

The angular error term read beamwidth in the wrong units in **both** of its
branches: the fallback used `Device`'s degrees unconverted, and the per-beam
branch multiplied a radians field by π/180. The proof that the fallback was a
bug and not a convention was in the same file — the constructor converted the
along-track beamwidth exactly as Calder's `DEG2RAD` does, while the
across-track path did not. Converting the fallback "the way the per-beam branch
does" would have locked in the 57×-too-small error, the more dangerous
direction, because it makes the estimator over-trust every sounding.

The fix (cube PR#153, 2026-09-10) **normalises units once at the boundary and
validates**: a per-beam width that is non-finite, non-positive or ≥ π is not a
measurement and falls back to the device figure — which is what defeats the
zero-filled array. The same PR fixed the `Platform` attitude units and the pitch
sign, fixed the port's Eqn. 3.49 term (a porting error — Calder's C was right),
and added diagnostics counting the beams that used the fallback. The stored tiles written before it are not invalidated:
the operator's decision is to rebuild every store after the pipeline changes
are done, not before.

### What the port's divergence record says

`cube_bathymetry/docs/divergences_from_calder.md` is the term-by-term record.
Two of its statements were corrected during this page's review and are now
right there: the fallback is the *only* path for M3 data, not the rare one; and
the `/12` divisor matches Calder — what did not match was the conversion and
the widening around it. The widening is device-dependent in Calder, not
universal, and fixing the units did not make the port device-aware
([cube#148](https://github.com/rolker/cube_bathymetry/issues/148)). §1 of that
record describes the tide half of RTK referencing as the whole story; it is
not ([cube#156](https://github.com/rolker/cube_bathymetry/issues/156)). And one
divergence it records is still live and has no defect-table row: the port
applies `bw/12` to every beam with no detection-method input, where Calder
switches per device.

## What this revision corrected about the page itself

Recorded so the next reader knows which claims were re-verified.

- The DeltaT driver was described as publishing positions only, every ping
  rejected. It publishes both variance fields, invented; the pings are accepted.
- The page recommended porting Calder's IHO f(z) model for a positions-only
  class that has no members. The recommendation is withdrawn.
- The ray tracer was described as consumed by the inversion work. It has no
  consumer.
- "Full −3 dB" was asserted without a source. The source is now given.
- The defect table said none were fixed as of 2026-09-10; five were fixed that
  afternoon.
- The explorer's uncertainty stand-in and its georeferencing helper were
  described from a branch in review; both have since merged and are described
  as shipped.

## Related

- [ADR-0002](decisions/0002-bathymetric-data-store.md) — bathymetric data store
- [ADR-0005](decisions/0005-multi-platform-provenance-registry.md) — cross-store provenance
- [ADR-0006](decisions/0006-multi-platform-backscatter-store.md) — sidescan backscatter store
- [ADR-0007](decisions/0007-mbes-backscatter-store.md) — MBES backscatter store
- [ADR-0008](decisions/0008-live-sonar-coverage-transport-and-render.md) — live coverage transport
- [ADR-0009](decisions/0009-sonar-info-message.md) — sonar info message
- [ADR-0010](decisions/0010-geospatial-world-model.md) — geospatial world model
- [ADR-0011](decisions/0011-overview-pyramid.md) — overview pyramid
- [#300](https://github.com/rolker/unh_marine_autonomy/issues/300) — sound-speed inversion epic; its store is the `water/` theme's sound-speed layer
- [#380](https://github.com/rolker/unh_marine_autonomy/issues/380) — the marine-messages update path
- `cube_bathymetry/docs/divergences_from_calder.md` — the term-by-term port comparison
