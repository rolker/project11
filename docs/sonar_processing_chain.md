# Sonar Processing Chain

How a sonar ping becomes a stored, shareable depth surface: the stages, the one
component that owns each, and what each stage must be fed.

This is the third of three sonar pages, and the one to read when you are about
to write code that touches soundings:

- [sonar_ecosystem.md](sonar_ecosystem.md) — the big-picture *status* map: what
  is built, what is next, where to direct effort.
- [sonar_reference.md](sonar_reference.md) — durable *hardware and protocol*
  facts: sensor identities, wire formats, where the data of record lives.
- **this page** — the *processing chain*: stages, ownership, inputs, units.

It records what the chain is and what it needs. It decides nothing; where a
decision governs a stage it is cited, not restated. Every claim here was read
out of the source on 2026-09-10; where the code and its own documentation
disagree, that disagreement is recorded as a defect rather than smoothed over.

## The chain at a glance

| # | Stage | Owner | In | Out |
|---|---|---|---|---|
| 1 | Acquisition | `marine_tools/kongsberg_em_bridge` | EM datagrams | `SonarDetections` + latched `SonarInfo` |
| 2 | Projection | `cube::DetectionsProjector` | ping + TF + SOG | sonar-frame soundings |
| 3 | Uncertainty | `cube::ErrorModel` | ping + `Platform` + `Vessel` + `Device` | per-sounding TPU |
| 4 | Georeferencing | `cube::DetectionsProjector` (live), `tf_lift.hpp` (explorer) | soundings + TF | world-frame soundings |
| 5 | Estimation | `cube::GeoMapSheet` / `cube::Node` | world soundings + `cube::Parameters` | per-node depth, uncertainty, backscatter |
| 6 | Store write | `cube::store_import` | nodes | GGGS tiles in `marine_bathymetry_store` |

Stages 2, 3 and 4 are **one component**. `DetectionsProjector` exists precisely
so that projection, the error model and the TF lookup happen together and once.
Its header forbids node-bound includes so bag replay can reuse it, and both the
live node and `bag_to_geotiff` are thin adapters over it.

## Units and conventions

Get these wrong and the numbers stay plausible while being wrong, which is how
the defects below survived. This table is the contract.

| Quantity | Unit | Convention |
|---|---|---|
| `SonarDetections.tx_angles` | radians | positive forward |
| `SonarDetections.rx_angles` | radians | positive to starboard |
| `SonarDetections.two_way_travel_times` | seconds | |
| `PingInfo.tx_beamwidths`, `rx_beamwidths` | **radians** | full −3 dB; may be empty |
| `PingInfo.frequency`, `sound_speed` | Hz, m/s | **0 means unavailable**, not zero |
| `cube::Device` beamwidth fields | **degrees** | the one place degrees are used |
| `cube::Platform` roll, pitch | degrees | roll positive port-side-up, pitch positive bow-up |
| `cube::Platform.heave` | metres | **positive down** |
| `cube::Sounding.depth` | metres | **elevation, positive up** |
| `SspRayTracer` depths | metres | **positive down**, below a shared surface datum |
| `cube::Sounding.vertical_error` | m², a **variance** | named "error", holds a variance |
| `cube::Sounding.horizontal_error` | m², **not a clean variance** | one term is doubled to approximate a 95% bound |

Two of these are traps worth naming out loud. `Sounding::depth` and the ray
tracer's `depth_below_surface` have **opposite signs**; the ray tracer names its
field the long way round for exactly that reason. And the `_error` fields hold
variances, so a consumer that squares them is wrong by a square.

## Stage 1 — Acquisition

**What it does.** Decodes the M3's EM datagram export into one
`SonarDetections` per ping, plus a latched `SonarInfo` companion
([ADR-0009](decisions/0009-sonar-info-message.md)) carrying what the intensities
actually are and what corrections have been applied.

**Owner.** `marine_tools/kongsberg_em_bridge`. The populated soundings datagram
on the M3 is Raw Range and Angle 78; XYZ88 is exported but empty. See
[sonar_reference.md](sonar_reference.md) for why the route is the sonar's own
export rather than QINSy.

**What it must be fed.** The datagram stream, and a frame id for the header. It
fills `frequency` from the first sector's centre frequency, `sound_speed` from
the datagram, per-beam travel times, tx delays, intensities and steering angles,
and a detection flag per beam.

**What it deliberately does not fill.** `tx_beamwidths` and `rx_beamwidths` are
left empty **on purpose**, with a comment saying so: the error model treats
those arrays as degrees while the message says radians, so leaving them empty
avoids the mismatch. This is a workaround for a downstream defect, not a driver
gap — which is why
[marine_tools#82](https://github.com/rolker/marine_tools/issues/82) is
explicitly sequenced *after* the consumer unit fix. Do not "fix" the driver
first: publishing correct radians into the current consumer makes the error
smaller-but-wrong in the more dangerous direction.

## Stage 2 — Projection

**What it does.** Turns each beam's travel time and steering angles into a point
in the sonar frame.

**Owner.** `cube::DetectionsProjector`, which delegates the per-beam arithmetic
to the `cube::Sounding` detections constructor.

**What it must be fed.** `two_way_travel_times`, `tx_angles`, `rx_angles` and
`ping_info.sound_speed`. Range is `twtt * sound_speed / 2`, so a ping whose
`sound_speed` is the "unavailable" 0 projects every beam to the origin. There is
a range gate: a sounding is kept only when its slant range is within
`[minimum_range, maximum_range]`, and the count dropped is reported in the
diagnostics rather than logged by the projector itself.

**Implemented more than once.** `marine_perception_tools/src/mbes_geometry.hpp`
re-implements the same projection for the survey explorer, with the same
formula and a single sound speed. Its header calls itself a QC-grade
projection, which is honest, but it is a second copy of stage 2.

**A third geometry exists and is not this one.**
`cube::SspRayTracer` is a constant-gradient ray tracer, the shared forward model
agreed on [#300](https://github.com/rolker/unh_marine_autonomy/issues/300). It
is not yet in the production path; it is consumed by the sound-speed inversion
work and is the intended basis for re-projection through a real profile
([marine_perception_tools#28](https://github.com/rolker/marine_perception_tools/issues/28)).
Straight-line projection is what ships today.

## Stage 3 — Uncertainty

**What it does.** Attaches a per-sounding vertical and horizontal error budget.
This is the stage that decides how much CUBE trusts each sounding, and it is the
least validated stage in the chain.

**Owner.** `cube::ErrorModel::compute(detections, platform)`, constructed with a
`Vessel` and a `Device`.

**What it must be fed.**

- *Per ping, via `cube::Platform`*: timestamp, roll, pitch, heave, surface sound
  speed, geometric-mean sound speed, and vessel speed over ground. The projector
  draws roll, pitch and heave from TF and takes speed over ground as an
  argument.
- *Per beam, via `SonarDetections`*: travel times, tx and rx steering angles,
  detection flags, and from `ping_info` the sound speed, frequency and
  beamwidths.
- *As configuration, via `Vessel`*: lever arms to the GPS and IMU, alignment and
  latency standard deviations, draft, static roll, and the measurement standard
  deviations for roll, pitch, gyro, sound-speed profile, heave and draft. Also
  `ellipsoidal_referenced`, which when true (the default) omits the tide terms
  from the vertical budget deliberately.
- *As configuration, via `Device`*: across-track and along-track beamwidths in
  degrees, range error as a fraction, and an absolute range-error floor.

**What happens when an input is absent.** A missing attitude transform leaves
roll and pitch NaN, so the resulting uncertainty is NaN, and the projector
reports it in `diagnostics.missing_attitude`. A missing heave transform defaults
heave to zero, which the projector documents as non-critical because heave
enters the budget only squared. Speed over ground may be passed as NaN.

**The `Vessel` half has no offline configuration path.** Offline tools construct
it with defaults, meaning zero lever arms and nominal standard deviations, so an
archive cannot be reprocessed with the real geometry. That is
[cube_bathymetry#145](https://github.com/rolker/cube_bathymetry/issues/145).

**Implemented more than once.**
`marine_perception_tools/src/sounding_uncertainty.hpp` is a stand-in that
propagates a range error and an angular error through each beam's own angle and
slant range, seeded from `cube::Device`'s constants. Its own header says it
lasts "until detections are carried through". It is not a small divergence: it
carries no roll, pitch, heave, sound-speed-profile or lever-arm terms at all,
and it applies the floor to each propagated component rather than to the range.
The explorer's bag reader holds the ping and the transform buffer at the same
moment, so carrying detections through to `DetectionsProjector` is practical
rather than aspirational.

**Validation status.** The estimator core was compared term by term against
Brian Calder's original C, which ships in this repo under `original_cube/`, in
[cube_bathymetry#30](https://github.com/rolker/cube_bathymetry/issues/30). The
verdict there is worth repeating: the **estimator is a faithful port** — feed it
the same soundings with the same uncertainty and the grid matches Calder. The
divergences are concentrated in this stage, the upstream error budget.

## Stage 4 — Georeferencing

**What it does.** Lifts sonar-frame soundings into the world frame.

**Owner.** `DetectionsProjector` on the live path, using the configured frames:
`base_link`, a level north-aligned `base_link_north_up`, and `map_tide`.
Namespaced deployments must override those names.

**What it must be fed.** A TF buffer covering the ping stamp. Lookups fall back
to the latest available transform when TF is momentarily behind, which is normal
under bag replay; attitude and heave vary slowly enough that tens of
milliseconds of staleness is harmless.

**Implemented more than once.** The explorer lifts separately in
`marine_perception_tools/src/tf_lift.hpp`. As of
[marine_perception_tools#42](https://github.com/rolker/marine_perception_tools/issues/42)
that is one shared helper with copy-then-overwrite semantics, so a field added
to the sounding type rides along rather than being silently dropped. It had
previously been two hand-written copies, and both dropped the beam angle and
slant range, which left every bag-loaded sounding unable to reach stage 3 at
all.

## Stage 5 — Estimation

**What it does.** Runs CUBE: soundings are spread over an influence radius
derived from their horizontal error, and each node maintains competing depth
hypotheses with Kalman updates, monitoring and intervention.

**Owner.** `cube::GeoMapSheet` over `cube::Node`, tuned by `cube::Parameters`.
The live node accumulates into a geographic map sheet so that persistence needs
no lossy Cartesian-to-geographic step.

**What it must be fed.** World-frame soundings carrying depth, vertical error
and horizontal error, plus intensity, beam angle and slant range when the
backscatter product is wanted
([ADR-0007](decisions/0007-mbes-backscatter-store.md)). The horizontal error is
what sets the influence radius, so an under-stated horizontal budget pins every
radius to the cell size.

**Implemented more than once.** `marine_perception_tools/src/cube_lab.cpp`
drives `cube::Node` directly rather than going through the grid, mirroring the
grid's insert effect square. It does so for a stated reason: the grid's
value extraction is depth-only, while the node extraction carries the
CUBE-settled backscatter the lab drapes with. That is the most defensible of the
three duplications, but it is a copy of the insert loop and will drift.

## Stage 6 — Store write

**What it does.** Turns estimated nodes into GGGS tiles on disk.

**Owner.** `cube::store_import`, writing through `marine_bathymetry_store` and
`marine_mbes_backscatter_store`. The live node persists into the `draft/` layer
directly, with a flush-and-close-checked write that is explicitly **not
crash-atomic**.

**What it must be fed.** Estimated nodes plus the store layout rules from
[ADR-0002](decisions/0002-bathymetric-data-store.md),
[ADR-0010](decisions/0010-geospatial-world-model.md) and
[ADR-0011](decisions/0011-overview-pyramid.md). Layer naming is a live trap:
the MBES backscatter store was collapsed to a single `survey` layer by
ADR-0007 A.2, and imagery layer names are not renamed to match the depth theme.

**Implemented more than once.** The explorer exports its own GeoTIFF through
`cube_export`. That is a different product — a single file for sharing, not
tiles — so it is not really a duplicate stage, but it does mean two places know
how to turn nodes into rasters.

## Known defects in the chain

Recorded here because a chain document that hides them is worse than none. All
are filed; none are fixed as of 2026-09-10.

| Where | Defect | Direction | Issue |
|---|---|---|---|
| Stage 3 | Beamwidth fallback uses a degrees field as radians | 57× too large | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | Per-beam branch converts a radians field as degrees | 57× too small | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | Angular term omits Calder's beam-footprint widening | no angle dependence at all | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | No offline `Vessel`/`Device` configuration | archive cannot be reprocessed | [cube#145](https://github.com/rolker/cube_bathymetry/issues/145) |
| Stage 1 | Drivers publish no beamwidths | works around the above | [marine_tools#82](https://github.com/rolker/marine_tools/issues/82) |

### The beamwidth units defect, in full

The angular error term reads beamwidth in the wrong units in **both** of its
branches. The proof that the fallback is a bug and not a convention sits two
lines away in the same constructor: the along-track beamwidth is converted with
`* M_PI / 180.0`, exactly as Calder's original does with its `DEG2RAD` macro,
while the across-track fallback is not converted at all.

The per-beam branch has the opposite problem. It multiplies
`ping_info.rx_beamwidths[i]` by `M_PI / 180.0`, but that field is radians. The
message says radians in both the released `marine_msgs` source and the installed
copy; `cube::Ping`'s own doc comment says radians; the bizzyboat retrofit script
writes radians; `marine_sidescan_mosaic` consumes radians. The error model is the
only dissenter in the workspace.

This changes the fix. Converting the fallback "the way the per-beam branch does"
would lock in the 57×-too-small error and make it the live branch the moment the
drivers start publishing. Too small is the more dangerous direction: it makes
the estimator over-trust every sounding. Normalise the units once at the
boundary instead, so a field documented in degrees cannot reach the formula
unconverted.

A third divergence sits alongside them. Calder widens the across-track beamwidth
by `1 / cos(angle)` before forming the angular error, so the term grows toward
the swath edge. The port has no angle dependence in that term at all.

**What the port's own divergence record already says, and where it needs
correcting.** `cube_bathymetry/docs/divergences_from_calder.md` records the
degrees-versus-radians inconsistency between the two branches, and leaves two
questions open. Both can now be closed:

- It says "the live path is the normal one (real pings carry `rx_beamwidths`);
  the fallback only fires when the message omits them." That is **backwards for
  every sonar in service**. The M3 driver leaves the arrays empty deliberately,
  so the fallback is not the rare path — it is the only path, on every ping. The
  severity is correspondingly higher than that note implies.
- It leaves open whether the divisor should be `12` or the textbook
  uniform-distribution `sqrt(12)`. Calder's own device code settles it: in the
  amplitude-detection branch the angular sigma is `bw / 12.0`, where `bw` is
  already `DEG2RAD(across_width) / cos(angle)`. The divisor **does** match
  Calder. What does not match is everything around it: the conversion to
  radians, and the widening by `1 / cos(angle)`.

The note is also right about something worth keeping: Calder's angular error is
**per device**, a switch over sonar models with different detection modes, while
the port has one generic formula. Fixing the units does not make the port
device-aware, and should not be described as if it did.

## Related

- [ADR-0002](decisions/0002-bathymetric-data-store.md) — bathymetric data store
- [ADR-0007](decisions/0007-mbes-backscatter-store.md) — MBES backscatter store
- [ADR-0009](decisions/0009-sonar-info-message.md) — sonar info message
- [ADR-0010](decisions/0010-geospatial-world-model.md) — geospatial world model
- [ADR-0011](decisions/0011-overview-pyramid.md) — overview pyramid
- `cube_bathymetry/docs/divergences_from_calder.md` — the term-by-term port comparison
