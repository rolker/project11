# Sonar Processing Chain

How a sonar ping becomes a stored, shareable depth surface: the stages, which
component owns each, and what each stage must be fed.

This is the third of three sonar pages, and the one to read when you are about
to write code that touches soundings:

- [sonar_ecosystem.md](sonar_ecosystem.md) — the big-picture *status* map: what
  is built, what is next, where to direct effort.
- [sonar_reference.md](sonar_reference.md) — durable *hardware and protocol*
  facts: sensor identities, wire formats, where the data of record lives.
- **this page** — the *processing chain*: stages, ownership, inputs, units.

It records what the chain is and what it needs. It decides nothing; where a
decision governs a stage it is cited, not restated.

**What "is" means here.** This page describes what ships on the default
branches. Where a stage is being changed on an open branch, that is marked
explicitly rather than written in the present tense, because a chain document
that describes unmerged work is a chain document that lies to whoever reads it
next. Everything below was read out of source on 2026-09-10; where code and its
own comments disagree, the disagreement is recorded rather than smoothed over.

## The chain at a glance

| # | Stage | Owner | In | Out |
|---|---|---|---|---|
| 1 | Acquisition | `marine_tools/kongsberg_em_bridge` | EM datagrams | `SonarDetections` + latched `SonarInfo` |
| 2 | Projection | `cube::DetectionsProjector` | ping + TF + SOG | **sonar-frame** soundings |
| 3 | Uncertainty | `cube::ErrorModel` | ping + `Platform` + `Vessel` + `Device` | per-sounding TPU |
| 4 | Georeferencing | **no single owner — five copies** | soundings + TF | world / geographic soundings |
| 5 | Estimation | `cube::GeoMapSheet` / `cube::Node` | geo soundings + `cube::Parameters` | per-node depth, uncertainty, backscatter |
| 6 | Store write | `cube::store_import` | nodes | GGGS tiles in `marine_bathymetry_store` |

Stages 2 and 3 are **one component**: `DetectionsProjector` exists so that
projection and the error model happen together and once, and its header forbids
node-bound includes so bag replay can reuse it. Stage 4 is **not** in it, and
that is the chain's weakest joint. See stage 4.

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
| `cube::Device` beamwidths | **degrees** | |
| `cube::Platform` roll, pitch | degrees | roll positive port-side-up, pitch positive bow-up |
| `cube::Vessel` alignments, angular sdevs, static roll | degrees | |
| `cube::Platform.heave` | metres | **positive down** |
| `cube::Sounding.depth` | metres | **elevation, positive up** |
| `SspRayTracer` depths | metres | **positive down**, below a shared surface datum |
| `cube::Sounding.vertical_error`, `horizontal_error` | m², **variances** | named "error", hold variances |

Angles on the wire are radians; angles in the `Vessel` and `Device`
configuration are degrees. That split is the single most common source of
defects in this chain, and every one of the known defects below is an instance
of it.

Two further traps. `Sounding::depth` and the ray tracer's
`depth_below_surface` have **opposite signs**; the ray tracer names its field
the long way round for exactly that reason. And the `_error` fields hold
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

**Other drivers do not behave alike**, which matters because the consumer
branches on whether the array is empty:

| Driver | Beamwidths | Consequence today |
|---|---|---|
| `garmin_sidescan` | populated, radians, from a per-generation table; left empty where uncharacterised | takes the per-beam branch, so it is already exposed to that branch's defect |
| `kongsberg_em_bridge` (M3) | empty, deliberately | takes the fallback branch |
| `imagenex_deltat` | absent | takes the fallback branch |
| `r2sonic` | assigned | takes the per-beam branch |
| `norbit_driver` | `resize(num_beams)` with the assignment commented out, marked "not reported" | **an array of zeros**, which is non-empty, so it takes the per-beam branch with a beamwidth of zero |

The Garmin driver is the pattern the other two should copy: full −3 dB widths in
radians from a cited table, and **empty rather than guessed** where a model is
not characterised.

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

**A third geometry exists and is not this one.** `cube::SspRayTracer` is a
constant-gradient ray tracer, the shared forward model agreed on
[#300](https://github.com/rolker/unh_marine_autonomy/issues/300). It is not in
the production path; it is consumed by the sound-speed inversion work and is the
intended basis for re-projection through a real profile
([marine_perception_tools#28](https://github.com/rolker/marine_perception_tools/issues/28)).
Straight-line projection is what ships.

## Stage 3 — Uncertainty

**What it does.** Attaches a per-sounding vertical and horizontal error budget.
This decides how much CUBE trusts each sounding, and it is the least validated
stage in the chain.

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

**Neither `Vessel` nor `Device` has an offline configuration path.** The offline
tools set only the frames and the range gate and leave both structs at their
defaults — zero lever arms, nominal standard deviations, 2° beamwidths — so an
archive cannot be reprocessed with the real geometry. That is
[cube#145](https://github.com/rolker/cube_bathymetry/issues/145).

**What the survey explorer runs instead.** On the default branch the explorer
does **not** use this model at all. `cube_lab.cpp` computes a depth-only
placeholder, `v_std = 0.1 + 0.007·d` and `h_std = 0.2 + 0.01·d`, squares them,
and hands those to CUBE. There is no angle term, no attitude, no lever arms.
An angle-aware replacement is in review on
[marine_perception_tools#50](https://github.com/rolker/marine_perception_tools/pull/50);
it is still a stand-in seeded from `cube::Device`'s constants, and
`cube_lab.hpp` says it lasts "until detections are carried through". The
explorer's bag reader holds the ping and the transform buffer at the same
moment, so carrying detections through to `DetectionsProjector` is practical
rather than aspirational.

**Validation status.** The estimator core was compared term by term against
Brian Calder's original C, which ships in this repo under `original_cube/`, in
[cube#30](https://github.com/rolker/cube_bathymetry/issues/30). The verdict
there is worth repeating: the **estimator is a faithful port** — feed it the
same soundings with the same uncertainty and the grid matches Calder. The
divergences are concentrated in this stage, the upstream error budget.

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
package, so this is not even a cross-repo problem — it is the same package
solving the same problem four times.

**What it must be fed.** A TF buffer covering the ping stamp, containing the
`earth` frame. Lookups fall back to the latest available transform when TF is
momentarily behind, which is normal under bag replay; attitude and heave vary
slowly enough that tens of milliseconds of staleness is harmless. A ping with no
`earth` transform is dropped, and on the live node the grid simply does not
update.

**A recent instance of the cost.** The explorer's copy was itself written twice,
in two files, field by field. Both copies dropped the beam angle and slant
range, which left every bag-loaded sounding unable to reach a stage 3 that reads
them — filed as
[marine_perception_tools#49](https://github.com/rolker/marine_perception_tools/issues/49)
and fixed on the branch in review, where the two copies become one helper with
copy-then-overwrite semantics so a field added later rides along instead of
being dropped.

## Stage 5 — Estimation

**What it does.** Runs CUBE: soundings are spread over an influence radius
derived from their horizontal error, and each node maintains competing depth
hypotheses with Kalman updates, monitoring and intervention.

**Owner.** `cube::GeoMapSheet` over `cube::Node`, tuned by `cube::Parameters`.
The live node accumulates into a geographic map sheet so that persistence needs
no lossy Cartesian-to-geographic step.

**What it must be fed.** Geographic soundings carrying depth, vertical error and
horizontal error, plus intensity, beam angle and slant range when the
backscatter product is wanted
([ADR-0007](decisions/0007-mbes-backscatter-store.md)). The horizontal error
sets the influence radius, so an under-stated horizontal budget pins every
radius to the cell size.

**Implemented more than once.** `marine_perception_tools/src/cube_lab.cpp`
drives `cube::Node` directly rather than going through the grid, mirroring the
grid's insert effect square. It does so for a stated reason: the grid's value
extraction is depth-only, while the node extraction carries the CUBE-settled
backscatter the lab drapes with. That is the most defensible of the
duplications, but it is a copy of the insert loop and will drift.

## Stage 6 — Store write

**What it does.** Turns estimated nodes into GGGS tiles on disk.

**Owner.** `cube::store_import`, writing through `marine_bathymetry_store` and
`marine_mbes_backscatter_store`. The live node persists into the `draft/` layer
directly, with a flush-and-close-checked write that is explicitly **not
crash-atomic**.

**What it must be fed.** Estimated nodes plus the store layout rules from
[ADR-0002](decisions/0002-bathymetric-data-store.md),
[ADR-0010](decisions/0010-geospatial-world-model.md) and
[ADR-0011](decisions/0011-overview-pyramid.md). Layer naming is a live trap: the
MBES backscatter store was collapsed to a single `survey` layer by ADR-0007 A.2,
and imagery layer names are not renamed to match the depth theme.

**A different product, not a duplicate stage.** The explorer exports a single
GeoTIFF for sharing through `cube_export`, rather than tiles. Two places
nonetheless know how to turn nodes into rasters.

## Known defects in the chain

Recorded here because a chain document that hides them is worse than none. None
are fixed as of 2026-09-10.

| Where | Defect | Direction | Issue |
|---|---|---|---|
| Stage 3 | Beamwidth fallback uses a degrees field as radians | 57× too large | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | Per-beam branch converts a radians field as degrees | 57× too small | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | Angular term omits Calder's beam-footprint widening | no angle dependence | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | A zero-filled beamwidth array is accepted as a measurement | angular term vanishes | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | A doc comment claims a doubling the code does not do | reader misled | [cube#144](https://github.com/rolker/cube_bathymetry/issues/144) |
| Stage 3 | No offline `Vessel` or `Device` configuration | archive cannot be reprocessed | [cube#145](https://github.com/rolker/cube_bathymetry/issues/145) |
| Stage 4 | Five copies of the world lift, four in one package | drift, and it has already happened | [cube#146](https://github.com/rolker/cube_bathymetry/issues/146) |
| Stage 1 | M3 and DeltaT publish no beamwidths | works around the above | [marine_tools#82](https://github.com/rolker/marine_tools/issues/82) |

### The beamwidth units defect, in full

The angular error term reads beamwidth in the wrong units in **both** of its
branches. The proof that the fallback is a bug and not a convention is in the
same file: the `ErrorModel` constructor converts the along-track beamwidth with
`* M_PI / 180.0`, exactly as Calder's original does with its `DEG2RAD` macro,
while `swath_angle_error` uses the across-track one unconverted. Same struct,
same documented units, two behaviours.

The per-beam branch has the opposite problem. It multiplies
`ping_info.rx_beamwidths[i]` by `M_PI / 180.0`, but that field is radians. The
installed message definition says radians, `cube::Ping`'s own doc comment says
radians, the bizzyboat retrofit script writes radians, and
`marine_sidescan_mosaic` consumes radians. The error model is the only dissenter.

This changes the fix. Converting the fallback "the way the per-beam branch does"
would lock in the 57×-too-small error and make it the live branch the moment the
drivers start publishing. Too small is the more dangerous direction: it makes
the estimator over-trust every sounding. Normalise the units once at the
boundary instead, so a field documented in degrees cannot reach the formula
unconverted.

**Normalising is not enough on its own.** The branch is selected on array
length, so the Norbit driver's zero-filled array is accepted as a measurement of
zero and removes the angular term entirely. The boundary needs to validate, not
just convert: a non-finite or non-positive beamwidth is not a measurement and
should fall back rather than be believed.

### What the port's divergence record says, and where it needs correcting

`cube_bathymetry/docs/divergences_from_calder.md` records the
degrees-versus-radians inconsistency and leaves two questions open. Both can now
be closed:

- It says "the live path is the normal one (real pings carry `rx_beamwidths`);
  the fallback only fires when the message omits them." That is **backwards for
  the sonar the fleet surveys with**. The M3 driver leaves the arrays empty
  deliberately, so for M3 data the fallback is not the rare path — it is the
  only path, on every ping.
- It leaves open whether the divisor should be `12` or the textbook
  `sqrt(12)`. Calder's device code settles it: for the flat-plate and FFT
  beamformer devices the angular sigma is `bw / 12.0`, where `bw` is already
  `DEG2RAD(across_width) / cos(angle)`. The divisor **does** match Calder. What
  does not match is the conversion to radians and the widening around it.

The widening is device-dependent in Calder, not universal: several device
families use a bare `DEG2RAD(across_width)` with no `1 / cos(angle)` term. The
note is also right about something worth keeping: Calder's angular error is
**per device**, a switch over sonar models with different detection modes, while
the port has one generic formula. Fixing the units does not make the port
device-aware and should not be described as if it did.

### The `horizontal_error` comment is wrong

`error_model.h` says the horizontal positioning term returns "twice the nominal
variance in order to approximate the 95% conf. interval". **The code does not do
this**, and neither does Calder's, whose corresponding function returns a plain
sum. `swath_horizontal` sums four variances with no factor anywhere.
`horizontal_error` is a clean variance in m². The comment is inherited text that
was never true of this port; it should be deleted rather than trusted.

## Related

- [ADR-0002](decisions/0002-bathymetric-data-store.md) — bathymetric data store
- [ADR-0007](decisions/0007-mbes-backscatter-store.md) — MBES backscatter store
- [ADR-0009](decisions/0009-sonar-info-message.md) — sonar info message
- [ADR-0010](decisions/0010-geospatial-world-model.md) — geospatial world model
- [ADR-0011](decisions/0011-overview-pyramid.md) — overview pyramid
- `cube_bathymetry/docs/divergences_from_calder.md` — the term-by-term port comparison
