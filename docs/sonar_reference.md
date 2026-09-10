# Sonar & Survey Data Reference

Durable hardware, protocol, and data-location facts behind the
[sonar ecosystem map](sonar_ecosystem.md). The map tracks *status*, and the
[sonar processing chain](sonar_processing_chain.md) tracks what happens to a
ping and what each stage must be fed; this page
records the facts that don't change with each PR — sensor identities, wire
protocols, and where the data of record actually lives. Sourced from field
sessions (2026 Massabesic campaign) and verified against the drivers where
possible.

## Kongsberg M3 multibeam

**Identity**: the M3 is Kongsberg **Mesotech** lineage — *not* the Kongsberg
EM echosounder line. EM-family assumptions (SIS acquisition, EM installation
conventions) do not apply to the M3 as a product; the naming overlap is only
in the export datagrams (below).

**Specs**: 500 kHz, 256 beams, 120–140° swath, 0.2–150 m range, up to 40 Hz
ping rate.

**Acquisition**: dedicated Windows "M3" application (not SIS 5) running on
mercat (`192.168.20.8`), with the sonar head on an isolated point-to-point
link (head factory IP `192.168.1.234`).

**Interfaces**:

| Channel | Transport | Content |
|---|---|---|
| Control | TCP XML API, port 20001 | Head configuration/control |
| Water column | IMB over TCP | Imagery (water-column) |
| Soundings | `.all` EM datagrams (UDP export) | Bottom detections, 256 pts/ping |
| Time | NMEA ZDA on UDP 31100 @ 1 Hz, plus hardware 1PPS | Head time sync |

The ROS-side consumer is `kongsberg_em_bridge`, which decodes the EM datagram
export — the *datagram format* is EM-standard even though the sonar is not.
Setup gotchas (1PPS dropdown, silent-until-enabled Export Data widget) are in
the BizzyBoat
[hydro payload install log](https://github.com/rolker/unh_echoboats_project11/blob/jazzy/bizzyboat_project11/docs/hydro_payload_install_log.md).

**Backscatter characteristics**: `reflectivity_db` is signed int16 × 0.1 dB
from the "Raw Range and Angle 78" datagram — **raw, not angle-normalized**.
Measured angular falloff is ~24 dB from nadir to the swath edge (~−40 dB at
nadir → ~−64 dB at 58°); a cos²θ Lambert model is roughly 4× too weak to
flatten it, which is why the corrector uses an empirical per-sonar
angular-response curve
([cube#81](https://github.com/rolker/cube_bathymetry/issues/81)).

## Real-time sounding route — why not through QINSy

QINSy's generic UDP output **cannot stream processed soundings**: its
processed product (QPD) is export-only, with no real-time network feed. The
working real-time route is therefore the **M3's own EM datagram UDP export**,
consumed directly by `kongsberg_em_bridge`. On the M3 the populated soundings
datagram is **Raw Range and Angle 78** (`N`); the XYZ88 (`X`) datagram is
exported but **empty** (kept in the decoder for EM2040/future sonars — see
[`kongsberg_em_bridge/em_datagrams.py`](https://github.com/rolker/marine_tools/blob/jazzy/kongsberg_em_bridge/kongsberg_em_bridge/em_datagrams.py)
in `marine_tools`). QINSy remains the hydrographic system
of record on mercat; the ROS side taps the sensor's export in parallel rather
than downstream of QINSy. Recorded here so nobody re-attempts the QINSy-UDP
path.

## Data of record

Canonical locations for the 2026 survey data (gabby/salmon home dirs unless
noted):

| What | Where | Notes |
|---|---|---|
| Tiled stores | `~/data/world/{depths,imagery}/` (bathymetry → `depths/`; backscatter + sidescan → `imagery/`) | Regenerable caches over the bags |
| Survey index | `~/data/world/survey_index.db` | v2 (134 Massabesic bags); dev-tooling metadata, so it sits at the world root rather than in a themed store (#310) |
| Retrofitted-bag originals | `salmon:~/data/retrofit_backups_2026-07-02/` | 10 `.orig` files — the pre-retrofit **data of record** |
| Surface sound speed | `~/surface_sound_speed/` | 1 Hz deliverable; gap-filled freshwater Marczak (1997), global offset ≈ +1.07 m/s |

Raw deployment bags are the ultimate source of truth; stores are regenerable
caches (see the store-redesign discussion in
[cube#96](https://github.com/rolker/cube_bathymetry/issues/96)).

**Store layout constraints**:

- Bathymetry and backscatter stores **cannot share a directory** — both use a
  `processed/` layer with different dtypes (Float64 vs Float32).
- NoData conventions: chart-bathy and backscatter tiles use **NaN**; sidescan
  uses **0**; the Massabesic source chart raster (`massabesic_bathy.tif`)
  uses **−9999**, converted to NaN on import.

**Sound-speed data caveats**: the AML probe
(`/bizzy/sensors/sound_speed/sound_speed`, ~20 Hz) fails via a
healthy → NaN → silence signature; heavy outages Jun 17 – Jul 1 mean some
bags carry zero sound-speed messages. The Garmin
`.../garmin_sidescan/water_temperature` topic exists in bags only from
**2026-06-15** onward.
