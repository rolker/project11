---
issue: 369
---

# Issue #369 — Survey tiles are written at one fixed level regardless of depth — CUBE's capture radius adapts, the grid does not

## Issue Review
**Status**: complete
**When**: 2026-09-09 11:56 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #369
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Actions
- [ ] Pin the forward-only vs. retroactive-reprocess scope split before plan-task: design question 6 ("Existing stores") makes retroactive reprocessing of Shoals/Massabesic contingent on #366 (the import ledger), which is still **OPEN**. Recommend scoping this PR to new imports only and filing retroactive reprocessing as a follow-up gated on #366, unless #366 lands first.
- [ ] Pin the live-vs-offline scope (design question 4) toward the option the issue itself names as best-fitting: `draft` stays fixed-level at write time (matches the existing D8 draft/processed asymmetry — draft is disposable/regenerable, processed is authoritative) and only `processed`/`import_bag` gets depth-adaptive level selection. Adapting the live node raises retiling machinery that isn't otherwise in scope and would make this a much larger change.
- [ ] Record the resulting decision as a `uma-ADR-0010` D9 amendment (the issue itself proposes this under "Where the work lands") — D9 currently states plainly "born at fine native levels," which a depth-adaptive `processed` layer contradicts. Capture-decisions principle: this is a store-contract change, not just an implementation detail.
- [ ] Verify, not just note, the capacity-radius/lattice-spacing question in the issue's "Worth verifying" paragraph: at a 0.9 m lattice the farthest a sounding can sit from its nearest node is 0.64 m, which exceeds the 0.5 m capture floor that applies below 10 m depth. Confirmed by reading `capture_distance_scale` (0.05, `parameters.h:205`) and its use in `node.cpp:154` — the formula in the issue is accurate. Whether shallow soundings are actually dropped depends on the node-selection loop, which the issue explicitly says it did not trace; this should be resolved with a test before or during implementation, not left as a residual note.
- [ ] State the seam behaviour explicitly (design question 3) rather than assuming it. `uma-ADR-0013` D8 already establishes that `shallowestReliable()` scans all layers and levels regardless of what's drawn, so a depth-adaptive `processed`/`draft` layer is safe by the same argument #331 already used for `reference`'s native-wins pyramid — but that argument should be written down for this layer too, not inherited silently.
- [ ] If `processed` becomes a mixed-level, region-disjoint layer (the same shape #331 built for `reference`), it inherits `uma-ADR-0013` D2 (producer-side error nesting) and D3 (a declared coverage manifest) obligations — the writers (`import_bag`, the depth-pyramid builder) need to emit both, following the precedent #331 already set.
- [ ] Design question 5 (bounds) needs a concrete floor, ceiling, and a storage estimate in the plan — "worth naming" in the issue is not sufficient; an unbounded fine level in shallow water is exactly the kind of open-ended default this workspace's principles flag as needing a stated decision, not a default left implicit.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue lays out its reasoning and open questions plainly; no hidden behavior proposed. |
| Enforcement over documentation | Watch | The eventual level-choice policy needs a validated bound (question 5) and a documented seam rule (question 3), not just prose — flag as plan-task scope. |
| Capture decisions, not just implementations | Action needed | The issue itself expects to amend `uma-ADR-0010` D9; that amendment should land as part of this work, not be assumed. |
| A change includes its consequences | Action needed | Existing-store reprocessing (question 6) is explicitly entangled with #366 (open); scope must be pinned so the PR's consequences are bounded and don't silently wait on an unrelated open issue. |
| Only what's needed | OK | Motivated by a measured mismatch (0.9 m cells vs. ~8 mm–12 cm sensor footprint across the surveyed depth range), not speculative. |
| Improve incrementally | Watch | As written (6 open design questions, two writers, a possible ADR amendment, a possible retroactive reprocess) this is large for one PR; the issue's own "draft fixed / processed adapts" and "new imports only" options are the way to keep it incremental. |
| Test what breaks | Action needed | The 0.64 m vs. 0.5 m capture-floor question is a concrete correctness risk (possible silently-uncaptured shallow soundings) and needs a regression test, not just a documentation note. |
| Workspace vs. project separation | OK | Correctly filed in `unh_marine_autonomy` (project repo); this is domain content (bathymetry store policy), not workspace infra. |
| Workspace improvements cascade to projects | N/A | Not applicable — project-domain change. |
| Primary framework first, portability where free | N/A | Not applicable. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| `uma-ADR-0010` (Geospatial World Model) | Yes | D9 ("LOD is a per-layer process") currently states `draft`/`processed` are "born at fine native levels" — a depth-adaptive level would amend this directly, as the issue itself anticipates. |
| `uma-ADR-0013` (Bounded LOD Navigation) | Yes | D2 (producer-side error nesting) and D3 (declared coverage manifest) apply if `processed` becomes mixed-level/region-disjoint, same as #331 did for `reference`. D8 (safety queries never consult an LOD level) already covers the safety argument the issue asks about in question 3 — it should be cited explicitly rather than re-derived. |
| ADR-0001 (Adopt ADRs) | Yes | A store-contract decision of this size needs its rationale captured, per the issue's own "Where the work lands" section. |

### Consequences

- Depends on #366 (import ledger, OPEN) if retroactive reprocessing of existing stores (Shoals, Massabesic) is included in scope — recommend excluding it from this PR and tracking separately.
- Touches both `cube_bathymetry` (writer: `import_bag`, live node) and `unh_marine_autonomy` (store-contract policy, `uma-ADR-0010` D9) — cross-repo consequence, both should land together or be sequenced explicitly.
- If `processed` becomes mixed-level, `docs/sonar_ecosystem.md` and any store-contract documentation referencing "one native level per survey layer" need updating in the same PR.

### Recommendations

- Scope this PR to: (1) the level-choice policy for `processed` only, driven by depth per the issue's own preferred proxy, decided per-tile-or-region per a two-pass `import_bag` scheme; (2) leave `draft` fixed-level, matching the existing D8 draft/processed asymmetry; (3) new imports only, with retroactive reprocessing filed as an explicit follow-up gated on #366.
- Carry the `uma-ADR-0010` D9 amendment in the same PR as the implementation (matches how #331 landed its D9 mixed-level amendment for `reference`) rather than deferring documentation.
- Add a regression test for the capture-floor/lattice-spacing interaction (0.05×depth vs. 0.5 m floor vs. cell diagonal) before relying on the depth-adaptive level to close the gap the issue describes.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Sonnet`
