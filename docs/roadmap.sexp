; The fixed3d roadmap: the data ROADMAP.md is generated from.
; Same shape as nova-sprint's docs/roadmap.sexp (internal/roadmapdoc's package comment):
; data, never evaluated. Edit this file, then regenerate ROADMAP.md with nova-sprint's
; generator, from a nova-sprint checkout:
;   go build -o /tmp/roadmap ./tools/roadmap && (cd <this repo> && /tmp/roadmap --file docs/roadmap.sexp --out ROADMAP.md)
; Never edit ROADMAP.md by hand. Created 2026-10-14 when the open sprint cards of this
; repository were moved out of the stopped sprint.
(roadmap "v1"
 :title "fixed3d roadmap"
 :text "Planned work for fixed3d. Each item below was an open sprint card, moved out of the stopped sprint on 2026-10-14; it names its card, its tier, its base branch and any pushed work to resume from. Landed work is in git."

 :groups
 (
  (group "github-issues" :title "Moved from GitHub issues on 2026-10-14"
   :text "Open GitHub issues of mas-bandwidth/fixed3d, recorded here on 2026-10-14 when roadmaps became the standard for future work (Glenn, 2026-10-14). Each item names its issue; the issue is closed with a pointer here.")

  (group "export-fixed3d-box3d-ports" :title "Moved out of the sprint on 2026-10-14: fixed3d-box3d-ports"
   :text "The open cards of sprint stream fixed3d-box3d-ports, moved out of the stopped sprint on 2026-10-14, one item per card. Landed cards of the stream are in git and are not listed. Each item names its card, its tier, its base branch and any pushed work to resume from.")
  (group "export-fixed3d-perf" :title "Moved out of the sprint on 2026-10-14: fixed3d-perf"
   :text "The open cards of sprint stream fixed3d-perf, moved out of the stopped sprint on 2026-10-14, one item per card. Landed cards of the stream are in git and are not listed. Each item names its card, its tier, its base branch and any pushed work to resume from.")
  (group "export-fixed3d-arch-opt" :title "Moved out of the sprint on 2026-10-15: fixed3d-arch-opt"
   :text "The open cards of sprint stream fixed3d-arch-opt, moved out of the sprint on 2026-10-15 when Glenn scoped the sprint down to tools/sprint, rocketnet and schema (2026-10-15), one item per card. Landed cards of the stream are in git and are not listed. Each item names its card, its tier, its base branch and any pushed work to resume from.")
 )

 :items
 (
  (item "issue-20" :group "github-issues" :area "fixed3d"
   :title "Port the box3d follow-cam / world-text sample viewer (upstream 3fc20f5), held for a bench with a display"
   :text "Upstream 3fc20f5 'Follow cam (#107)' (2026-07-29) carries a sample-viewer feature that the 2026-08-22 port pass deliberately did not take across. The engine, CI and constants half of that commit is ported (see the port commit on the port branch), and the cursor moves past it with this issue as the named exception, so nothing is hidden."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #20, opened 2026-08-22 by rowan-claude; moved here 2026-10-14"
   :kept (:github-issue "20" :labels ""))
  (item "issue-25" :group "github-issues" :area "fixed3d"
   :title "Baked-geometry version constants do not distinguish LUDICROUS_MODE, so a cross-config blob passes the guard and is read 48 bytes short"
   :text "The three baked-geometry version constants — B3_HULL_VERSION, B3_MESH_VERSION and B3_HEIGHT_FIELD_VERSION — are plain #defines in a public header and take the same value in both build configurations. The layouts they exist to certify do not."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #25, opened 2026-08-23 by rowan-claude; moved here 2026-10-14"
   :kept (:github-issue "25" :labels ""))
  (item "issue-27" :group "github-issues" :area "fixed3d"
   :title "Move the fixed-point core to the fixed library; fixed3d keeps b3-named wrappers"
   :text "Glenn's call, 2026-08-24, from one function outward to the class: 'isn't this a class of functions that should be moved out to 'fixed' library? Surely converting a fixed point to double is fixed library functionality, not fixed3d (although fixed3d can totally export it, that's fine).'"
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #27, opened 2026-08-23 by gafferongames; moved here 2026-10-14"
   :kept (:github-issue "27" :labels ""))
  (item "issue-31" :group "github-issues" :area "fixed3d"
   :title "Precision-envelope audit: find every magnitude-dependent failure, make the arithmetic loud"
   :text "Glenn's question (2026-08-24), after invMass (#29) and the inertia inversion (#30) both proved broken at scale: are there OTHER parts needing expanded precision? The class signature: inverses of large quantities (underflow to zero or garbage), products of large quantities in reductions (overflow, wrapped casts), silent truncation at int64 casts, and ..."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #31, opened 2026-08-24 by gafferongames; moved here 2026-10-14"
   :kept (:github-issue "31" :labels ""))
  (item "issue-32" :group "github-issues" :area "fixed3d"
   :title "Centre of mass multiplies by the truncated reciprocal, so it collapses to the body origin for heavy bodies"
   :text "Found while fixing #29, deliberately NOT bundled with it because the fix perturbs every body and trips an assert that needs its own investigation. b3UpdateBodyMassData (src/body.c) accumulates localCenter = sum( mass_i * center_i ) and then divides by the total mass by multiplying by the reciprocal:"
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #32, opened 2026-08-24 by gafferongames; moved here 2026-10-14"
   :kept (:github-issue "32" :labels ""))
  (item "issue-41" :group "github-issues" :area "fixed3d"
   :title "Angular damping has a resolution floor: a slow-spinning large body never damps at all"
   :text "b3IntegrateVelocitiesTask (src/solver.c) damps by multiplying: b3FixMul rounds half up. So a Q48.16 angular velocity component only moves when the per-substep decrement clears half a quantum: Below that, every component is a fixed point of the multiply and the spin holds forever. Not 'damps slowly' — does not change at all, for any number of steps."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #41, opened 2026-08-24 by gafferongames; moved here 2026-10-14"
   :kept (:github-issue "41" :labels ""))
  (item "issue-47" :group "github-issues" :area "fixed3d"
   :title "Port the samples half of box3d 47d7f7c (character mover, issues, joint)"
   :text "Upstream 47d7f7c 'clockwise option, joint is awake (#130)' carries a samples half that PR #46 deliberately does not take: samples/sample_character.cpp (225 lines) — mover behavior against the new b3Body_TimeOfImpactMover and the new b3PlaneResult indices samples/sample_issues.cpp (84 lines) samples/sample_joint.cpp (65 lines) — exercises b3Joint_IsAwake ..."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #47, opened 2026-08-30 by rowan-claude; moved here 2026-10-14"
   :kept (:github-issue "47" :labels ""))
  (item "issue-50" :group "github-issues" :area "fixed3d"
   :title "Height field shape-cast legs changed proxy order and shift origin for clockwise fields, recorded as a pure refactor"
   :text "Issue #50 asked for a measurement rather than an argument; it was taken in 7ae09e6 on sprint/fixed3d-box3d-ports (card fixed3d-issue-50). Pre-change bddc87f against post-change 135e64a, 1000 seeded b3ShapeCastHeightField casts per proxy: the counter-clockwise control is bit-identical; on a clockwise field 1738 of 3000 casts differ, every difference a hit-to-miss flip from the new back-side culling, and the post-change build records zero hits on any clockwise field from an above-ground query (flat field included). Casts both builds hit are bit-identical. What remains is the decision the measurement recorded as Glenn's: keep upstream's culling for clockwise height fields, or restore the old spelling, then pin the chosen behaviour with a test."
   :why "Triage 2026-10-15: the measurement half is done; the keep-or-restore decision is open, so the item stays."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #50, opened 2026-09-01 by rowan-claude; moved here 2026-10-14"
   :kept (:github-issue "50" :labels ""))
  (item "issue-51" :group "github-issues" :area "fixed3d"
   :title "Mover mesh collide feeds the winding-swapped vertex order to GJK, not just to the cull"
   :text "b3CollideMoverAndMesh now swaps vertex2 and vertex3 when the mesh scale is not counter clockwise, and the swap happens before the vertices are used for anything. The swapped order reaches triangleVertices, which is handed to GJK as proxyA, not only to the new signed volume cull that actually needs the orientation."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #51, opened 2026-09-01 by rowan-claude; moved here 2026-10-14"
   :kept (:github-issue "51" :labels ""))
  (item "issue-53" :group "github-issues" :area "fixed3d"
   :title "Height field cull spells triangle 2 as two different cyclic rotations across its two sites"
   :text "The height field spells the second cell triangle two different ways. The shape-cast cull uses b3SignedVolume of point21, point22, point12, and the mover cull uses b3SignedVolume of point22, point12, point21."
   :date "2026-10-14"
   :origin "mas-bandwidth/fixed3d issue #53, opened 2026-09-01 by rowan-claude; moved here 2026-10-14"
   :kept (:github-issue "53" :labels ""))
  (item "port-f555ee4-tla-tree-moved-bit-and-pair-keys" :group "export-fixed3d-box3d-ports" :area "fixed3d"
   :title "TLA+ model of the broad-phase tree's moved bit and sorted pair keys, before the code"
   :text "Write the model, not the code. Read the upstream diff (git fetch https://github.com/erincatto/box3d.git f555ee4; git show f555ee4 -- src/dynamic_tree.c src/broad_phase.c src/broad_phase.h) and the triage porting/triage-2026-10-09/T1.md sections 1 to 6 (lines 31 to 195), and fixed3d's current src/dynamic_tree.c and src/broad_phase.c."
   :exists "Work was pushed: head port-f555ee4-tla-tree-moved-bit-and-pair-keys.w2 on branch sprint/port-f555ee4-tla-tree-moved-bit-and-pair-keys.w1.g1.e20 (latest attempt branch; the head may be an earlier commit on it) (attempt 3). Resume from it; do not start over."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work. Triage 2026-10-15: the model is open as PR #70 (branch sprint/port-f555ee4-tla-tree-moved-bit-and-pair-keys.w1.g1.e20, head 70734ec), with the corrections of card port-f555ee4-tla-model-corrections merged into it as PR #73. Its only red check is the CAA bot, because the commits are authored as nova-sprint. Land PR #70 rather than starting again."
   :date "2026-10-14"
   :origin "sprint card port-f555ee4-tla-tree-moved-bit-and-pair-keys (stream fixed3d-box3d-ports, column ready, tier heavy, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "port-f555ee4-tla-tree-moved-bit-and-pair-keys" :sprint-stream "fixed3d-box3d-ports" :sprint-column "ready" :tier "heavy" :sprint-base "sprint/fixed3d-box3d-ports" :needs "" :pushed-head "port-f555ee4-tla-tree-moved-bit-and-pair-keys.w2" :pushed-branch "sprint/port-f555ee4-tla-tree-moved-bit-and-pair-keys.w1.g1.e20 (latest attempt branch; the head may be an earlier commit on it)"))
  (item "port-51f056e-world-byte-count-reports-the-world" :group "export-fixed3d-box3d-ports" :area "fixed3d"
   :title "Port 51f056e's one held piece: b3GetWorldByteCount( world, log ) reports the world, not the process allocator"
   :text "Read porting/triage-2026-10-09/T3.md (read) lines 247 to 332. The refactor fixes b3Counters::byteCount, which reports the process allocator today. Port `b3GetWorldByteCount( world, log )` so the count is the bytes the WORLD holds (arrays, pools, tree, solver sets, the arena high-water mark), optionally logged per category."
   :exists "Work was pushed: head 0adaeb4bc98a736c99793ba796dfe4dfedc77d32 on branch sprint/port-51f056e-world-byte-count-reports-the-world.emma (attempt 4). Resume from it; do not start over."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work. Triage 2026-10-15: PR #71 (head dc6889f, branch sprint/port-51f056e-world-byte-count-reports-the-world.w3.g3.e20) conflicts with sprint/fixed3d-box3d-ports and was closed. It differs from the newer head named above only in CLAUDE.md and samples/sample.cpp: it widens b3Counters::byteCount from int to int64_t (ABI 200 to 208 bytes) and changes the sample display to %lld with a long long cast. If the widening is kept, the samples change is needed for the samples build; decide it with the step."
   :date "2026-10-14"
   :origin "sprint card port-51f056e-world-byte-count-reports-the-world (stream fixed3d-box3d-ports, column review, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "port-51f056e-world-byte-count-reports-the-world" :sprint-stream "fixed3d-box3d-ports" :sprint-column "review" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "" :pushed-head "0adaeb4bc98a736c99793ba796dfe4dfedc77d32" :pushed-branch "sprint/port-51f056e-world-byte-count-reports-the-world.emma"))
  (item "port-16f7f4c-mesh-contact-scratch-to-the-arena" :group "export-fixed3d-box3d-ports" :area "fixed3d"
   :title "Port 16f7f4c step 1: the mesh contact scratch arrays move from the C stack to the arena"
   :text "Read porting/triage-2026-10-09/T1.md (read) lines 282 to 372 (16f7f4c hunk-group table and 'already satisfied' section)."
   :exists "Work was pushed: head 49b6cd3df51d9e62a2e081330954b2e8da1fee2d on branch sprint/port-16f7f4c-mesh-contact-scratch-to-the-arena.w2.g1.e20 (attempt 2). Resume from it; do not start over."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work. Triage 2026-10-15: PR #69 (head 35c013c, branch sprint/port-16f7f4c-mesh-contact-scratch-to-the-arena.w1.g8.e20) is an earlier attempt of this step that conflicts with sprint/fixed3d-box3d-ports and was closed. One piece of it is not in the newer head named above: the TLA+ module tla/MeshContactArenaScratch.tla and .cfg (bump index, by-value arena restore; invariants Bounded, SizedByCap, CallerStable, property LiveReturn; TLC clean on bench b1). Carry that model across when this step resumes."
   :date "2026-10-14"
   :origin "sprint card port-16f7f4c-mesh-contact-scratch-to-the-arena (stream fixed3d-box3d-ports, column review, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "port-16f7f4c-mesh-contact-scratch-to-the-arena" :sprint-stream "fixed3d-box3d-ports" :sprint-column "review" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "" :pushed-head "49b6cd3df51d9e62a2e081330954b2e8da1fee2d" :pushed-branch "sprint/port-16f7f4c-mesh-contact-scratch-to-the-arena.w2.g1.e20"))
  (item "port-f555ee4-tree-node-relayout-and-empty-node-sentinel" :group "export-fixed3d-box3d-ports" :area "fixed3d"
   :title "Port f555ee4 part 1: the dynamic tree node relayout and the empty-node sentinel"
   :text "First half of PR #159: the tree's data layout, with the pair traversal still the old one. Read triage T1.md (porting/triage-2026-10-09/T1.md (read) lines 31 to 280: hunk-group table, API changes, determinism exposure, dependencies) and tla/BroadPhaseTree.tla (the model this card cites from its code)."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work."
   :date "2026-10-14"
   :origin "sprint card port-f555ee4-tree-node-relayout-and-empty-node-sentinel (stream fixed3d-box3d-ports, column waiting, tier heavy, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "port-f555ee4-tree-node-relayout-and-empty-node-sentinel" :sprint-stream "fixed3d-box3d-ports" :sprint-column "waiting" :tier "heavy" :sprint-base "sprint/fixed3d-box3d-ports" :needs "port-954cf87-fixes-land-the-branch,port-f555ee4-tla-tree-moved-bit-and-pair-keys" :pushed-head "" :pushed-branch ""))
  (item "port-f555ee4-tree-against-tree-pairs-and-sorted-keys" :group "export-fixed3d-box3d-ports" :area "fixed3d"
   :title "Port f555ee4 part 2: tree-against-tree pair traversal, sorted pair keys, test_broad_phase"
   :text "Second half of PR #159. Read porting/triage-2026-10-09/T1.md (read) sections 2 and 5 (hunk groups, determinism exposure) and tla/BroadPhaseTree.tla."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work. Triage 2026-10-15: also carries the fifth 9e5a4cd bug fix (a static rebuild clears the move marks), which the 9e5a4cd block deferred because it needs this tree rework (878ccc6 ported the four that apply without it)."
   :date "2026-10-14"
   :origin "sprint card port-f555ee4-tree-against-tree-pairs-and-sorted-keys (stream fixed3d-box3d-ports, column waiting, tier heavy, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "port-f555ee4-tree-against-tree-pairs-and-sorted-keys" :sprint-stream "fixed3d-box3d-ports" :sprint-column "waiting" :tier "heavy" :sprint-base "sprint/fixed3d-box3d-ports" :needs "port-f555ee4-tree-node-relayout-and-empty-node-sentinel" :pushed-head "" :pushed-branch ""))
  (item "port-16f7f4c-tree-perf-engine-groups" :group "export-fixed3d-box3d-ports" :area "fixed3d"
   :title "Port 16f7f4c step 2: the eight engine-local tree-perf groups (the wide mesh solver is held)"
   :text "Read porting/triage-2026-10-09/T1.md (read) lines 282 to 526 (16f7f4c: hunk groups, already-satisfied, fixed-point hazards, tests, determinism, dependencies, effort). Port the eight engine-local groups the plan lists, one commit per group, each with a note of which hunks it took."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work."
   :date "2026-10-14"
   :origin "sprint card port-16f7f4c-tree-perf-engine-groups (stream fixed3d-box3d-ports, column waiting, tier heavy, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "port-16f7f4c-tree-perf-engine-groups" :sprint-stream "fixed3d-box3d-ports" :sprint-column "waiting" :tier "heavy" :sprint-base "sprint/fixed3d-box3d-ports" :needs "port-f555ee4-tree-against-tree-pairs-and-sorted-keys,port-16f7f4c-mesh-contact-scratch-to-the-arena" :pushed-head "" :pushed-branch ""))
  (item "fixed3d-perf-profile-report" :group "export-fixed3d-perf" :area "fixed3d"
   :title "Profile report: where the time goes by scene and worker count, on every ISA the fleet has"
   :text "Measure, do not change code."
   :why "Moved out of the stopped sprint on 2026-10-14: neither rocketnet nor v1.3 work. Triage 2026-10-15: partial evidence is open as draft PR #83 (perf/PROFILE-2026-10-14.md: 78 uninstrumented phase rows, 13 scenes, workers 1 and 4, Vision scalar, b3 scalar, b3 AVX-512). Not done by its own account: top-15 sampled symbols, Apple/NEON coverage, hardware counters, several bucket attributions. This item also stands for fixed3d issue #62 (closed 2026-10-15), whose acceptance criteria (bit-exact across worker counts, the 40-bit inverse scale and 256-bit arithmetic kept, the failed experiments not repeated) bind every perf change."
   :date "2026-10-14"
   :origin "sprint card fixed3d-perf-profile-report (stream fixed3d-perf, column waiting, tier pro, base sprint/fixed3d-perf), moved out of the sprint 2026-10-14"
   :kept (:sprint-card "fixed3d-perf-profile-report" :sprint-stream "fixed3d-perf" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-perf" :needs "fixed3d-perf-harness" :pushed-head "" :pushed-branch ""))
  (item "fixed3d-isa-dispatch-interface" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Runtime ISA dispatch for fixed3d, the interface: the variant table, the registration point and the scalar equivalence test"
   :text "OWNER RULING (2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: split from fixed3d-isa-dispatch-core by the needs audit of 2026-10-15 afternoon: the dispatch core held 20 cards, and the avx2 spike and the NEON card use only its interface (the variant table, the registration point, the name-based force and the equivalence test), not its AVX-512 variant, its x86 CI or its AVX-512 bench gate. The owner, 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card fixed3d-isa-dispatch-interface (stream fixed3d-arch-opt, column ready, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "fixed3d-isa-dispatch-interface" :sprint-stream "fixed3d-arch-opt" :sprint-column "ready" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "" :pushed-head "" :pushed-branch ""))
  (item "fixed3d-isa-dispatch-arm-and-ci" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Runtime ISA dispatch for fixed3d, part 2: NEON as a variant and CI on every host kind"
   :text "OWNER RULING (2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: the owner, 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" the owner, 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card fixed3d-isa-dispatch-arm-and-ci (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "fixed3d-isa-dispatch-arm-and-ci" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-interface" :pushed-head "" :pushed-branch ""))
  (item "fixed3d-isa-dispatch-core" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Runtime ISA dispatch for fixed3d: one build, every variant, detected and forced"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Evidence: perf/findings/spike-runtime-cpu-dispatch-per-world.md (detection harvested; the dispatch refactor is this follow-up)."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card fixed3d-isa-dispatch-core (stream fixed3d-arch-opt, column ready, tier heavy, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "fixed3d-isa-dispatch-core" :sprint-stream "fixed3d-arch-opt" :sprint-column "ready" :tier "heavy" :sprint-base "sprint/fixed3d-box3d-ports" :needs "" :pushed-head "b4a1f3a3348902495ef89d15e3255c6c1b61bec7" :pushed-branch "sprint/fixed3d-isa-dispatch-core.w2.g1.e21"))
  (item "spike-f3d-avx2-wide-solver-variant" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "An avx2 variant of the wide contact solver: the portable x86 floor, measured end to end"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-avx2-wide-solver-variant (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-avx2-wide-solver-variant" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-interface" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-avx512-variant-on-skylake-w" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "The AVX-512 variant on Skylake-W: bit-identical, and is it faster after the frequency cost?"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-avx512-variant-on-skylake-w (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-avx512-variant-on-skylake-w" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-core" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-avx512-variant-vs-scalar-zen4" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Does the AVX-512 variant beat scalar on Zen 4, scene by scene and kernel by kernel?"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-avx512-variant-vs-scalar-zen4 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-avx512-variant-vs-scalar-zen4" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-core" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-avx512-zmm-width-8" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Eight lanes in zmm: a width-8 avx512 variant on Zen 4"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-avx512-zmm-width-8 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-avx512-zmm-width-8" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-core" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-body-gather-transpose-permutes" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Body-state gather and scatter by in-register transposes (vpermt2q, VBMI2) instead of scalar loads"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-body-gather-transpose-permutes (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-body-gather-transpose-permutes" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-core" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-ifma-lane-products-zen4" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "AVX-512 IFMA (vpmadd52luq/huq) for the exact lane products on Zen 4"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-ifma-lane-products-zen4 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-ifma-lane-products-zen4" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-core" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-invmul-fits-64-on-m5" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "The b3InvMul fits-64 arm on arm64, where the 128-bit multiply is two instructions"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-invmul-fits-64-on-m5 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-invmul-fits-64-on-m5" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-arm-and-ci" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-invmulw-lanes-fits-64-gate" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "b3InvMulW in lanes: a uniform fits-64 gate so the impulse side stops scalarising"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-invmulw-lanes-fits-64-gate (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-invmulw-lanes-fits-64-gate" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-core" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-neon-variant-on-m5" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "The neon narrow-phase variant on the M5: still bit-identical, still faster?"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-neon-variant-on-m5 (stream fixed3d-arch-opt, column waiting, tier flash, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-neon-variant-on-m5" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "flash" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-arm-and-ci" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-sme2-streaming-contact-solve-m5" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "SME2 streaming SVE for the wide contact solver on the M5: SMULH lanes, mode cost, shared unit"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-sme2-streaming-contact-solve-m5 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-sme2-streaming-contact-solve-m5" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-arm-and-ci" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-avx2-wide-solver-variant" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: the avx2 variant of the wide solver"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-avx2-wide-solver-variant (perf/findings/spike-f3d-avx2-wide-solver-variant.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-avx2-wide-solver-variant (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-avx2-wide-solver-variant" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-avx2-wide-solver-variant" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-avx512-zmm-width-8" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: width-8 avx512 contact solve"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-avx512-zmm-width-8 (perf/findings/spike-f3d-avx512-zmm-width-8.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-avx512-zmm-width-8 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-avx512-zmm-width-8" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-avx512-zmm-width-8" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-body-gather-transpose-permutes" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: transposed body gather and scatter"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-body-gather-transpose-permutes (perf/findings/spike-f3d-body-gather-transpose-permutes.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-body-gather-transpose-permutes (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-body-gather-transpose-permutes" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-body-gather-transpose-permutes" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-ifma-lane-products" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: the avx512ifma variant"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-ifma-lane-products-zen4 (perf/findings/spike-f3d-ifma-lane-products-zen4.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-ifma-lane-products (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-ifma-lane-products" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-ifma-lane-products-zen4" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-invmul-fits-64-arm64" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: the b3InvMul fits-64 arm on aarch64"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-invmul-fits-64-on-m5 (perf/findings/spike-f3d-invmul-fits-64-on-m5.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-invmul-fits-64-arm64 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-invmul-fits-64-arm64" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-invmul-fits-64-on-m5" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-invmulw-lanes-fits-64-gate" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: b3InvMulW in lanes behind the fits-64 gate"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-invmulw-lanes-fits-64-gate (perf/findings/spike-f3d-invmulw-lanes-fits-64-gate.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-invmulw-lanes-fits-64-gate (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-invmulw-lanes-fits-64-gate" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-invmulw-lanes-fits-64-gate" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-sme2-streaming-contact-solve" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: the sme2 variant of the contact solve"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-sme2-streaming-contact-solve-m5 (perf/findings/spike-f3d-sme2-streaming-contact-solve-m5.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-sme2-streaming-contact-solve (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-sme2-streaming-contact-solve" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-sme2-streaming-contact-solve-m5" :pushed-head "" :pushed-branch ""))
  (item "port-f3d-sve2-non-streaming-solver" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "Port: the sve2 variant on the M6"
   :text "OWNER RULING (Glenn, 2026-10-15): fixed3d must be deterministic itself, bit-identical across runs, worker counts and platforms; it may diverge slightly from box3d. Every ISA variant is bit-identical to the scalar variant, so a mixed fleet stays in lockstep. Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\" The spike spike-f3d-sve2-non-streaming-m6 (perf/findings/spike-f3d-sve2-non-streaming-m6.md) is this card's evidence."
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card port-f3d-sve2-non-streaming-solver (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "port-f3d-sve2-non-streaming-solver" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "spike-f3d-sve2-non-streaming-m6" :pushed-head "" :pushed-branch ""))
  (item "spike-f3d-sve2-non-streaming-m6" :group "export-fixed3d-arch-opt" :area "fixed3d"
   :title "If the M6 has non-streaming SVE2: the lane multiply without a mode switch"
   :text "SPIKE CONTRACT (KIND: spike; Glenn's tracer-bullet rule). Its end is stated before it starts: it runs end to end ONCE (change, measure, check, write up), then STOPS and names what it showed. No tuning past the first full measurement; a promising direction is a sentence in the finding. What is kept is what it taught; production code arrives only through the separate port card that needs this one, and only on a KEEP ve Source: Glenn 2026-10-15 13:51Z: \"Please card all the optimizations for architectures.\" Glenn 2026-10-15 13:54Z: \"I don't want to have to for example, have to do a build per-instruction set. They should all be automatically detected and used, where appropriate.\""
   :why "Scoped out of the sprint by Glenn 2026-10-15 (\"I want nothing in it except the tools/sprint work and rocketnet and schema. The rest should go out to the appropriate roadmap.\"): not tools/sprint, rocketnet or schema work."
   :date "2026-10-15"
   :origin "sprint card spike-f3d-sve2-non-streaming-m6 (stream fixed3d-arch-opt, column waiting, tier pro, base sprint/fixed3d-box3d-ports), moved out of the sprint 2026-10-15"
   :kept (:sprint-card "spike-f3d-sve2-non-streaming-m6" :sprint-stream "fixed3d-arch-opt" :sprint-column "waiting" :tier "pro" :sprint-base "sprint/fixed3d-box3d-ports" :needs "fixed3d-isa-dispatch-arm-and-ci" :pushed-head "" :pushed-branch ""))
))
