; fixed3d roadmap: the data ROADMAP.md is generated from (nova-roadmap, a nova-tools release binary;
; make roadmap renders, make roadmap-check fails on drift). Read as data, never evaluated.
; A cell is done only when its task is :done with :evidence, a test or a commit;
; :tested-revision is the main commit whose CI passed with that evidence.
(:schema 1
 :root "goal"
 :page (:title "fixed3d roadmap"
        :roadmaps ("collision" "dynamics" "fixed" "simd" "platforms" "upstream"))
 :nodes
 ((:id "goal" :type :work-set :children ("collision" "dynamics" "fixed" "simd" "platforms" "upstream"))
  (:id "collision" :type :roadmap :title "Shapes and collision"
   :text "The shapes, the narrow phase, scene queries, continuous collision and the character mover."
   :rows (("sphere-capsule" "spheres and capsules")
          ("hulls" "convex hulls, built from points")
          ("meshes" "triangle meshes, with stride and winding options")
          ("height-fields" "height fields")
          ("compounds" "compound shapes, with shared hulls, meshes and materials")
          ("manifolds" "contact manifolds for every shape pair")
          ("ray-casts" "ray casts")
          ("shape-casts" "shape casts")
          ("overlaps" "overlap queries")
          ("distance-toi" "distance and time of impact")
          ("continuous" "continuous collision for fast bodies")
          ("sensors" "sensors")
          ("events" "contact and hit events")
          ("mover" "character mover: plane collection and time of impact")
          ("broad-phase" "optimized broad phase: paired tree and sorted pair keys (upstream f555ee4)")
          ("heightfield-clockwise" "shape casts against clockwise height fields: behaviour decided and pinned by a test")
          ("mover-mesh-winding" "mover against a mirrored mesh hands GJK the original vertex order")
          ("heightfield-triangle" "height field triangle 2 spelled the same at both cull sites"))
   :columns (("done" "done"))
   :cells (("sphere-capsule" "done" "collision/sphere-capsule")
           ("hulls" "done" "collision/hulls")
           ("meshes" "done" "collision/meshes")
           ("height-fields" "done" "collision/height-fields")
           ("compounds" "done" "collision/compounds")
           ("manifolds" "done" "collision/manifolds")
           ("ray-casts" "done" "collision/ray-casts")
           ("shape-casts" "done" "collision/shape-casts")
           ("overlaps" "done" "collision/overlaps")
           ("distance-toi" "done" "collision/distance-toi")
           ("continuous" "done" "collision/continuous")
           ("sensors" "done" "collision/sensors")
           ("events" "done" "collision/events")
           ("mover" "done" "collision/mover")
           ("broad-phase" "done" "collision/broad-phase")
           ("heightfield-clockwise" "done" "collision/heightfield-clockwise")
           ("mover-mesh-winding" "done" "collision/mover-mesh-winding")
           ("heightfield-triangle" "done" "collision/heightfield-triangle")))
  (:id "collision/sphere-capsule" :type :task :title "spheres and capsules" :state :done :evidence ("test/test_shape.c: RayCastSphereHitTest" "test/test_shape.c: RayCastCapsuleSideTest" "test/test_mover.c: MoverCapsuleTouching") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/hulls" :type :task :title "convex hulls, built from points" :state :done :evidence ("test/test_hull.c: CreateHullCubeTest" "test/test_hull.c: CreateHullDeterminismTest" "test/test_hull.c: CreateHullDegenerateTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/meshes" :type :task :title "triangle meshes, with stride and winding options" :state :done :evidence ("test/test_mesh.c: MeshCreators" "test/test_mesh.c: MeshDenseStride" "test/test_mesh.c: MeshClockWise") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/height-fields" :type :task :title "height fields" :state :done :evidence ("test/test_height_field.c: HeightFieldCreate" "test/test_height_field.c: RayCastBruteForce" "test/test_height_field.c: FileRoundtrip") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/compounds" :type :task :title "compound shapes, with shared hulls, meshes and materials" :state :done :evidence ("test/test_compound.c: CompoundCreateMixed" "test/test_compound.c: CompoundMaterialDedup" "test/test_compound.c: CompoundSerializeRoundtrip") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/manifolds" :type :task :title "contact manifolds for every shape pair" :state :done :evidence ("test/test_manifold.c: CrossedEdgeTest" "test/test_manifold.c: EdgeAxisRandomOracleTest" "test/test_manifold.c: SphereHullSeamTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/ray-casts" :type :task :title "ray casts" :state :done :evidence ("test/test_body_query.c: CastRayHitsSphere" "test/test_body_query.c: CastRayClosestShape" "test/test_shape.c: RayCastShapeTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/shape-casts" :type :task :title "shape casts" :state :done :evidence ("test/test_body_query.c: CastShapeHitsBox" "test/test_distance.c: ShapeCastTest" "test/test_compound.c: CompoundShapeCastClosest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/overlaps" :type :task :title "overlap queries" :state :done :evidence ("test/test_body_query.c: OverlapTrue" "test/test_body_query.c: OverlapFilter" "test/test_compound.c: CompoundOverlap") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/distance-toi" :type :task :title "distance and time of impact" :state :done :evidence ("test/test_distance.c: ShapeDistanceTest" "test/test_distance.c: TimeOfImpactTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/continuous" :type :task :title "continuous collision for fast bodies" :state :done :evidence ("test/test_world.c: TestContinuousMoveEvent" "test/test_world.c: SetBulletDriftTest" "test/test_large_world.c: LargeWorldBulletTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/sensors" :type :task :title "sensors" :state :done :evidence ("test/test_world.c: TestSensor" "test/test_world.c: TestSensorVisitorMustBeConvex") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/events" :type :task :title "contact and hit events" :state :done :evidence ("test/test_world.c: TestContactEvents" "test/test_world.c: TestHitEvents" "test/test_world.c: TestCompoundHitEvents") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/mover" :type :task :title "character mover: plane collection and time of impact" :state :done :evidence ("test/test_mover.c: GamePlanes" "test/test_mover.c: MoverHullTouching" "test/test_body_query.c: MoverTOIHitsBox") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "collision/broad-phase" :type :task :title "optimized broad phase: paired tree and sorted pair keys (upstream f555ee4)" :state :todo
   :note "Three steps: a TLA+ model of the tree moved bit and sorted pair keys, then the node relayout and empty-node sentinel, then tree-against-tree pair traversal with test_broad_phase. Work in PR #99; layer list in porting/."
   :github-pr "99"
   :sprint-cards "port-f555ee4-tla-tree-moved-bit-and-pair-keys,port-f555ee4-tree-node-relayout-and-empty-node-sentinel,port-f555ee4-tree-against-tree-pairs-and-sorted-keys"
   :tier "heavy"
   :sprint-base "sprint/fixed3d-box3d-ports"
   :pushed-branch "sprint/port-f555ee4-tla-tree-moved-bit-and-pair-keys.w1.g1.e20")
  (:id "collision/heightfield-clockwise" :type :task :title "shape casts against clockwise height fields: behaviour decided and pinned by a test" :state :todo
   :note "Measured in 7ae09e6 on sprint/fixed3d-box3d-ports: on a clockwise field 1738 of 3000 casts flip hit to miss under upstream back-side culling. Glenn decides keep or restore; a test pins it."
   :github-issue "50")
  (:id "collision/mover-mesh-winding" :type :task :title "mover against a mirrored mesh hands GJK the original vertex order" :state :todo
   :note "b3CollideMoverAndMesh (src/mesh.c) swaps vertex2 and vertex3 before building triangleVertices, so GJK gets the swapped order, not only the signed-volume cull."
   :github-issue "51")
  (:id "collision/heightfield-triangle" :type :task :title "height field triangle 2 spelled the same at both cull sites" :state :todo
   :note "src/height_field.c spells it point21, point22, point12 in the shape-cast cull and point22, point12, point21 in the mover cull."
   :github-issue "53")
  (:id "dynamics" :type :roadmap :title "Bodies, solver and joints"
   :text "Rigid bodies, mass, sleeping, the multithreaded solver and every joint type."
   :rows (("bodies" "rigid bodies: create, destroy and validate")
          ("mass" "mass properties from shapes, or set by hand")
          ("sleep" "sleeping and waking")
          ("materials" "friction mixed per material")
          ("explosions" "explosions")
          ("contact-recycling" "contact recycling")
          ("workers" "multithreaded solver with a settable worker count")
          ("distance-joint" "distance joint")
          ("filter-joint" "filter joint")
          ("motor-joint" "motor joint")
          ("prismatic-joint" "prismatic joint")
          ("revolute-joint" "revolute joint")
          ("spherical-joint" "spherical joint")
          ("weld-joint" "weld joint")
          ("wheel-joint" "wheel joint")
          ("parallel-joints" "joints solved in parallel blocks")
          ("axial-solve" "revolute and spherical joints solve spring, motor and limit in one block (upstream 0cfc475)")
          ("gyro-tolerance" "isotropic inertia tolerance gates gyroscopic torque (upstream efca0c1)")
          ("world-byte-count" "b3GetWorldByteCount reports the world, not the process allocator (upstream 51f056e)"))
   :columns (("done" "done"))
   :cells (("bodies" "done" "dynamics/bodies")
           ("mass" "done" "dynamics/mass")
           ("sleep" "done" "dynamics/sleep")
           ("materials" "done" "dynamics/materials")
           ("explosions" "done" "dynamics/explosions")
           ("contact-recycling" "done" "dynamics/contact-recycling")
           ("workers" "done" "dynamics/workers")
           ("distance-joint" "done" "dynamics/distance-joint")
           ("filter-joint" "done" "dynamics/filter-joint")
           ("motor-joint" "done" "dynamics/motor-joint")
           ("prismatic-joint" "done" "dynamics/prismatic-joint")
           ("revolute-joint" "done" "dynamics/revolute-joint")
           ("spherical-joint" "done" "dynamics/spherical-joint")
           ("weld-joint" "done" "dynamics/weld-joint")
           ("wheel-joint" "done" "dynamics/wheel-joint")
           ("parallel-joints" "done" "dynamics/parallel-joints")
           ("axial-solve" "done" "dynamics/axial-solve")
           ("gyro-tolerance" "done" "dynamics/gyro-tolerance")
           ("world-byte-count" "done" "dynamics/world-byte-count")))
  (:id "dynamics/bodies" :type :task :title "rigid bodies: create, destroy and validate" :state :done :evidence ("test/test_world.c: HelloWorld" "test/test_world.c: DestroyAllBodiesWorld" "test/test_world.c: TestIsValid") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/mass" :type :task :title "mass properties from shapes, or set by hand" :state :done :evidence ("test/test_shape.c: ShapeMassTest" "test/test_body.c: SetMassDataRoundTrip" "test/test_body.c: SetMassDataWorldInertiaRotated") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/sleep" :type :task :title "sleeping and waking" :state :done :evidence ("test/test_world.c: EnableSleepFlagSyncTest" "test/test_world.c: SleepingWorldTreeJoinTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/materials" :type :task :title "friction mixed per material" :state :done :evidence ("test/test_world.c: TestCompoundContactMaterials" "test/test_world.c: TestFrictionCapWrap") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/explosions" :type :task :title "explosions" :state :done :evidence ("test/test_world.c: TestExplosion") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/contact-recycling" :type :task :title "contact recycling" :state :done :evidence ("test/test_world.c: EnableContactRecyclingTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/workers" :type :task :title "multithreaded solver with a settable worker count" :state :done :evidence ("test/test_world.c: TestSetWorkerCount" "test/test_world.c: TestOverflowColorPile" "test/test_determinism.c: MultithreadingTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/distance-joint" :type :task :title "distance joint" :state :done :evidence ("test/test_joint.c: TestDistanceJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/filter-joint" :type :task :title "filter joint" :state :done :evidence ("test/test_joint.c: TestFilterJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/motor-joint" :type :task :title "motor joint" :state :done :evidence ("test/test_joint.c: TestMotorJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/prismatic-joint" :type :task :title "prismatic joint" :state :done :evidence ("test/test_joint.c: TestPrismaticJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/revolute-joint" :type :task :title "revolute joint" :state :done :evidence ("test/test_joint.c: TestRevoluteJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/spherical-joint" :type :task :title "spherical joint" :state :done :evidence ("test/test_joint.c: TestSphericalJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/weld-joint" :type :task :title "weld joint" :state :done :evidence ("test/test_joint.c: TestWeldJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/wheel-joint" :type :task :title "wheel joint" :state :done :evidence ("test/test_joint.c: TestWheelJoint") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/parallel-joints" :type :task :title "joints solved in parallel blocks" :state :done :evidence ("test/test_joint.c: TestParallelJoint" "test/test_joint.c: TestParallelJointSentinelTorqueCap") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "dynamics/axial-solve" :type :task :title "revolute and spherical joints solve spring, motor and limit in one block (upstream 0cfc475)" :state :todo
   :note "A reassociation that moves goldens: design first, then port with every golden harvested in narrow and ludicrous modes over worker counts 1 to 5, or refuse it with the measurement."
   :upstream "0cfc475"
   :tier "heavy"
   :sprint-base "sprint/fixed3d-box3d-ports")
  (:id "dynamics/gyro-tolerance" :type :task :title "isotropic inertia tolerance gates gyroscopic torque (upstream efca0c1)" :state :todo
   :note "Engine half in progress on port/efca0c1-gyro-tolerance against sprint/fixed3d-box3d-ports; resume from it. Isotropy test in exact 128-bit raw products."
   :upstream "efca0c1"
   :tier "pro"
   :pushed-branch "port/efca0c1-gyro-tolerance")
  (:id "dynamics/world-byte-count" :type :task :title "b3GetWorldByteCount reports the world, not the process allocator (upstream 51f056e)" :state :todo
   :note "Pushed work in review on its sprint branch."
   :sprint-card "port-51f056e-world-byte-count-reports-the-world"
   :tier "pro"
   :pushed-head "0adaeb4bc98a736c99793ba796dfe4dfedc77d32"
   :pushed-branch "sprint/port-51f056e-world-byte-count-reports-the-world.emma")
  (:id "fixed" :type :roadmap :title "Fixed point, precision and determinism"
   :text "Q48.16 everywhere, uniform precision across a huge world, bit-exact results and replay."
   :rows (("q48-16" "Q48.16 fixed point through the whole simulation")
          ("fixed-library" "fixed-point core from the fixed library, behind Box3D names")
          ("cross-platform" "bit-exact results on every platform")
          ("worker-determinism" "same results for any worker count")
          ("large-world" "uniform precision across a ±1.4×10¹⁴ m world")
          ("ludicrous" "LUDICROUS MODE: 128-bit positions and broad phase")
          ("heavy-bodies" "very heavy bodies still respond to impulses")
          ("recording" "recording, replay and scrubbing")
          ("baked-version" "baked geometry from the other position width is refused")
          ("precision-audit" "precision-envelope audit: every magnitude-dependent failure found and made loud")
          ("center-of-mass" "centre of mass stays exact for heavy bodies")
          ("angular-damping" "angular damping slows a slow-spinning large body"))
   :columns (("done" "done"))
   :cells (("q48-16" "done" "fixed/q48-16")
           ("fixed-library" "done" "fixed/fixed-library")
           ("cross-platform" "done" "fixed/cross-platform")
           ("worker-determinism" "done" "fixed/worker-determinism")
           ("large-world" "done" "fixed/large-world")
           ("ludicrous" "done" "fixed/ludicrous")
           ("heavy-bodies" "done" "fixed/heavy-bodies")
           ("recording" "done" "fixed/recording")
           ("baked-version" "done" "fixed/baked-version")
           ("precision-audit" "done" "fixed/precision-audit")
           ("center-of-mass" "done" "fixed/center-of-mass")
           ("angular-damping" "done" "fixed/angular-damping")))
  (:id "fixed/q48-16" :type :task :title "Q48.16 fixed point through the whole simulation" :state :done :evidence ("test/test_math.c: VendoredFixedTest" "test/test_math.c: Exp2Log2PowTest" "test/test_math.c: Quat30Test") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/fixed-library" :type :task :title "fixed-point core from the fixed library, behind Box3D names" :state :done :evidence ("commit 042339c (Vendor mas-bandwidth/fixed and take the scalar core from it)" ".github/workflows/vendor-drift.yml: extern/fixed matches its pin, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13"
   :github-issue "27")
  (:id "fixed/cross-platform" :type :task :title "bit-exact results on every platform" :state :done :evidence ("test/test_determinism.c: CrossPlatformTest" ".github/workflows/build.yml: test matrix (ubuntu gcc and clang, windows, macos) job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/worker-determinism" :type :task :title "same results for any worker count" :state :done :evidence ("test/test_determinism.c: MultithreadingTest" "test/test_determinism.c: WavePileTest" "test/test_determinism.c: MeshDropTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/large-world" :type :task :title "uniform precision across a ±1.4×10¹⁴ m world" :state :done :evidence ("test/test_large_world.c: LargeWorldStackTest" "test/test_large_world.c: LargeWorldQueryTest" "test/test_body.c: FarSingleSphereMass") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/ludicrous" :type :task :title "LUDICROUS MODE: 128-bit positions and broad phase" :state :done :evidence ("test/test_large_world.c: LargeWorldBeyondRangeTest" ".github/workflows/build.yml: ubuntu-wide-positions job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/heavy-bodies" :type :task :title "very heavy bodies still respond to impulses" :state :done :evidence ("test/test_body.c: AsteroidImpulseResponse" "test/test_body.c: InverseInertiaScaleEnvelope" "test/test_joint.c: TestJointInverseScaleCorrection") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/recording" :type :task :title "recording, replay and scrubbing" :state :done :evidence ("test/test_recording.c: AllOps" "test/test_recording.c: ScrubBackward" "test/test_name_cache.c: RollbackNames") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "fixed/baked-version" :type :task :title "baked geometry from the other position width is refused" :state :todo
   :note "B3_HULL_VERSION, B3_MESH_VERSION and B3_HEIGHT_FIELD_VERSION (include/box3d/types.h) are the same in both configs, so a LUDICROUS_MODE blob passes the guard and is read 48 bytes short."
   :github-issue "25")
  (:id "fixed/precision-audit" :type :task :title "precision-envelope audit: every magnitude-dependent failure found and made loud" :state :todo
   :note "Inverses of large quantities, products of large quantities in reductions, and silent truncation at int64 casts, swept across the engine."
   :github-issue "31")
  (:id "fixed/center-of-mass" :type :task :title "centre of mass stays exact for heavy bodies" :state :todo
   :note "b3UpdateBodyMassData (src/body.c) multiplies by the truncated reciprocal of the mass, so the centre collapses to the body origin for heavy bodies."
   :github-issue "32")
  (:id "fixed/angular-damping" :type :task :title "angular damping slows a slow-spinning large body" :state :todo
   :note "b3IntegrateVelocitiesTask (src/solver.c) damps by b3FixMul, so a component below half a quantum of decrement never changes."
   :github-issue "41")
  (:id "simd" :type :roadmap :title "SIMD and performance"
   :text "The SIMD paths, all bit-identical to scalar, and how fast the engine runs against float Box3D."
   :rows (("scalar" "scalar path, the reference")
          ("neon" "NEON narrow phase on arm64, bit-identical")
          ("avx512" "AVX-512 wide contact solver, bit-identical")
          ("benchmarks" "benchmark suite with published float comparisons")
          ("isa-dispatch" "runtime ISA dispatch: one build, every variant detected or forced")
          ("avx2" "AVX2 wide contact solver, the portable x86 floor")
          ("avx512-measured" "AVX-512 measured against scalar on Zen 4 and Skylake-W")
          ("avx512-width8" "AVX-512 width-8 contact solve, with transposed body gather")
          ("avx512-ifma" "AVX-512 IFMA lane products")
          ("fits-64" "fits-64 fast path for exact inverse multiplies (arm64 and in lanes)")
          ("neon-m5" "NEON still bit-identical and faster on Apple M5")
          ("sme2" "SME2 streaming contact solve on Apple M5")
          ("sve2" "non-streaming SVE2 contact solve on Apple M6")
          ("tree-perf" "upstream 16f7f4c tree and mesh performance")
          ("profile-report" "profile report: where the time goes by scene, worker count and ISA"))
   :columns (("done" "done"))
   :cells (("scalar" "done" "simd/scalar")
           ("neon" "done" "simd/neon")
           ("avx512" "done" "simd/avx512")
           ("benchmarks" "done" "simd/benchmarks")
           ("isa-dispatch" "done" "simd/isa-dispatch")
           ("avx2" "done" "simd/avx2")
           ("avx512-measured" "done" "simd/avx512-measured")
           ("avx512-width8" "done" "simd/avx512-width8")
           ("avx512-ifma" "done" "simd/avx512-ifma")
           ("fits-64" "done" "simd/fits-64")
           ("neon-m5" "done" "simd/neon-m5")
           ("sme2" "done" "simd/sme2")
           ("sve2" "done" "simd/sve2")
           ("tree-perf" "done" "simd/tree-perf")
           ("profile-report" "done" "simd/profile-report")))
  (:id "simd/scalar" :type :task :title "scalar path, the reference" :state :done :evidence (".github/workflows/build.yml: ubuntu-gcc job, green on 29d5544" "test/test_determinism.c: CrossPlatformTest") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "simd/neon" :type :task :title "NEON narrow phase on arm64, bit-identical" :state :done :evidence ("commit 5aca95a (Add BOX3D_NEON: the narrow phase four lanes wide on Apple silicon)" ".github/workflows/build.yml: macos-relwithdebinfo (-DBOX3D_NEON=ON) job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "simd/avx512" :type :task :title "AVX-512 wide contact solver, bit-identical" :state :unknown
   :note "Added in commit a9f4dc8, but no CI run on AVX-512 hardware has passed (the extended ubuntu-clang-avx512 job is compile-only on most runners and the extended workflow has no green run). A green test run on AVX-512 hardware makes this done.")
  (:id "simd/benchmarks" :type :task :title "benchmark suite with published float comparisons" :state :done :evidence ("commit c52e502 (Publish contact and joint performance results with state hash tool)" "benchmark/2026-09-07-contact-joint-performance.md") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "simd/isa-dispatch" :type :task :title "runtime ISA dispatch: one build, every variant detected or forced" :state :todo
   :note "Interface (variant table, registration, scalar equivalence test), core (detect and force), then NEON as a variant with CI on every host kind. Core work in PR #90."
   :github-pr "90"
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "fixed3d-isa-dispatch-interface,fixed3d-isa-dispatch-core,fixed3d-isa-dispatch-arm-and-ci"
   :pushed-head "b4a1f3a3348902495ef89d15e3255c6c1b61bec7"
   :pushed-branch "sprint/fixed3d-isa-dispatch-core.w2.g1.e21")
  (:id "simd/avx2" :type :task :title "AVX2 wide contact solver, the portable x86 floor" :state :todo
   :note "Spike measured end to end, then the port."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-avx2-wide-solver-variant,port-f3d-avx2-wide-solver-variant")
  (:id "simd/avx512-measured" :type :task :title "AVX-512 measured against scalar on Zen 4 and Skylake-W" :state :todo
   :note "Scene by scene and kernel by kernel, including the Skylake-W frequency cost."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-avx512-variant-vs-scalar-zen4,spike-f3d-avx512-variant-on-skylake-w")
  (:id "simd/avx512-width8" :type :task :title "AVX-512 width-8 contact solve, with transposed body gather" :state :todo
   :note "Eight lanes in zmm on Zen 4, and body-state gather and scatter by in-register transposes (vpermt2q, VBMI2)."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-avx512-zmm-width-8,port-f3d-avx512-zmm-width-8,spike-f3d-body-gather-transpose-permutes,port-f3d-body-gather-transpose-permutes")
  (:id "simd/avx512-ifma" :type :task :title "AVX-512 IFMA lane products" :state :todo
   :note "vpmadd52luq/huq for the exact lane products on Zen 4."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-ifma-lane-products-zen4,port-f3d-ifma-lane-products")
  (:id "simd/fits-64" :type :task :title "fits-64 fast path for exact inverse multiplies (arm64 and in lanes)" :state :todo
   :note "b3InvMul fits-64 arm on aarch64, and b3InvMulW in lanes behind a uniform fits-64 gate so the impulse side stops scalarising."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-invmul-fits-64-on-m5,port-f3d-invmul-fits-64-arm64,spike-f3d-invmulw-lanes-fits-64-gate,port-f3d-invmulw-lanes-fits-64-gate")
  (:id "simd/neon-m5" :type :task :title "NEON still bit-identical and faster on Apple M5" :state :todo
   :note "Re-measure the NEON variant on the M5."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-card "spike-f3d-neon-variant-on-m5"
   :tier "flash")
  (:id "simd/sme2" :type :task :title "SME2 streaming contact solve on Apple M5" :state :todo
   :note "SMULH lanes, mode-switch cost, shared unit."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-sme2-streaming-contact-solve-m5,port-f3d-sme2-streaming-contact-solve")
  (:id "simd/sve2" :type :task :title "non-streaming SVE2 contact solve on Apple M6" :state :todo
   :note "If the M6 has non-streaming SVE2: the lane multiply without a mode switch."
   :sprint-stream "fixed3d-arch-opt"
   :sprint-cards "spike-f3d-sve2-non-streaming-m6,port-f3d-sve2-non-streaming-solver")
  (:id "simd/tree-perf" :type :task :title "upstream 16f7f4c tree and mesh performance" :state :todo
   :note "Step 1 moves the mesh contact scratch from the C stack to the arena (pushed, in review); step 2 the eight engine-local tree-perf groups, after the f555ee4 broad phase. The wide mesh solver is held."
   :sprint-cards "port-16f7f4c-mesh-contact-scratch-to-the-arena,port-16f7f4c-tree-perf-engine-groups"
   :sprint-base "sprint/fixed3d-box3d-ports"
   :pushed-head "49b6cd3df51d9e62a2e081330954b2e8da1fee2d"
   :pushed-branch "sprint/port-16f7f4c-mesh-contact-scratch-to-the-arena.w2.g1.e20")
  (:id "simd/profile-report" :type :task :title "profile report: where the time goes by scene, worker count and ISA" :state :todo
   :note "On every ISA the fleet has. Partial Linux evidence in draft PR #83."
   :github-pr "83"
   :sprint-card "fixed3d-perf-profile-report"
   :sprint-stream "fixed3d-perf")
  (:id "platforms" :type :roadmap :title "Platforms"
   :text "Where the full test suite runs on every pull request, and where it is not yet proven."
   :rows (("linux-gcc" "Linux, GCC")
          ("linux-clang" "Linux, Clang")
          ("macos" "macOS on Apple silicon")
          ("windows" "Windows x64 (clang-cl)")
          ("windows-arm64" "Windows on arm64")
          ("mingw" "Windows, MinGW")
          ("wasm" "WebAssembly (Emscripten)")
          ("sanitizers" "clean under ASan, UBSan, TSan and MSan"))
   :columns (("done" "done"))
   :cells (("linux-gcc" "done" "platforms/linux-gcc")
           ("linux-clang" "done" "platforms/linux-clang")
           ("macos" "done" "platforms/macos")
           ("windows" "done" "platforms/windows")
           ("windows-arm64" "done" "platforms/windows-arm64")
           ("mingw" "done" "platforms/mingw")
           ("wasm" "done" "platforms/wasm")
           ("sanitizers" "done" "platforms/sanitizers")))
  (:id "platforms/linux-gcc" :type :task :title "Linux, GCC" :state :done :evidence (".github/workflows/build.yml: ubuntu-gcc job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "platforms/linux-clang" :type :task :title "Linux, Clang" :state :done :evidence (".github/workflows/build.yml: ubuntu-clang job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "platforms/macos" :type :task :title "macOS on Apple silicon" :state :done :evidence (".github/workflows/build.yml: macos-relwithdebinfo job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "platforms/windows" :type :task :title "Windows x64 (clang-cl)" :state :done :evidence (".github/workflows/build.yml: windows-relwithdebinfo job, green on 29d5544") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "platforms/windows-arm64" :type :task :title "Windows on arm64" :state :unknown
   :note "Only in the manually triggered extended workflow (windows-arm64), which has no green run. A green extended run makes this done.")
  (:id "platforms/mingw" :type :task :title "Windows, MinGW" :state :unknown
   :note "Only in the extended workflow (windows-mingw), which has no green run. A green extended run makes this done.")
  (:id "platforms/wasm" :type :task :title "WebAssembly (Emscripten)" :state :unknown
   :note "Only in the extended workflow (emscripten, build only), which has no green run. A green extended run makes this done.")
  (:id "platforms/sanitizers" :type :task :title "clean under ASan, UBSan, TSan and MSan" :state :unknown
   :note "Sanitizer jobs are in the extended workflow only, which has no green run; the README reports local ASan+UBSan passes. A green extended run makes this done.")
  (:id "upstream" :type :roadmap :title "Box3D parity and samples"
   :text "Keeping up with upstream Box3D, and its samples app."
   :rows (("port-47d7f7c" "engine ported up to upstream box3d 47d7f7c")
          ("port-efca0c1" "engine ported up to upstream box3d efca0c1")
          ("samples" "samples app builds against the engine")
          ("follow-cam" "samples: follow cam and world text (upstream 3fc20f5)" :later t)
          ("samples-47d7f7c" "samples: character mover, issues and joint pages (upstream 47d7f7c)" :later t))
   :columns (("done" "done"))
   :cells (("port-47d7f7c" "done" "upstream/port-47d7f7c")
           ("port-efca0c1" "done" "upstream/port-efca0c1")
           ("samples" "done" "upstream/samples")
           ("follow-cam" "done" "upstream/follow-cam")
           ("samples-47d7f7c" "done" "upstream/samples-47d7f7c")))
  (:id "upstream/port-47d7f7c" :type :task :title "engine ported up to upstream box3d 47d7f7c" :state :done :evidence ("commit 61fe62f (Merge pull request #46 from mas-bandwidth/port/47d7f7c-clockwise-mover)") :tested-revision "29d5544b02f660adc5bd36ddfbf92f8b91beda13")
  (:id "upstream/port-efca0c1" :type :task :title "engine ported up to upstream box3d efca0c1" :state :todo
   :note "Plan #65 ports (47d7f7c..5d83df8) land with PR #94 from sprint/fixed3d-box3d-ports; 0cfc475 and efca0c1 are their own rows under Bodies, solver and joints."
   :github-pr "94")
  (:id "upstream/samples" :type :task :title "samples app builds against the engine" :state :unknown
   :note "The samples jobs are in the extended workflow only, which has no green run. A green extended samples job makes this done.")
  (:id "upstream/follow-cam" :type :task :title "samples: follow cam and world text (upstream 3fc20f5)" :state :todo
   :note "Held for a bench with a display; no simulation content."
   :github-issue "20")
  (:id "upstream/samples-47d7f7c" :type :task :title "samples: character mover, issues and joint pages (upstream 47d7f7c)" :state :todo
   :note "sample_character.cpp, sample_issues.cpp, sample_joint.cpp and samples/mover.*; held on the same ground as #20."
   :github-issue "47")))
