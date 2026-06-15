# msp_rigid_body
rapier f64 rigid body by ffm api or jni

The Rust crate exposes both Java access paths from the same `cdylib`:

- `src/abi/ffm.rs`: C ABI metadata for Java FFM and other C callers.
- `src/abi/jni.rs`: JNI-compatible `Java_*` wrappers that call the existing
  `rc_*` C ABI implementation.

Smoke test projects:

- `test21`: Gradle Java 21 JNI smoke test. Run with `gradle -p test21 check`.
- `test25`: Gradle Java 25 FFM smoke test. Run with `gradle -p test25 check`.

## Voxel collider support

Voxel colliders can now be created from three sources:

- Raw occupancy grids: `collider_builder_create_voxels`, JNI `voxelCollider(...)`,
  Java `byte[]`, or native memory through `VoxelGrid.address()`.
- Axis-aligned bounding boxes: `collider_builder_create_voxel_aabb`,
  Java `world.voxelAabbCollider(...)`, and FFM `colliderBuilderCreateVoxelAabb(...)`.
- Oriented bounding boxes: `collider_builder_create_voxel_obb` and
  Java `world.voxelObbCollider(...)`.

The build mode is controlled by `VoxelColliderOptions`:

- `Auto`: choose a mode from voxel count and body type.
- `Cuboids`: one cuboid per solid voxel.
- `GreedyCuboids`: merge adjacent solid voxels into larger cuboids.
- `SurfaceMesh`: build a triangle surface mesh for larger static voxel sets.

Use `VoxelBuildStats` before building when you need to estimate the result:
cell count, solid count, selected mode, estimated parts, vertices, triangles,
and generated grid dimensions. JNI exposes this through
`Collider.Builder.voxelStats`, `voxelAabbStats`, and `voxelObbStats`; FFM uses
the out-pointer ABI `voxel_aabb_build_stats_out`.

Queries can use voxel-shaped filters directly:

- `Query.countVoxelAabb(...)` / `Query.intersectVoxelAabb(...)`
- `Query.countVoxelObb(...)` / `Query.intersectVoxelObb(...)`

For common world insertion workflows, Java also has direct helpers:

- `PhysicsWorld.insertStaticVoxelAabb(...)`
- `PhysicsWorld.insertDynamicVoxelObb(...)`

The Java 21 `VoxelGrid` helper supports `get`, `clear`, `solidCount`,
`fillBox`, `fillAabb`, `fillSphere`, `copyFrom`, `union`, `subtract`, and
`intersect`, so simple voxel assets can be assembled without manually writing
the occupancy byte array.
