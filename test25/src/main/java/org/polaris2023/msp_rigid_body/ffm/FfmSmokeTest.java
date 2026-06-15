package org.polaris2023.msp_rigid_body.ffm;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.nio.file.Path;

public final class FfmSmokeTest {
    private static final double EPSILON = 1.0e-9;

    private FfmSmokeTest() {
    }

    public static void main(String[] args) {
        int javaVersion = Runtime.version().feature();
        if (javaVersion != 25) {
            throw new AssertionError("test25 must run on Java 25, got Java " + javaVersion);
        }

        String nativePath = System.getProperty("rigidbody.native.path");
        if (nativePath == null || nativePath.isBlank()) {
            throw new AssertionError("missing rigidbody.native.path");
        }

        try (Arena arena = Arena.ofShared()) {
            RigidBodyFfm api = new RigidBodyFfm(Path.of(nativePath), arena);

            if (api.abiVersion() < 1) {
                throw new AssertionError("invalid ABI version");
            }

            MemorySegment world = api.worldCreate(0.0, -9.81, 0.0);
            try {
                MemorySegment gravity = api.worldGetGravity(world);
                assertClose(-9.81, RigidBodyFfm.y(gravity), "initial gravity y");

                api.worldSetGravity(world, 1.0, 2.0, 3.0);
                gravity = api.worldGetGravity(world);
                assertClose(1.0, RigidBodyFfm.x(gravity), "gravity x");
                assertClose(2.0, RigidBodyFfm.y(gravity), "gravity y");
                assertClose(3.0, RigidBodyFfm.z(gravity), "gravity z");

                MemorySegment builder = api.rigidBodyBuilderCreate(0);
                api.rigidBodyBuilderSetTranslation(builder, 4.0, 5.0, 6.0);
                MemorySegment builtBody = api.rigidBodyBuilderBuild(builder);
                if (builtBody.address() == 0L) {
                    throw new AssertionError("rigid_body_builder_build returned null");
                }
                long body = api.worldInsertRigidBody(world, builtBody);
                if (body == 0L) {
                    throw new AssertionError("world_insert_rigid_body returned zero handle");
                }

                MemorySegment translation = api.rigidBodyGetTranslation(world, body);
                assertClose(4.0, RigidBodyFfm.x(translation), "body translation x");
                assertClose(5.0, RigidBodyFfm.y(translation), "body translation y");
                assertClose(6.0, RigidBodyFfm.z(translation), "body translation z");
                api.worldStep(world, 1.0 / 60.0);
            } finally {
                api.worldDestroy(world);
            }

            MemorySegment tree = api.crbTreeCreate();
            try {
                if (!api.crbTreeInsert(tree, 10L, api.aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0))) {
                    throw new AssertionError("crb_tree_insert 10 failed");
                }
                if (!api.crbTreeInsert(tree, 20L, api.aabb(2.0, 2.0, 2.0, 3.0, 3.0, 3.0))) {
                    throw new AssertionError("crb_tree_insert 20 failed");
                }
                int hitCount = api.crbTreeQueryAabbCount(tree, api.aabb(0.5, 0.5, 0.5, 2.5, 2.5, 2.5));
                if (hitCount != 2) {
                    throw new AssertionError("crb_tree_query_aabb_count expected 2, got " + hitCount);
                }
            } finally {
                api.crbTreeDestroy(tree);
            }

            world = api.worldCreate(0.0, -9.81, 0.0);
            try {
                MemorySegment voxelAabb = api.aabb(0.0, 0.0, 0.0, 2.0, 1.0, 1.0);
                MemorySegment options = api.voxelOptions(0, false, 128, 20_000);
                MemorySegment stats = api.voxelAabbBuildStats(voxelAabb, 0.5, options);
                if (RigidBodyFfm.voxelStatsSolidCount(stats) == 0 || RigidBodyFfm.voxelStatsSelectedMode(stats) == 0) {
                    throw new AssertionError("invalid voxel AABB stats");
                }

                MemorySegment builder = api.colliderBuilderCreateVoxelAabb(voxelAabb, 0.5, options);
                if (builder.address() == 0L) {
                    throw new AssertionError("collider_builder_create_voxel_aabb returned null");
                }
                MemorySegment collider = api.colliderBuilderBuild(builder);
                if (collider.address() == 0L) {
                    throw new AssertionError("collider_builder_build for voxel AABB returned null");
                }
                long colliderHandle = api.worldInsertCollider(world, collider);
                if (colliderHandle == 0L) {
                    throw new AssertionError("world_insert_collider for voxel AABB returned zero");
                }
                api.worldStep(world, 1.0 / 60.0);
            } finally {
                api.worldDestroy(world);
            }
        }

        System.out.println("FFM smoke test passed on Java " + javaVersion);
    }

    private static void assertClose(double expected, double actual, String label) {
        if (Math.abs(expected - actual) > EPSILON) {
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        }
    }
}
