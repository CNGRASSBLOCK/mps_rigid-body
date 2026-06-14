package org.polaris2023.msp_rigid_body;

import org.polaris2023.msp_rigid_body.util.PhysicsWorld;
import org.polaris2023.msp_rigid_body.util.RigidBody;
import sun.misc.Unsafe;

import java.lang.reflect.Field;

public final class JniSmokeTest {
    private static final double EPSILON = 1.0e-9;
    private static final int VOXEL_MODE_GREEDY_CUBOIDS = 2;

    private JniSmokeTest() {
    }

    public static void main(String[] args) throws Exception {
        int javaVersion = Runtime.version().feature();
        if (javaVersion != 21) {
            throw new AssertionError("test21 must run on Java 21, got Java " + javaVersion);
        }

        int abiVersion = RigidBodyNative.abiVersion();
        if (abiVersion < 1) {
            throw new AssertionError("invalid ABI version: " + abiVersion);
        }

        try (PhysicsWorld world = new PhysicsWorld(0.0, -9.81, 0.0)) {
            if (world.isEmpty()) {
                throw new AssertionError("worldCreate returned null");
            }

            assertClose(-9.81, world.gravityY(), "initial gravity y");
            world.set(1.0, 2.0, 3.0);
            assertClose(1.0, world.gravityX(), "gravity x");
            assertClose(2.0, world.gravityY(), "gravity y");
            assertClose(3.0, world.gravityZ(), "gravity z");

            RigidBody.Builder body = world.body(0);
            if (body.isEmpty()) {
                throw new AssertionError("rigidBodyBuilderCreate returned null");
            }

            try {
                world.translation(4.0, 5.0, 6.0);
                world.insert();
                assertClose(4.0, world.translationX(), "body translation x");
                assertClose(5.0, world.translationY(), "body translation y");
                assertClose(6.0, world.translationZ(), "body translation z");
                world.step();
            } finally {
                body.close();
            }
        }

        assertVoxelColliderCanBeCreatedAndInserted();

        long tree = RigidBodyNative.crbTreeCreate();
        if (tree == 0L) {
            throw new AssertionError("crbTreeCreate returned null");
        }
        try {
            if (!RigidBodyNative.crbTreeInsert(tree, 10L, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0)) {
                throw new AssertionError("crbTreeInsert 10 failed");
            }
            if (!RigidBodyNative.crbTreeInsert(tree, 20L, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0)) {
                throw new AssertionError("crbTreeInsert 20 failed");
            }
            int hitCount = RigidBodyNative.crbTreeQueryAabbCount(tree, 0.5, 0.5, 0.5, 2.5, 2.5, 2.5);
            if (hitCount != 2) {
                throw new AssertionError("crbTreeQueryAabbCount expected 2, got " + hitCount);
            }
        } finally {
            RigidBodyNative.crbTreeDestroy(tree);
        }

        System.out.println("JNI smoke test passed on Java " + javaVersion);
    }

    private static void assertVoxelColliderCanBeCreatedAndInserted() throws Exception {
        int sizeX = 16;
        int sizeY = 8;
        int sizeZ = 16;
        byte[] voxels = new byte[sizeX * sizeY * sizeZ];
        fillVoxelBox(voxels, sizeX, sizeY, sizeZ, 4, 0, 4, 12, 4, 12);

        Unsafe unsafe = unsafe();
        long voxelAddress = copyToNative(unsafe, voxels);
        long world = RigidBodyNative.worldCreate(0.0, -9.81, 0.0);
        if (world == 0L) {
            unsafe.freeMemory(voxelAddress);
            throw new AssertionError("worldCreate returned null for voxel test");
        }

        try {
            long builder = RigidBodyNative.colliderBuilderCreateVoxels(
                    voxelAddress,
                    sizeX, sizeY, sizeZ,
                    1.0,
                    0.0, 0.0, 0.0,
                    VOXEL_MODE_GREEDY_CUBOIDS,
                    0,
                    128,
                    20_000);
            if (builder == 0L) {
                throw new AssertionError("colliderBuilderCreateVoxels returned null");
            }

            RigidBodyNative.colliderBuilderSetFriction(builder, 0.8);
            RigidBodyNative.colliderBuilderSetRestitution(builder, 0.1);

            long collider = RigidBodyNative.colliderBuilderBuild(builder);
            if (collider == 0L) {
                throw new AssertionError("colliderBuilderBuild returned null for voxels");
            }

            long colliderHandle = RigidBodyNative.worldInsertCollider(world, collider);
            if (colliderHandle == 0L) {
                throw new AssertionError("worldInsertCollider returned null for voxel collider");
            }
            if (RigidBodyNative.worldGetColliderSetSize(world) != 1) {
                throw new AssertionError("voxel collider was not inserted into world");
            }

            RigidBodyNative.worldStep(world, 1.0 / 60.0);
        } finally {
            RigidBodyNative.worldDestroy(world);
            unsafe.freeMemory(voxelAddress);
        }
    }

    private static void fillVoxelBox(
            byte[] voxels,
            int sizeX, int sizeY, int sizeZ,
            int minX, int minY, int minZ,
            int maxX, int maxY, int maxZ) {
        for (int z = minZ; z < maxZ; z++) {
            for (int y = minY; y < maxY; y++) {
                for (int x = minX; x < maxX; x++) {
                    if (x < 0 || y < 0 || z < 0 || x >= sizeX || y >= sizeY || z >= sizeZ) {
                        continue;
                    }
                    voxels[voxelIndex(x, y, z, sizeX, sizeY)] = 1;
                }
            }
        }
    }

    private static int voxelIndex(int x, int y, int z, int sizeX, int sizeY) {
        return z * sizeX * sizeY + y * sizeX + x;
    }

    private static long copyToNative(Unsafe unsafe, byte[] data) {
        long address = unsafe.allocateMemory(data.length);
        for (int i = 0; i < data.length; i++) {
            unsafe.putByte(address + i, data[i]);
        }
        return address;
    }

    private static Unsafe unsafe() throws Exception {
        Field field = Unsafe.class.getDeclaredField("theUnsafe");
        field.setAccessible(true);
        return (Unsafe) field.get(null);
    }

    private static void assertClose(double expected, double actual, String label) {
        if (Math.abs(expected - actual) > EPSILON) {
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        }
    }
}
