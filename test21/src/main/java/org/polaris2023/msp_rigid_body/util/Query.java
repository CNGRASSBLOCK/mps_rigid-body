package org.polaris2023.msp_rigid_body.util;

import org.polaris2023.msp_rigid_body.RigidBodyNative;

import java.util.Arrays;

public final class Query {
    public static final int SHAPE_BALL = 0;
    public static final int SHAPE_CUBOID = 1;

    private final PhysicsWorld world;

    Query(PhysicsWorld world) {
        this.world = world;
    }

    public RayHit castRay(
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            double maxToi) {
        try (NativeMemory out = new NativeMemory(48)) {
            long collider = RigidBodyNative.queryCastRay(
                    world.handle(),
                    ox, oy, oz,
                    dx, dy, dz,
                    maxToi,
                    1,
                    0, 0xffff, 0xffff, 0,
                    0L, 0, 0L, 0,
                    out.address());
            if (collider == 0L) {
                return RayHit.empty();
            }
            return new RayHit(
                    collider,
                    out.getDouble(8),
                    out.getVec3(16),
                    out.getInt(40));
        }
    }

    public int countAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.queryIntersectAabbCount(
                world.handle(),
                minX, minY, minZ,
                maxX, maxY, maxZ,
                0, 0xffff, 0xffff, 0,
                0L, 0, 0L, 0);
    }

    public long[] intersectSphere(double x, double y, double z, double radius, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        try (NativeMemory out = NativeMemory.longs(capacity)) {
            int written = RigidBodyNative.queryIntersectSphere(
                    world.handle(),
                    x, y, z, radius,
                    0, 0xffff, 0xffff, 0,
                    0L, 0, 0L, 0,
                    out.address(), capacity);
            return Arrays.copyOf(out.getLongs(capacity), Math.max(0, Math.min(written, capacity)));
        }
    }

    public long[] intersectObb(
            double cx, double cy, double cz,
            double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        try (NativeMemory out = NativeMemory.longs(capacity)) {
            int written = RigidBodyNative.queryIntersectObb(
                    world.handle(),
                    cx, cy, cz,
                    hx, hy, hz,
                    qi, qj, qk, qw,
                    0, 0xffff, 0xffff, 0,
                    0L, 0, 0L, 0,
                    out.address(), capacity);
            return Arrays.copyOf(out.getLongs(capacity), Math.max(0, Math.min(written, capacity)));
        }
    }

    public ShapeCastHit castShape(
            int shapeType, double a, double b, double c, double d,
            double tx, double ty, double tz,
            double qi, double qj, double qk, double qw,
            double vx, double vy, double vz,
            double maxToi) {
        try (NativeMemory out = new NativeMemory(120)) {
            long collider = RigidBodyNative.queryCastShape(
                    world.handle(),
                    shapeType, a, b, c, d,
                    tx, ty, tz,
                    qi, qj, qk, qw,
                    vx, vy, vz,
                    maxToi,
                    0.0,
                    1, 1,
                    0, 0xffff, 0xffff, 0,
                    0L, 0, 0L, 0,
                    out.address());
            if (collider == 0L) {
                return ShapeCastHit.empty();
            }
            return new ShapeCastHit(
                    collider,
                    out.getDouble(8),
                    out.getVec3(16),
                    out.getVec3(40),
                    out.getVec3(64),
                    out.getVec3(88),
                    out.getInt(112));
        }
    }

    public record RayHit(long collider, double timeOfImpact, double[] normal, int feature) {
        static RayHit empty() {
            return new RayHit(0L, 0.0, new double[] {0.0, 0.0, 0.0}, 0);
        }

        public boolean isEmpty() {
            return collider == 0L;
        }
    }

    public record ShapeCastHit(
            long collider,
            double timeOfImpact,
            double[] witness1,
            double[] witness2,
            double[] normal1,
            double[] normal2,
            int status) {
        static ShapeCastHit empty() {
            double[] zero = {0.0, 0.0, 0.0};
            return new ShapeCastHit(0L, 0.0, zero, zero, zero, zero, 0);
        }

        public boolean isEmpty() {
            return collider == 0L;
        }
    }
}
