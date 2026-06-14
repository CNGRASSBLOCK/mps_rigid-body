package org.polaris2023.msp_rigid_body.util;

import org.polaris2023.msp_rigid_body.RigidBodyNative;

public final class Collider {
    private final PhysicsWorld world;
    private long handle;

    Collider(PhysicsWorld world, long handle) {
        this.world = world;
        this.handle = handle;
    }

    public boolean isEmpty() {
        return handle == 0L;
    }

    public long handle() {
        return handle;
    }

    public PhysicsWorld world() {
        return world;
    }

    public double density() {
        return RigidBodyNative.colliderGetDensity(world.handle(), handle);
    }

    public double[] translation() {
        requirePresent();
        return RigidBodyNative.colliderGetTranslation(world.handle(), handle);
    }

    public double[] rotation() {
        requirePresent();
        return RigidBodyNative.colliderGetRotation(world.handle(), handle);
    }

    public Collider pose(double x, double y, double z, double qi, double qj, double qk, double qw) {
        requirePresent();
        RigidBodyNative.colliderSetPose(world.handle(), handle, x, y, z, qi, qj, qk, qw);
        return this;
    }

    public Collider sensor(boolean sensor) {
        requirePresent();
        RigidBodyNative.colliderSetSensor(world.handle(), handle, sensor ? 1 : 0);
        return this;
    }

    public Collider friction(double friction) {
        requirePresent();
        RigidBodyNative.colliderSetFriction(world.handle(), handle, friction);
        return this;
    }

    public Collider restitution(double restitution) {
        requirePresent();
        RigidBodyNative.colliderSetRestitution(world.handle(), handle, restitution);
        return this;
    }

    public boolean remove(boolean wakeUp) {
        requirePresent();
        boolean removed = RigidBodyNative.worldRemoveCollider(world.handle(), handle, wakeUp ? 1 : 0);
        if (removed) {
            handle = 0L;
        }
        return removed;
    }

    private void requirePresent() {
        if (handle == 0L) {
            throw new IllegalStateException("collider is empty");
        }
    }

    public static final class Builder implements AutoCloseable, IParent<PhysicsWorld> {
        private final PhysicsWorld parent;
        private long handle;

        private Builder(PhysicsWorld parent, long handle) {
            this.parent = parent;
            this.handle = handle;
        }

        public static Builder voxels(
                PhysicsWorld parent,
                long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
                double originX, double originY, double originZ,
                int mode, boolean dynamicBody, int smallVoxelLimit, int meshVoxelLimit) {
            long handle = RigidBodyNative.colliderBuilderCreateVoxels(
                    voxels, sizeX, sizeY, sizeZ, voxelSize,
                    originX, originY, originZ,
                    mode, dynamicBody ? 1 : 0, smallVoxelLimit, meshVoxelLimit);
            return new Builder(parent, handle);
        }

        public static Builder cuboid(PhysicsWorld parent, double hx, double hy, double hz) {
            return new Builder(parent, RigidBodyNative.colliderBuilderCreate(Query.SHAPE_CUBOID, hx, hy, hz));
        }

        public static Builder sphere(PhysicsWorld parent, double x, double y, double z, double radius) {
            return new Builder(parent, RigidBodyNative.colliderBuilderCreateSphere(x, y, z, radius));
        }

        public static Builder capsule(PhysicsWorld parent, double ax, double ay, double az, double bx, double by, double bz, double radius) {
            return new Builder(parent, RigidBodyNative.colliderBuilderCreateCapsule(ax, ay, az, bx, by, bz, radius));
        }

        public static Builder cylinder(PhysicsWorld parent, double x, double y, double z, double radius, double halfHeight) {
            return new Builder(parent, RigidBodyNative.colliderBuilderCreateCylinder(x, y, z, radius, halfHeight, 0.0, 0.0, 0.0, 1.0));
        }

        public boolean isEmpty() {
            return handle == 0L;
        }

        public Builder friction(double friction) {
            requireOpen();
            RigidBodyNative.colliderBuilderSetFriction(handle, friction);
            return this;
        }

        public Builder restitution(double restitution) {
            requireOpen();
            RigidBodyNative.colliderBuilderSetRestitution(handle, restitution);
            return this;
        }

        public Builder density(double density) {
            requireOpen();
            RigidBodyNative.colliderBuilderSetDensity(handle, density);
            return this;
        }

        public Builder sensor(boolean sensor) {
            requireOpen();
            RigidBodyNative.colliderBuilderSetSensor(handle, sensor ? 1 : 0);
            return this;
        }

        public Builder translation(double x, double y, double z) {
            requireOpen();
            RigidBodyNative.colliderBuilderSetTranslation(handle, x, y, z);
            return this;
        }

        public Raw buildRaw() {
            requireOpen();
            long raw = RigidBodyNative.colliderBuilderBuild(handle);
            handle = 0L;
            return new Raw(raw);
        }

        public Collider insert() {
            try (Raw raw = buildRaw()) {
                return parent.insert(raw);
            }
        }

        @Override
        public void close() {
            if (handle != 0L) {
                RigidBodyNative.colliderBuilderDestroy(handle);
                handle = 0L;
            }
        }

        @Override
        public PhysicsWorld parent() {
            return parent;
        }

        private void requireOpen() {
            if (handle == 0L) {
                throw new IllegalStateException("collider builder is closed");
            }
        }
    }

    public static final class Raw implements AutoCloseable {
        private long handle;

        private Raw(long handle) {
            this.handle = handle;
        }

        public boolean isEmpty() {
            return handle == 0L;
        }

        long release() {
            long value = handle;
            handle = 0L;
            return value;
        }

        @Override
        public void close() {
            if (handle != 0L) {
                RigidBodyNative.colliderDestroyRaw(handle);
                handle = 0L;
            }
        }
    }
}
