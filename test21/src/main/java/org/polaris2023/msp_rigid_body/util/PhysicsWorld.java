package org.polaris2023.msp_rigid_body.util;

import org.polaris2023.msp_rigid_body.RigidBodyNative;

public final class PhysicsWorld implements AutoCloseable {
    private long handle;
    private double deltaSeconds = 1.0 / 60.0;
    private RigidBody.Builder builder;
    private RigidBody rigidBody;

    public PhysicsWorld(double gravityX, double gravityY, double gravityZ) {
        handle = RigidBodyNative.worldCreate(gravityX, gravityY, gravityZ);
    }

    public boolean isEmpty() {
        return handle == 0L;
    }

    public long handle() {
        requireOpen();
        return handle;
    }

    public int colliderCount() {
        requireOpen();
        return RigidBodyNative.worldGetColliderSetSize(handle);
    }

    public int rigidBodyCount() {
        requireOpen();
        return RigidBodyNative.worldGetRigidBodySetSize(handle);
    }

    public PhysicsWorld set(double gravityX, double gravityY, double gravityZ) {
        requireOpen();
        RigidBodyNative.worldSetGravity(handle, gravityX, gravityY, gravityZ);
        return this;
    }

    public double[] gravity() {
        requireOpen();
        return RigidBodyNative.worldGetGravity(handle);
    }

    public double gravityX() {
        return gravity()[0];
    }

    public double gravityY() {
        return gravity()[1];
    }

    public double gravityZ() {
        return gravity()[2];
    }

    public PhysicsWorld deltaSeconds(double deltaSeconds) {
        requireOpen();
        this.deltaSeconds = deltaSeconds;
        return this;
    }

    public PhysicsWorld step() {
        requireOpen();
        RigidBodyNative.worldStep(handle, deltaSeconds);
        return this;
    }

    public RigidBody.Builder body() {
        return body(0);
    }

    public RigidBody.Builder body(int status) {
        requireOpen();
        builder = RigidBody.Builder.builder(this).status(status).build();
        return builder;
    }

    public RigidBody insert(RigidBody.Builder builder) {
        requireOpen();
        if (builder == null || builder.isEmpty()) {
            throw new IllegalArgumentException("rigid body builder is empty");
        }
        return builder.body(this);
    }

    public Collider.Builder cuboidCollider(double hx, double hy, double hz) {
        requireOpen();
        return Collider.Builder.cuboid(this, hx, hy, hz);
    }

    public Collider.Builder sphereCollider(double x, double y, double z, double radius) {
        requireOpen();
        return Collider.Builder.sphere(this, x, y, z, radius);
    }

    public Collider.Builder capsuleCollider(double ax, double ay, double az, double bx, double by, double bz, double radius) {
        requireOpen();
        return Collider.Builder.capsule(this, ax, ay, az, bx, by, bz, radius);
    }

    public Collider.Builder cylinderCollider(double x, double y, double z, double radius, double halfHeight) {
        requireOpen();
        return Collider.Builder.cylinder(this, x, y, z, radius, halfHeight);
    }

    public PhysicsWorld translation(double x, double y, double z) {
        if (builder == null) {
            throw new IllegalStateException("body() must be called before translation()");
        }
        builder.translation(x, y, z);
        return this;
    }

    public Collider.Builder voxelCollider(
            long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
            double originX, double originY, double originZ,
            int mode, boolean dynamicBody, int smallVoxelLimit, int meshVoxelLimit) {
        requireOpen();
        return Collider.Builder.voxels(this,
                voxels, sizeX, sizeY, sizeZ, voxelSize,
                originX, originY, originZ,
                mode, dynamicBody, smallVoxelLimit, meshVoxelLimit);
    }

    public Collider insert(Collider.Raw raw) {
        requireOpen();
        if (raw == null || raw.isEmpty()) {
            throw new IllegalArgumentException("raw collider is empty");
        }
        long collider = RigidBodyNative.worldInsertCollider(handle, raw.release());
        return new Collider(this, collider);
    }

    public Collider insert(Collider.Raw raw, RigidBody parent) {
        requireOpen();
        if (raw == null || raw.isEmpty()) {
            throw new IllegalArgumentException("raw collider is empty");
        }
        long collider = RigidBodyNative.worldInsertColliderWithParent(handle, raw.release(), parent.handle());
        return new Collider(this, collider);
    }

    public Joint.Builder fixedJoint() {
        requireOpen();
        return Joint.Builder.fixed(this);
    }

    public Joint.Builder revoluteJoint(double ax, double ay, double az) {
        requireOpen();
        return Joint.Builder.revolute(this, ax, ay, az);
    }

    public Query query() {
        requireOpen();
        return new Query(this);
    }

    public void clearEvents() {
        requireOpen();
        RigidBodyNative.worldClearEvents(handle);
    }

    public int collisionEventCount() {
        requireOpen();
        return RigidBodyNative.worldCollisionEventCount(handle);
    }

    public CollisionEvent collisionEvent(int index) {
        requireOpen();
        try (NativeMemory out = new NativeMemory(32)) {
            RigidBodyNative.worldGetCollisionEvent(handle, index, out.address());
            return new CollisionEvent(
                    out.getBool(0),
                    out.getLong(8),
                    out.getLong(16),
                    out.getBool(24),
                    out.getBool(25));
        }
    }

    public int contactForceEventCount() {
        requireOpen();
        return RigidBodyNative.worldContactForceEventCount(handle);
    }

    public ContactForceEvent contactForceEvent(int index) {
        requireOpen();
        try (NativeMemory out = new NativeMemory(80)) {
            RigidBodyNative.worldGetContactForceEvent(handle, index, out.address());
            return new ContactForceEvent(
                    out.getLong(0),
                    out.getLong(8),
                    out.getVec3(16),
                    out.getDouble(40),
                    out.getVec3(48),
                    out.getDouble(72));
        }
    }

    public PhysicsWorld insert() {
        if (builder == null) {
            throw new IllegalStateException("body() must be called before insert()");
        }
        rigidBody = builder.body(this);
        return this;
    }

    public double[] translation() {
        if (rigidBody == null) {
            throw new IllegalStateException("insert() must be called before translation()");
        }
        return rigidBody.translation(this);
    }

    public double translationX() {
        return translation()[0];
    }

    public double translationY() {
        return translation()[1];
    }

    public double translationZ() {
        return translation()[2];
    }

    @Override
    public void close() {
        if (builder != null) {
            builder.close();
            builder = null;
        }
        if (handle != 0L) {
            RigidBodyNative.worldDestroy(handle);
            handle = 0L;
        }
    }

    private void requireOpen() {
        if (handle == 0L) {
            throw new IllegalStateException("world is closed");
        }
    }

    public record CollisionEvent(boolean started, long collider1, long collider2, boolean sensor, boolean removed) {
    }

    public record ContactForceEvent(
            long collider1,
            long collider2,
            double[] totalForce,
            double totalForceMagnitude,
            double[] maxForceDirection,
            double maxForceMagnitude) {
    }
}
