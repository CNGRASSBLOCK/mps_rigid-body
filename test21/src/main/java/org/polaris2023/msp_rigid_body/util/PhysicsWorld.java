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
        return handle;
    }

    public PhysicsWorld set(double gravityX, double gravityY, double gravityZ) {
        RigidBodyNative.worldSetGravity(handle, gravityX, gravityY, gravityZ);
        return this;
    }

    public double[] gravity() {
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
        this.deltaSeconds = deltaSeconds;
        return this;
    }

    public PhysicsWorld step() {
        RigidBodyNative.worldStep(handle, deltaSeconds);
        return this;
    }

    public RigidBody.Builder body() {
        return body(0);
    }

    public RigidBody.Builder body(int status) {
        builder = RigidBody.Builder.builder(this).status(status).build();
        return builder;
    }

    public PhysicsWorld translation(double x, double y, double z) {
        if (builder == null) {
            throw new IllegalStateException("body() must be called before translation()");
        }
        builder.translation(x, y, z);
        return this;
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
}
