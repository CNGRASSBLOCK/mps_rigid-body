use std::ffi::c_void;

use crate::ffi::{BodyStatus, RigidBodyBuilderHandle, RigidBodyHandleRaw, Vec3, WorldHandle};

type JNIEnv = *mut c_void;
type JClass = *mut c_void;
type JDouble = f64;
type JInt = i32;
type JLong = i64;

fn ptr_to_jlong<T>(value: *mut T) -> JLong {
    value as isize as JLong
}

fn jlong_to_mut<T>(value: JLong) -> *mut T {
    value as isize as *mut T
}

fn jlong_to_const<T>(value: JLong) -> *const T {
    value as isize as *const T
}

fn vec3(x: JDouble, y: JDouble, z: JDouble) -> Vec3 {
    Vec3 { x, y, z }
}

fn body_status(value: JInt) -> BodyStatus {
    match value {
        0 => BodyStatus::Dynamic,
        1 => BodyStatus::Fixed,
        2 => BodyStatus::KinematicPositionBased,
        3 => BodyStatus::KinematicVelocityBased,
        _ => BodyStatus::Fixed,
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_abiVersion(
    _env: JNIEnv,
    _class: JClass,
) -> JInt {
    crate::abi::ffm::abi_version() as JInt
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldCreate(
    _env: JNIEnv,
    _class: JClass,
    gravity_x: JDouble,
    gravity_y: JDouble,
    gravity_z: JDouble,
) -> JLong {
    ptr_to_jlong(crate::world::world_create(vec3(
        gravity_x, gravity_y, gravity_z,
    )))
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldDestroy(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
) {
    crate::world::world_destroy(jlong_to_mut::<WorldHandle>(world));
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldStep(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
    delta_seconds: JDouble,
) {
    crate::world::world_step(jlong_to_mut::<WorldHandle>(world), delta_seconds);
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldSetGravity(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
    gravity_x: JDouble,
    gravity_y: JDouble,
    gravity_z: JDouble,
) {
    crate::world::world_set_gravity(
        jlong_to_mut::<WorldHandle>(world),
        vec3(gravity_x, gravity_y, gravity_z),
    );
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetGravityX(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
) -> JDouble {
    crate::world::world_get_gravity(jlong_to_const::<WorldHandle>(world)).x
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetGravityY(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
) -> JDouble {
    crate::world::world_get_gravity(jlong_to_const::<WorldHandle>(world)).y
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetGravityZ(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
) -> JDouble {
    crate::world::world_get_gravity(jlong_to_const::<WorldHandle>(world)).z
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldDynamicBodySnapshotCount(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
) -> JInt {
    crate::world::world_dynamic_body_snapshot_count(jlong_to_const::<WorldHandle>(world)) as JInt
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderCreate(
    _env: JNIEnv,
    _class: JClass,
    status: JInt,
) -> JLong {
    ptr_to_jlong(crate::rigid_body::rigid_body_builder_create(body_status(
        status,
    )))
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderDestroy(
    _env: JNIEnv,
    _class: JClass,
    builder: JLong,
) {
    crate::rigid_body::rigid_body_builder_destroy(jlong_to_mut::<RigidBodyBuilderHandle>(builder));
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetTranslation(
    _env: JNIEnv,
    _class: JClass,
    builder: JLong,
    x: JDouble,
    y: JDouble,
    z: JDouble,
) {
    crate::rigid_body::rigid_body_builder_set_translation(
        jlong_to_mut::<RigidBodyBuilderHandle>(builder),
        vec3(x, y, z),
    );
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertRigidBody(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
    builder: JLong,
) -> JLong {
    crate::rigid_body::world_insert_rigid_body(
        jlong_to_mut::<WorldHandle>(world),
        jlong_to_mut::<RigidBodyBuilderHandle>(builder),
    ) as JLong
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetTranslationX(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
    body: JLong,
) -> JDouble {
    crate::rigid_body::rigid_body_get_translation(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
    .x
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetTranslationY(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
    body: JLong,
) -> JDouble {
    crate::rigid_body::rigid_body_get_translation(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
    .y
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetTranslationZ(
    _env: JNIEnv,
    _class: JClass,
    world: JLong,
    body: JLong,
) -> JDouble {
    crate::rigid_body::rigid_body_get_translation(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
    .z
}
