use rapier3d::prelude::RigidBodyBuilder;

use crate::ffi::{
    RcBodyStatus, RcBool, RcQuat, RcRigidBodyBuilderHandle, RcRigidBodyHandle, RcVec3,
    RcWorldHandle, body_status_from_rapier, body_status_to_rapier, isometry_from_parts,
    pack_rigid_body_handle, quat_from_rapier, unpack_rigid_body_handle, vec3_from_rapier,
    vec3_to_rapier,
};

fn builder_from_status(status: RcBodyStatus) -> RigidBodyBuilder {
    match status {
        RcBodyStatus::Dynamic => RigidBodyBuilder::dynamic(),
        RcBodyStatus::Fixed => RigidBodyBuilder::fixed(),
        RcBodyStatus::KinematicPositionBased => RigidBodyBuilder::kinematic_position_based(),
        RcBodyStatus::KinematicVelocityBased => RigidBodyBuilder::kinematic_velocity_based(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_create(
    status: RcBodyStatus,
) -> *mut RcRigidBodyBuilderHandle {
    Box::into_raw(Box::new(RcRigidBodyBuilderHandle {
        inner: builder_from_status(status),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_destroy(builder: *mut RcRigidBodyBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_translation(
    builder: *mut RcRigidBodyBuilderHandle,
    translation: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.translation(vec3_to_rapier(translation));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_rotation(
    builder: *mut RcRigidBodyBuilderHandle,
    rotation_axis_angle: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_pose(
    builder: *mut RcRigidBodyBuilderHandle,
    translation: RcVec3,
    rotation: RcQuat,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.pose(isometry_from_parts(translation, rotation));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_linvel(
    builder: *mut RcRigidBodyBuilderHandle,
    linvel: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.linvel(vec3_to_rapier(linvel));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_angvel(
    builder: *mut RcRigidBodyBuilderHandle,
    angvel: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.angvel(vec3_to_rapier(angvel));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_gravity_scale(
    builder: *mut RcRigidBodyBuilderHandle,
    gravity_scale: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.gravity_scale(gravity_scale);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_linear_damping(
    builder: *mut RcRigidBodyBuilderHandle,
    linear_damping: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.linear_damping(linear_damping);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_angular_damping(
    builder: *mut RcRigidBodyBuilderHandle,
    angular_damping: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.angular_damping(angular_damping);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_can_sleep(
    builder: *mut RcRigidBodyBuilderHandle,
    can_sleep: RcBool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.can_sleep(can_sleep.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_enabled_rotations(
    builder: *mut RcRigidBodyBuilderHandle,
    allow_x: RcBool,
    allow_y: RcBool,
    allow_z: RcBool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.enabled_rotations(allow_x.0 != 0, allow_y.0 != 0, allow_z.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_user_data(
    builder: *mut RcRigidBodyBuilderHandle,
    user_data_low: u64,
    user_data_high: u64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let user_data = (user_data_low as u128) | ((user_data_high as u128) << 64);
    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.user_data(user_data);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_builder_set_additional_mass(
    builder: *mut RcRigidBodyBuilderHandle,
    mass: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.additional_mass(mass);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_insert_rigid_body(
    world: *mut RcWorldHandle,
    builder: *mut RcRigidBodyBuilderHandle,
) -> RcRigidBodyHandle {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return 0;
    };

    let built = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic()).build();
    pack_rigid_body_handle(world.inner.bodies.insert(built))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_remove_rigid_body(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    remove_attached_colliders: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };

    world
        .inner
        .bodies
        .remove(
            unpack_rigid_body_handle(handle),
            &mut world.inner.islands,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
            remove_attached_colliders.0 != 0,
        )
        .is_some()
        .into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_remove_rigid_body_flag(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    remove_attached_colliders: RcBool,
) -> u8 {
    rc_world_remove_rigid_body(world, handle, remove_attached_colliders).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_status(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcBodyStatus {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcBodyStatus::Fixed;
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| body_status_from_rapier(body.body_type()))
        .unwrap_or(RcBodyStatus::Fixed)
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_set_status(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    status: RcBodyStatus,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };

    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.set_body_type(body_status_to_rapier(status), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_translation(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcVec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcVec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.translation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_translation_out(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
    out_translation: *mut RcVec3,
) {
    let Some(out_translation) = (unsafe { out_translation.as_mut() }) else {
        return;
    };

    *out_translation = rc_rigid_body_get_translation(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_rotation(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcQuat {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcQuat::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| quat_from_rapier(*body.rotation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_rotation_out(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
    out_rotation: *mut RcQuat,
) {
    let Some(out_rotation) = (unsafe { out_rotation.as_mut() }) else {
        return;
    };

    *out_rotation = rc_rigid_body_get_rotation(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_set_pose(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    translation: RcVec3,
    rotation: RcQuat,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.set_position(isometry_from_parts(translation, rotation), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_set_pose_flag(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    translation: RcVec3,
    rotation: RcQuat,
    wake_up: RcBool,
) -> u8 {
    rc_rigid_body_set_pose(world, handle, translation, rotation, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_linvel(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcVec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcVec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.linvel()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_linvel_out(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
    out_linvel: *mut RcVec3,
) {
    let Some(out_linvel) = (unsafe { out_linvel.as_mut() }) else {
        return;
    };

    *out_linvel = rc_rigid_body_get_linvel(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_set_linvel(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    linvel: RcVec3,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.set_linvel(vec3_to_rapier(linvel), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_set_linvel_flag(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    linvel: RcVec3,
    wake_up: RcBool,
) -> u8 {
    rc_rigid_body_set_linvel(world, handle, linvel, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_get_angvel(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcVec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcVec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.angvel()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_set_angvel(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    angvel: RcVec3,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.set_angvel(vec3_to_rapier(angvel), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_add_force(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    force: RcVec3,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.add_force(vec3_to_rapier(force), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_add_force_flag(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    force: RcVec3,
    wake_up: RcBool,
) -> u8 {
    rc_rigid_body_add_force(world, handle, force, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_add_torque(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    torque: RcVec3,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.add_torque(vec3_to_rapier(torque), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_apply_impulse(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    impulse: RcVec3,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.apply_impulse(vec3_to_rapier(impulse), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_apply_torque_impulse(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    torque_impulse: RcVec3,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.apply_torque_impulse(vec3_to_rapier(torque_impulse), wake_up.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_enable_ccd(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    enabled: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.enable_ccd(enabled.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_sleep(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.sleep();
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_sleep_flag(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> u8 {
    rc_rigid_body_sleep(world, handle).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_wake_up(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    strong: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return RcBool::FALSE;
    };

    body.wake_up(strong.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_wake_up_flag(
    world: *mut RcWorldHandle,
    handle: RcRigidBodyHandle,
    strong: RcBool,
) -> u8 {
    rc_rigid_body_wake_up(world, handle, strong).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_is_sleeping(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> RcBool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcBool::FALSE;
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| body.is_sleeping().into())
        .unwrap_or(RcBool::FALSE)
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_rigid_body_is_sleeping_flag(
    world: *const RcWorldHandle,
    handle: RcRigidBodyHandle,
) -> u8 {
    rc_rigid_body_is_sleeping(world, handle).0
}
