use rapier3d::prelude::ColliderBuilder;

use crate::ffi::{
    RcBool, RcColliderBuilderHandle, RcColliderHandle, RcInteractionGroups, RcQuat,
    RcRigidBodyHandle, RcShapeDesc, RcShapeType, RcVec3, RcWorldHandle, active_events_from_bits,
    active_hooks_from_bits, interaction_groups_to_rapier, isometry_from_parts,
    pack_collider_handle, quat_from_rapier, shape_from_desc, unpack_collider_handle,
    unpack_rigid_body_handle, vec3_from_rapier, vec3_to_rapier,
};

fn default_builder(shape_desc: RcShapeDesc) -> ColliderBuilder {
    ColliderBuilder::new(shape_from_desc(shape_desc))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_create(
    shape_type: RcShapeType,
    shape_data: RcVec3,
) -> *mut RcColliderBuilderHandle {
    let shape_desc = RcShapeDesc {
        shape_type,
        a: shape_data.x,
        b: shape_data.y,
        c: shape_data.z,
        d: 0.0,
    };
    Box::into_raw(Box::new(RcColliderBuilderHandle {
        inner: default_builder(shape_desc),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_create_ex(
    shape_desc: RcShapeDesc,
) -> *mut RcColliderBuilderHandle {
    Box::into_raw(Box::new(RcColliderBuilderHandle {
        inner: default_builder(shape_desc),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_destroy(builder: *mut RcColliderBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_translation(
    builder: *mut RcColliderBuilderHandle,
    translation: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.translation(vec3_to_rapier(translation));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_rotation(
    builder: *mut RcColliderBuilderHandle,
    rotation_axis_angle: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_pose(
    builder: *mut RcColliderBuilderHandle,
    translation: RcVec3,
    rotation: RcQuat,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.position(isometry_from_parts(translation, rotation));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_sensor(
    builder: *mut RcColliderBuilderHandle,
    sensor: RcBool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.sensor(sensor.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_friction(
    builder: *mut RcColliderBuilderHandle,
    friction: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.friction(friction);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_restitution(
    builder: *mut RcColliderBuilderHandle,
    restitution: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.restitution(restitution);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_density(
    builder: *mut RcColliderBuilderHandle,
    density: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.density(density);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_collision_groups(
    builder: *mut RcColliderBuilderHandle,
    groups: RcInteractionGroups,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.collision_groups(interaction_groups_to_rapier(groups));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_solver_groups(
    builder: *mut RcColliderBuilderHandle,
    groups: RcInteractionGroups,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.solver_groups(interaction_groups_to_rapier(groups));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_active_events(
    builder: *mut RcColliderBuilderHandle,
    active_events_bits: u32,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.active_events(active_events_from_bits(active_events_bits));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_active_hooks(
    builder: *mut RcColliderBuilderHandle,
    active_hooks_bits: u32,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.active_hooks(active_hooks_from_bits(active_hooks_bits));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_builder_set_contact_force_event_threshold(
    builder: *mut RcColliderBuilderHandle,
    threshold: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.contact_force_event_threshold(threshold);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_insert_collider(
    world: *mut RcWorldHandle,
    builder: *mut RcColliderBuilderHandle,
) -> RcColliderHandle {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return 0;
    };

    let built = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5)).build();
    pack_collider_handle(world.inner.colliders.insert(built))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_insert_collider_with_parent(
    world: *mut RcWorldHandle,
    builder: *mut RcColliderBuilderHandle,
    parent: RcRigidBodyHandle,
) -> RcColliderHandle {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return 0;
    };

    let built = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5)).build();
    pack_collider_handle(world.inner.colliders.insert_with_parent(
        built,
        unpack_rigid_body_handle(parent),
        &mut world.inner.bodies,
    ))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_remove_collider(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };

    world
        .inner
        .colliders
        .remove(
            unpack_collider_handle(handle),
            &mut world.inner.islands,
            &mut world.inner.bodies,
            wake_up.0 != 0,
        )
        .is_some()
        .into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_remove_collider_flag(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    wake_up: RcBool,
) -> u8 {
    rc_world_remove_collider(world, handle, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_get_translation(
    world: *const RcWorldHandle,
    handle: RcColliderHandle,
) -> RcVec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcVec3::default();
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| vec3_from_rapier(collider.translation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_get_rotation(
    world: *const RcWorldHandle,
    handle: RcColliderHandle,
) -> RcQuat {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcQuat::default();
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| quat_from_rapier(collider.rotation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_pose(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    translation: RcVec3,
    rotation: RcQuat,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_position(isometry_from_parts(translation, rotation));
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_sensor(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    sensor: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_sensor(sensor.0 != 0);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_friction(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    friction: f64,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_friction(friction);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_restitution(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    restitution: f64,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_restitution(restitution);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_collision_groups(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    groups: RcInteractionGroups,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_collision_groups(interaction_groups_to_rapier(groups));
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_solver_groups(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    groups: RcInteractionGroups,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_solver_groups(interaction_groups_to_rapier(groups));
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_active_events(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    active_events_bits: u32,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_active_events(active_events_from_bits(active_events_bits));
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_active_hooks(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    active_hooks_bits: u32,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_active_hooks(active_hooks_from_bits(active_hooks_bits));
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_set_contact_force_event_threshold(
    world: *mut RcWorldHandle,
    handle: RcColliderHandle,
    threshold: f64,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return RcBool::FALSE;
    };

    collider.set_contact_force_event_threshold(threshold);
    RcBool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_collider_get_density(
    world: *const RcWorldHandle,
    handle: RcColliderHandle,
) -> f64 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0.0;
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| collider.density())
        .unwrap_or(0.0)
}
