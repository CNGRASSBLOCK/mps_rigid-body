use rapier3d::prelude::ColliderBuilder;
use std::slice;

use crate::ffi::{
    Bool, ColliderBuilderHandle, ColliderHandleRaw, InteractionGroupsDesc, Obb, Quat,
    RigidBodyHandleRaw, ShapeDesc, ShapeType, Sphere, Vec3, WorldHandle, active_events_from_bits,
    active_hooks_from_bits, interaction_groups_to_rapier, isometry_from_parts,
    pack_collider_handle, quat_from_rapier, shape_from_desc, unpack_collider_handle,
    unpack_rigid_body_handle, vec3_from_rapier, vec3_to_rapier,
};

fn default_builder(shape_desc: ShapeDesc) -> ColliderBuilder {
    ColliderBuilder::new(shape_from_desc(shape_desc))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create(
    shape_type: ShapeType,
    shape_data: Vec3,
) -> *mut ColliderBuilderHandle {
    let shape_desc = ShapeDesc {
        shape_type,
        a: shape_data.x,
        b: shape_data.y,
        c: shape_data.z,
        d: 0.0,
    };
    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: default_builder(shape_desc),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_ex(shape_desc: ShapeDesc) -> *mut ColliderBuilderHandle {
    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: default_builder(shape_desc),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_obb(obb: Obb) -> *mut ColliderBuilderHandle {
    if obb.half_extents.x <= 0.0 || obb.half_extents.y <= 0.0 || obb.half_extents.z <= 0.0 {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::cuboid(obb.half_extents.x, obb.half_extents.y, obb.half_extents.z)
            .position(isometry_from_parts(obb.center, obb.rotation)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_sphere(sphere: Sphere) -> *mut ColliderBuilderHandle {
    if sphere.radius <= 0.0 {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::ball(sphere.radius).translation(vec3_to_rapier(sphere.center)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_convex_hull(
    points_xyz: *const f64,
    point_count: u32,
) -> *mut ColliderBuilderHandle {
    if points_xyz.is_null() || point_count < 4 {
        return std::ptr::null_mut();
    }

    let Some(value_count) = (point_count as usize).checked_mul(3) else {
        return std::ptr::null_mut();
    };
    let values = unsafe { slice::from_raw_parts(points_xyz, value_count) };
    let points: Vec<_> = values
        .chunks_exact(3)
        .map(|chunk| {
            vec3_to_rapier(Vec3 {
                x: chunk[0],
                y: chunk[1],
                z: chunk[2],
            })
        })
        .collect();

    let Some(builder) = ColliderBuilder::convex_hull(&points) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_destroy(builder: *mut ColliderBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_translation(
    builder: *mut ColliderBuilderHandle,
    translation: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.translation(vec3_to_rapier(translation));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_rotation(
    builder: *mut ColliderBuilderHandle,
    rotation_axis_angle: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_pose(
    builder: *mut ColliderBuilderHandle,
    translation: Vec3,
    rotation: Quat,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.position(isometry_from_parts(translation, rotation));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_sensor(builder: *mut ColliderBuilderHandle, sensor: Bool) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.sensor(sensor.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_friction(
    builder: *mut ColliderBuilderHandle,
    friction: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.friction(friction);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_restitution(
    builder: *mut ColliderBuilderHandle,
    restitution: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.restitution(restitution);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_density(builder: *mut ColliderBuilderHandle, density: f64) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.density(density);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_collision_groups(
    builder: *mut ColliderBuilderHandle,
    groups: InteractionGroupsDesc,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.collision_groups(interaction_groups_to_rapier(groups));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_solver_groups(
    builder: *mut ColliderBuilderHandle,
    groups: InteractionGroupsDesc,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.solver_groups(interaction_groups_to_rapier(groups));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_active_events(
    builder: *mut ColliderBuilderHandle,
    active_events_bits: u32,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.active_events(active_events_from_bits(active_events_bits));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_active_hooks(
    builder: *mut ColliderBuilderHandle,
    active_hooks_bits: u32,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.active_hooks(active_hooks_from_bits(active_hooks_bits));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_contact_force_event_threshold(
    builder: *mut ColliderBuilderHandle,
    threshold: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.contact_force_event_threshold(threshold);
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_collider(
    world: *mut WorldHandle,
    builder: *mut ColliderBuilderHandle,
) -> ColliderHandleRaw {
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
pub extern "C" fn world_insert_collider_with_parent(
    world: *mut WorldHandle,
    builder: *mut ColliderBuilderHandle,
    parent: RigidBodyHandleRaw,
) -> ColliderHandleRaw {
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
pub extern "C" fn world_remove_collider(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
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
pub extern "C" fn world_remove_collider_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    wake_up: Bool,
) -> u8 {
    world_remove_collider(world, handle, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_translation(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| vec3_from_rapier(collider.translation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_rotation(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> Quat {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Quat::default();
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| quat_from_rapier(collider.rotation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_pose(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
    rotation: Quat,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_position(isometry_from_parts(translation, rotation));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_sensor(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    sensor: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_sensor(sensor.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_friction(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    friction: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_friction(friction);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_restitution(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    restitution: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_restitution(restitution);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_collision_groups(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_collision_groups(interaction_groups_to_rapier(groups));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_solver_groups(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_solver_groups(interaction_groups_to_rapier(groups));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_events(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_events_bits: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_active_events(active_events_from_bits(active_events_bits));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_hooks(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_hooks_bits: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_active_hooks(active_hooks_from_bits(active_hooks_bits));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_contact_force_event_threshold(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    threshold: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_contact_force_event_threshold(threshold);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_density(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convex_hull_builder_accepts_cube_points() {
        let points = [
            -1.0, -1.0, -1.0, //
            -1.0, -1.0, 1.0, //
            -1.0, 1.0, -1.0, //
            -1.0, 1.0, 1.0, //
            1.0, -1.0, -1.0, //
            1.0, -1.0, 1.0, //
            1.0, 1.0, -1.0, //
            1.0, 1.0, 1.0,
        ];

        let builder = collider_builder_create_convex_hull(points.as_ptr(), 8);
        assert!(!builder.is_null());
        collider_builder_destroy(builder);
    }
}
