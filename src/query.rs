use rapier3d::geometry::{Aabb, Ray};
use rapier3d::parry::shape::FeatureId;

use crate::ffi::{
    RcAabb, RcBool, RcColliderHandle, RcPointProjection, RcQueryFilterDesc, RcRayHit,
    RcShapeCastHit, RcShapeCastOptions, RcShapeDesc, RcVec3, RcWorldHandle, pack_collider_handle,
    query_filter_from_desc, shape_cast_options_to_rapier, shape_from_desc, vec3_from_rapier,
    vec3_to_rapier,
};

fn aabb_to_rapier(aabb: RcAabb) -> Aabb {
    Aabb::new(vec3_to_rapier(aabb.mins), vec3_to_rapier(aabb.maxs))
}

fn feature_id_to_u32(feature: FeatureId) -> u32 {
    match feature {
        FeatureId::Unknown => 0,
        FeatureId::Vertex(id) => 0x1000_0000 | id,
        FeatureId::Edge(id) => 0x2000_0000 | id,
        FeatureId::Face(id) => 0x3000_0000 | id,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_cast_ray(
    world: *const RcWorldHandle,
    origin: RcVec3,
    direction: RcVec3,
    max_toi: f64,
    solid: RcBool,
    filter: RcQueryFilterDesc,
) -> RcRayHit {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcRayHit::default();
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );
    let ray = Ray::new(vec3_to_rapier(origin), vec3_to_rapier(direction));

    query
        .cast_ray_and_get_normal(&ray, max_toi, solid.0 != 0)
        .map(|(handle, hit)| RcRayHit {
            collider: pack_collider_handle(handle),
            time_of_impact: hit.time_of_impact,
            normal: vec3_from_rapier(hit.normal),
            feature: feature_id_to_u32(hit.feature),
        })
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_project_point(
    world: *const RcWorldHandle,
    point: RcVec3,
    max_dist: f64,
    solid: RcBool,
    filter: RcQueryFilterDesc,
    out_collider: *mut RcColliderHandle,
) -> RcPointProjection {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcPointProjection::default();
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let Some((handle, projection)) =
        query.project_point(vec3_to_rapier(point), max_dist, solid.0 != 0)
    else {
        return RcPointProjection::default();
    };

    if let Some(out_collider) = unsafe { out_collider.as_mut() } {
        *out_collider = pack_collider_handle(handle);
    }

    RcPointProjection {
        point: vec3_from_rapier(projection.point),
        is_inside: projection.is_inside.into(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_intersect_point_count(
    world: *const RcWorldHandle,
    point: RcVec3,
    filter: RcQueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query.intersect_point(vec3_to_rapier(point)).count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_intersect_aabb_count(
    world: *const RcWorldHandle,
    aabb: RcAabb,
    filter: RcQueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query
        .intersect_aabb_conservative(aabb_to_rapier(aabb))
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_intersect_aabb_count_all(
    world: *const RcWorldHandle,
    aabb: RcAabb,
) -> u32 {
    rc_query_intersect_aabb_count(world, aabb, RcQueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_intersect_aabb_rigid_body_count_all(
    world: *const RcWorldHandle,
    aabb: RcAabb,
) -> u32 {
    crate::compat::rc_query_intersect_aabb_rigid_body_count(
        world,
        aabb,
        RcQueryFilterDesc::default(),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_intersect_aabb_rigid_bodies_all(
    world: *const RcWorldHandle,
    aabb: RcAabb,
    out_handles: *mut crate::ffi::RcRigidBodyHandle,
    capacity: u32,
) -> u32 {
    crate::compat::rc_query_intersect_aabb_rigid_bodies(
        world,
        aabb,
        RcQueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_query_cast_shape(
    world: *const RcWorldHandle,
    shape_desc: RcShapeDesc,
    translation: RcVec3,
    rotation: crate::ffi::RcQuat,
    velocity: RcVec3,
    options: RcShapeCastOptions,
    filter: RcQueryFilterDesc,
) -> RcShapeCastHit {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcShapeCastHit::default();
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );
    let shape = shape_from_desc(shape_desc);

    query
        .cast_shape(
            &crate::ffi::isometry_from_parts(translation, rotation),
            vec3_to_rapier(velocity),
            shape.as_ref(),
            shape_cast_options_to_rapier(options),
        )
        .map(|(handle, hit)| RcShapeCastHit {
            collider: pack_collider_handle(handle),
            time_of_impact: hit.time_of_impact,
            witness1: vec3_from_rapier(hit.witness1),
            witness2: vec3_from_rapier(hit.witness2),
            normal1: vec3_from_rapier(hit.normal1),
            normal2: vec3_from_rapier(hit.normal2),
            status: hit.status as u32,
        })
        .unwrap_or_default()
}
