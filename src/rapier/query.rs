use rapier3d::geometry::{Aabb, Ray};
use rapier3d::parry::shape::FeatureId;
use rapier3d::prelude::SharedShape;

use crate::rapier::ffi::{
    AabbDesc, Bool, ColliderHandleRaw, Obb, PointProjection, QueryFilterDesc, RayHit, ShapeCastHit,
    ShapeCastOptionsDesc, ShapeDesc, Sphere, Vec3, WorldHandle, pack_collider_handle,
    query_filter_from_desc, shape_cast_options_to_rapier, shape_from_desc, vec3_from_rapier,
    vec3_to_rapier,
};

fn aabb_to_rapier(aabb: AabbDesc) -> Aabb {
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

fn obb_shape(obb: Obb) -> Option<SharedShape> {
    if obb.half_extents.x <= 0.0 || obb.half_extents.y <= 0.0 || obb.half_extents.z <= 0.0 {
        return None;
    }

    Some(SharedShape::cuboid(
        obb.half_extents.x,
        obb.half_extents.y,
        obb.half_extents.z,
    ))
}

fn sphere_shape(sphere: Sphere) -> Option<SharedShape> {
    if sphere.radius <= 0.0 {
        return None;
    }

    Some(SharedShape::ball(sphere.radius))
}

fn identity_rotation() -> crate::rapier::ffi::Quat {
    crate::rapier::ffi::Quat {
        i: 0.0,
        j: 0.0,
        k: 0.0,
        w: 1.0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_ray(
    world: *const WorldHandle,
    origin: Vec3,
    direction: Vec3,
    max_toi: f64,
    solid: Bool,
    filter: QueryFilterDesc,
) -> RayHit {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RayHit::default();
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
        .map(|(handle, hit)| RayHit {
            collider: pack_collider_handle(handle),
            time_of_impact: hit.time_of_impact,
            normal: vec3_from_rapier(hit.normal),
            feature: feature_id_to_u32(hit.feature),
        })
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn query_project_point(
    world: *const WorldHandle,
    point: Vec3,
    max_dist: f64,
    solid: Bool,
    filter: QueryFilterDesc,
    out_collider: *mut ColliderHandleRaw,
) -> PointProjection {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return PointProjection::default();
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
        return PointProjection::default();
    };

    if let Some(out_collider) = unsafe { out_collider.as_mut() } {
        *out_collider = pack_collider_handle(handle);
    }

    PointProjection {
        point: vec3_from_rapier(projection.point),
        is_inside: projection.is_inside.into(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_point_count(
    world: *const WorldHandle,
    point: Vec3,
    filter: QueryFilterDesc,
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
pub extern "C" fn query_intersect_aabb_count(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
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
pub extern "C" fn query_intersect_aabb_count_all(world: *const WorldHandle, aabb: AabbDesc) -> u32 {
    query_intersect_aabb_count(world, aabb, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_count(
    world: *const WorldHandle,
    obb: Obb,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    let Some(shape) = obb_shape(obb) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query
        .intersect_shape(
            crate::rapier::ffi::isometry_from_parts(obb.center, obb.rotation),
            shape.as_ref(),
        )
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_count_all(world: *const WorldHandle, obb: Obb) -> u32 {
    query_intersect_obb_count(world, obb, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb(
    world: *const WorldHandle,
    obb: Obb,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 {
        return 0;
    }
    let Some(shape) = obb_shape(obb) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_shape(
        crate::rapier::ffi::isometry_from_parts(obb.center, obb.rotation),
        shape.as_ref(),
    ) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_all(
    world: *const WorldHandle,
    obb: Obb,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    query_intersect_obb(
        world,
        obb,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_count(
    world: *const WorldHandle,
    sphere: Sphere,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    let Some(shape) = sphere_shape(sphere) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query
        .intersect_shape(
            crate::rapier::ffi::isometry_from_parts(sphere.center, identity_rotation()),
            shape.as_ref(),
        )
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_count_all(
    world: *const WorldHandle,
    sphere: Sphere,
) -> u32 {
    query_intersect_sphere_count(world, sphere, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere(
    world: *const WorldHandle,
    sphere: Sphere,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 {
        return 0;
    }
    let Some(shape) = sphere_shape(sphere) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_shape(
        crate::rapier::ffi::isometry_from_parts(sphere.center, identity_rotation()),
        shape.as_ref(),
    ) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_all(
    world: *const WorldHandle,
    sphere: Sphere,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    query_intersect_sphere(
        world,
        sphere,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_body_count_all(
    world: *const WorldHandle,
    aabb: AabbDesc,
) -> u32 {
    crate::rapier::compat::query_intersect_aabb_rigid_body_count(world, aabb, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_bodies_all(
    world: *const WorldHandle,
    aabb: AabbDesc,
    out_handles: *mut crate::rapier::ffi::RigidBodyHandleRaw,
    capacity: u32,
) -> u32 {
    crate::rapier::compat::query_intersect_aabb_rigid_bodies(
        world,
        aabb,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_shape(
    world: *const WorldHandle,
    shape_desc: ShapeDesc,
    translation: Vec3,
    rotation: crate::rapier::ffi::Quat,
    velocity: Vec3,
    options: ShapeCastOptionsDesc,
    filter: QueryFilterDesc,
) -> ShapeCastHit {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return ShapeCastHit::default();
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
            &crate::rapier::ffi::isometry_from_parts(translation, rotation),
            vec3_to_rapier(velocity),
            shape.as_ref(),
            shape_cast_options_to_rapier(options),
        )
        .map(|(handle, hit)| ShapeCastHit {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{Quat, Sphere, Vec3};

    #[test]
    fn obb_query_hits_inserted_obb_collider() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let obb = Obb {
            center: Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            half_extents: Vec3 {
                x: 0.5,
                y: 1.0,
                z: 1.5,
            },
            rotation: Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        };
        let builder = crate::rapier::collider::collider_builder_build(crate::rapier::collider::collider_builder_create_obb(obb));
        assert!(!builder.is_null());

        let collider = crate::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        assert_eq!(query_intersect_obb_count_all(world, obb), 1);

        let mut handles = [0; 1];
        assert_eq!(
            query_intersect_obb_all(world, obb, handles.as_mut_ptr(), handles.len() as u32),
            1
        );
        assert_eq!(handles[0], collider);

        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn sphere_query_hits_inserted_sphere_collider() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 2.0,
                y: 3.0,
                z: 4.0,
            },
            radius: 1.25,
        };
        let builder = crate::rapier::collider::collider_builder_build(crate::rapier::collider::collider_builder_create_sphere(sphere));
        assert!(!builder.is_null());

        let collider = crate::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        assert_eq!(query_intersect_sphere_count_all(world, sphere), 1);

        let mut handles = [0; 1];
        assert_eq!(
            query_intersect_sphere_all(world, sphere, handles.as_mut_ptr(), handles.len() as u32),
            1
        );
        assert_eq!(handles[0], collider);

        crate::rapier::world::world_destroy(world);
    }
}
