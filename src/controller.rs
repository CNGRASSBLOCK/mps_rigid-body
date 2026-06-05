use rapier3d::control::{
    CharacterAutostep, CharacterCollision, CharacterLength, KinematicCharacterController,
};

use crate::ffi::{
    RcBool, RcCharacterCollision, RcCharacterControllerHandle, RcEffectiveCharacterMovement,
    RcQuat, RcShapeDesc, RcVec3, RcWorldHandle, pack_collider_handle, shape_from_desc,
    vec3_from_rapier, vec3_to_rapier,
};

pub(crate) struct CharacterControllerState {
    pub(crate) controller: KinematicCharacterController,
    pub(crate) collisions: Vec<CharacterCollision>,
}

impl Default for CharacterControllerState {
    fn default() -> Self {
        Self {
            controller: KinematicCharacterController::default(),
            collisions: Vec::new(),
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_create() -> *mut RcCharacterControllerHandle {
    Box::into_raw(Box::new(RcCharacterControllerHandle {
        inner: CharacterControllerState::default(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_destroy(controller: *mut RcCharacterControllerHandle) {
    if controller.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(controller));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_up(
    controller: *mut RcCharacterControllerHandle,
    up: RcVec3,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.up = vec3_to_rapier(up);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_offset_absolute(
    controller: *mut RcCharacterControllerHandle,
    offset: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.offset = CharacterLength::Absolute(offset);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_offset_relative(
    controller: *mut RcCharacterControllerHandle,
    offset: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.offset = CharacterLength::Relative(offset);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_slide(
    controller: *mut RcCharacterControllerHandle,
    slide: RcBool,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.slide = slide.0 != 0;
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_autostep(
    controller: *mut RcCharacterControllerHandle,
    enabled: RcBool,
    max_height: f64,
    min_width: f64,
    include_dynamic_bodies: RcBool,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.autostep = if enabled.0 != 0 {
        Some(CharacterAutostep {
            max_height: CharacterLength::Absolute(max_height),
            min_width: CharacterLength::Absolute(min_width),
            include_dynamic_bodies: include_dynamic_bodies.0 != 0,
        })
    } else {
        None
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_snap_to_ground(
    controller: *mut RcCharacterControllerHandle,
    enabled: RcBool,
    distance: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.snap_to_ground = if enabled.0 != 0 {
        Some(CharacterLength::Absolute(distance))
    } else {
        None
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_set_slope_angles(
    controller: *mut RcCharacterControllerHandle,
    max_climb_angle: f64,
    min_slide_angle: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.max_slope_climb_angle = max_climb_angle;
    controller.inner.controller.min_slope_slide_angle = min_slide_angle;
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_move_shape(
    world: *const RcWorldHandle,
    controller: *mut RcCharacterControllerHandle,
    dt: f64,
    shape_desc: RcShapeDesc,
    translation: RcVec3,
    rotation: RcQuat,
    desired_translation: RcVec3,
) -> RcEffectiveCharacterMovement {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcEffectiveCharacterMovement::default();
    };
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return RcEffectiveCharacterMovement::default();
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        rapier3d::prelude::QueryFilter::default(),
    );
    let shape = shape_from_desc(shape_desc);
    controller.inner.collisions.clear();
    let movement = controller.inner.controller.move_shape(
        dt,
        &query,
        shape.as_ref(),
        &crate::ffi::isometry_from_parts(translation, rotation),
        vec3_to_rapier(desired_translation),
        |collision| controller.inner.collisions.push(collision),
    );

    RcEffectiveCharacterMovement {
        translation: vec3_from_rapier(movement.translation),
        grounded: movement.grounded.into(),
        is_sliding_down_slope: movement.is_sliding_down_slope.into(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_collision_count(
    controller: *const RcCharacterControllerHandle,
) -> u32 {
    let Some(controller) = (unsafe { controller.as_ref() }) else {
        return 0;
    };

    controller.inner.collisions.len() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_get_collision(
    controller: *const RcCharacterControllerHandle,
    index: u32,
) -> RcCharacterCollision {
    let Some(controller) = (unsafe { controller.as_ref() }) else {
        return RcCharacterCollision::default();
    };
    let Some(collision) = controller.inner.collisions.get(index as usize) else {
        return RcCharacterCollision::default();
    };

    RcCharacterCollision {
        collider: pack_collider_handle(collision.handle),
        character_translation: vec3_from_rapier(collision.character_pos.translation),
        translation_applied: vec3_from_rapier(collision.translation_applied),
        translation_remaining: vec3_from_rapier(collision.translation_remaining),
        world_witness1: vec3_from_rapier(collision.hit.witness1),
        world_witness2: vec3_from_rapier(collision.hit.witness2),
        normal1: vec3_from_rapier(collision.hit.normal1),
        normal2: vec3_from_rapier(collision.hit.normal2),
        time_of_impact: collision.hit.time_of_impact,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_character_controller_solve_impulses(
    world: *mut RcWorldHandle,
    controller: *mut RcCharacterControllerHandle,
    dt: f64,
    shape_desc: RcShapeDesc,
    character_mass: f64,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return RcBool::FALSE;
    };

    let shape = shape_from_desc(shape_desc);
    let query = world.inner.broad_phase.as_query_pipeline_mut(
        world.inner.narrow_phase.query_dispatcher(),
        &mut world.inner.bodies,
        &mut world.inner.colliders,
        rapier3d::prelude::QueryFilter::default(),
    );

    controller
        .inner
        .controller
        .solve_character_collision_impulses(
            dt,
            &mut { query },
            shape.as_ref(),
            character_mass,
            controller.inner.collisions.iter(),
        );
    RcBool::TRUE
}
