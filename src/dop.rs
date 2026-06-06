use std::{collections::HashSet, slice};

use rapier3d::prelude::{ColliderBuilder, Vector};

use crate::ffi::{ColliderBuilderHandle, KdopPreset};

const EPSILON: f64 = 1.0e-9;

#[derive(Clone, Copy)]
struct Slab {
    normal: Vector,
    min: f64,
    max: f64,
}

trait DirectionHull {
    fn directions(&self) -> &[Vector];

    fn build(&self, points: &[Vector]) -> Option<ColliderBuilder> {
        build_direction_hull(points, self.directions())
    }
}

struct KdopHull {
    directions: Vec<Vector>,
}

impl DirectionHull for KdopHull {
    fn directions(&self) -> &[Vector] {
        &self.directions
    }
}

struct FdhHull<'a> {
    directions: &'a [Vector],
}

impl DirectionHull for FdhHull<'_> {
    fn directions(&self) -> &[Vector] {
        self.directions
    }
}

fn normalize_direction(direction: Vector) -> Option<Vector> {
    let len = direction.length();
    (len > EPSILON).then_some(direction / len)
}

fn same_direction(a: Vector, b: Vector) -> bool {
    a.dot(b).abs() >= 1.0 - 1.0e-9
}

fn quantize(value: f64, scale: f64) -> i64 {
    (value * scale).round() as i64
}

fn direction_key(mut direction: Vector) -> (i64, i64, i64) {
    if direction.x < 0.0
        || (direction.x == 0.0 && direction.y < 0.0)
        || (direction.x == 0.0 && direction.y == 0.0 && direction.z < 0.0)
    {
        direction = -direction;
    }
    (
        quantize(direction.x, 1.0e9),
        quantize(direction.y, 1.0e9),
        quantize(direction.z, 1.0e9),
    )
}

fn point_key(point: Vector) -> (i64, i64, i64) {
    (
        quantize(point.x, 1.0e6),
        quantize(point.y, 1.0e6),
        quantize(point.z, 1.0e6),
    )
}

fn push_unique_direction(
    directions: &mut Vec<Vector>,
    keys: &mut HashSet<(i64, i64, i64)>,
    direction: Vector,
) {
    let Some(direction) = normalize_direction(direction) else {
        return;
    };
    let key = direction_key(direction);
    if !keys.insert(key)
        || directions
            .iter()
            .any(|existing| same_direction(*existing, direction))
    {
        return;
    }
    directions.push(direction);
}

fn unique_directions(directions: impl IntoIterator<Item = Vector>) -> Vec<Vector> {
    let mut unique = Vec::new();
    let mut keys = HashSet::new();
    for direction in directions {
        push_unique_direction(&mut unique, &mut keys, direction);
    }
    unique
}

fn read_vectors(values: &[f64]) -> Vec<Vector> {
    values
        .chunks_exact(3)
        .map(|chunk| Vector::new(chunk[0], chunk[1], chunk[2]))
        .collect()
}

fn kdop_directions(preset: KdopPreset) -> Vec<Vector> {
    let mut directions = vec![
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.0, 0.0, 1.0),
    ];

    if matches!(preset, KdopPreset::K14 | KdopPreset::K18 | KdopPreset::K26) {
        directions.extend([
            Vector::new(1.0, 1.0, 1.0),
            Vector::new(1.0, 1.0, -1.0),
            Vector::new(1.0, -1.0, 1.0),
            Vector::new(-1.0, 1.0, 1.0),
        ]);
    }

    if matches!(preset, KdopPreset::K18 | KdopPreset::K26) {
        directions.extend([Vector::new(1.0, 1.0, 0.0), Vector::new(1.0, -1.0, 0.0)]);
    }

    if matches!(preset, KdopPreset::K26) {
        directions.extend([
            Vector::new(1.0, 0.0, 1.0),
            Vector::new(1.0, 0.0, -1.0),
            Vector::new(0.0, 1.0, 1.0),
            Vector::new(0.0, 1.0, -1.0),
        ]);
    }

    unique_directions(directions)
}

fn slabs_from_points(points: &[Vector], directions: &[Vector]) -> Option<Vec<Slab>> {
    let mut slabs = Vec::new();
    for normal in unique_directions(directions.iter().copied()) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for point in points {
            let projection = normal.dot(*point);
            min = min.min(projection);
            max = max.max(projection);
        }

        if min.is_finite() && max.is_finite() {
            slabs.push(Slab { normal, min, max });
        }
    }

    (slabs.len() >= 3).then_some(slabs)
}

fn solve_planes(a: Vector, da: f64, b: Vector, db: f64, c: Vector, dc: f64) -> Option<Vector> {
    let cross_bc = b.cross(c);
    let det = a.dot(cross_bc);
    if det.abs() <= EPSILON {
        return None;
    }

    Some((cross_bc * da + c.cross(a) * db + a.cross(b) * dc) / det)
}

fn contains_point(slabs: &[Slab], point: Vector) -> bool {
    slabs.iter().all(|slab| {
        let projection = slab.normal.dot(point);
        projection >= slab.min - 1.0e-7 && projection <= slab.max + 1.0e-7
    })
}

fn push_unique(points: &mut Vec<Vector>, keys: &mut HashSet<(i64, i64, i64)>, point: Vector) {
    if !keys.insert(point_key(point))
        || points
            .iter()
            .any(|existing| (*existing - point).length_squared() <= 1.0e-12)
    {
        return;
    }

    points.push(point);
}

fn build_direction_hull(points: &[Vector], directions: &[Vector]) -> Option<ColliderBuilder> {
    if points.len() < 4 {
        return None;
    }

    let slabs = slabs_from_points(points, directions)?;
    let mut planes = Vec::with_capacity(slabs.len() * 2);
    for slab in &slabs {
        planes.push((slab.normal, slab.max));
        planes.push((-slab.normal, -slab.min));
    }

    let mut vertices = Vec::new();
    let mut vertex_keys = HashSet::new();
    for i in 0..planes.len() {
        for j in (i + 1)..planes.len() {
            for k in (j + 1)..planes.len() {
                let Some(point) = solve_planes(
                    planes[i].0,
                    planes[i].1,
                    planes[j].0,
                    planes[j].1,
                    planes[k].0,
                    planes[k].1,
                ) else {
                    continue;
                };

                if contains_point(&slabs, point) {
                    push_unique(&mut vertices, &mut vertex_keys, point);
                }
            }
        }
    }

    ColliderBuilder::convex_hull(&vertices)
}

fn builder_from_raw_points(points_xyz: *const f64, point_count: u32) -> Option<Vec<Vector>> {
    if points_xyz.is_null() || point_count < 4 {
        return None;
    }
    let value_count = (point_count as usize).checked_mul(3)?;
    let values = unsafe { slice::from_raw_parts(points_xyz, value_count) };
    Some(read_vectors(values))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_kdop(
    points_xyz: *const f64,
    point_count: u32,
    preset: KdopPreset,
) -> *mut ColliderBuilderHandle {
    let Some(points) = builder_from_raw_points(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };

    let hull = KdopHull {
        directions: kdop_directions(preset),
    };
    let Some(builder) = hull.build(&points) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_fdh(
    points_xyz: *const f64,
    point_count: u32,
    directions_xyz: *const f64,
    direction_count: u32,
) -> *mut ColliderBuilderHandle {
    let Some(points) = builder_from_raw_points(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };
    if directions_xyz.is_null() || direction_count < 3 {
        return std::ptr::null_mut();
    }

    let Some(direction_value_count) = (direction_count as usize).checked_mul(3) else {
        return std::ptr::null_mut();
    };
    let direction_values = unsafe { slice::from_raw_parts(directions_xyz, direction_value_count) };
    let directions = read_vectors(direction_values);
    let hull = FdhHull {
        directions: &directions,
    };
    let Some(builder) = hull.build(&points) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube_points() -> Vec<Vector> {
        let mut points = Vec::new();
        for x in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for z in [-1.0, 1.0] {
                    points.push(Vector::new(x, y, z));
                }
            }
        }
        points
    }

    #[test]
    fn kdop_builds_from_cube_points() {
        let hull = KdopHull {
            directions: kdop_directions(KdopPreset::K14),
        };
        assert!(hull.build(&cube_points()).is_some());
    }

    #[test]
    fn fdh_builds_from_custom_directions() {
        let directions = kdop_directions(KdopPreset::K6);
        let hull = FdhHull {
            directions: &directions,
        };
        assert!(hull.build(&cube_points()).is_some());
    }
}
