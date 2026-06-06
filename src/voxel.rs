use std::{collections::HashMap, slice};

use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::prelude::{ColliderBuilder, SharedShape};

use crate::ffi::{ColliderBuilderHandle, Vec3, VoxelColliderMode, VoxelColliderOptions};

struct VoxelGrid<'a> {
    voxels: &'a [u8],
    size_x: usize,
    size_y: usize,
    size_z: usize,
    voxel_size: f64,
    origin: Vec3,
}

impl VoxelGrid<'_> {
    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.size_y * self.size_x + y * self.size_x + x
    }

    fn is_solid(&self, x: usize, y: usize, z: usize) -> bool {
        self.voxels[self.index(x, y, z)] != 0
    }

    fn is_solid_checked(&self, x: isize, y: isize, z: isize) -> bool {
        if x < 0
            || y < 0
            || z < 0
            || x as usize >= self.size_x
            || y as usize >= self.size_y
            || z as usize >= self.size_z
        {
            return false;
        }

        self.is_solid(x as usize, y as usize, z as usize)
    }
}

fn choose_mode(solid_count: usize, options: VoxelColliderOptions) -> VoxelColliderMode {
    if options.mode != VoxelColliderMode::Auto {
        return options.mode;
    }
    if solid_count <= options.small_voxel_limit as usize {
        return VoxelColliderMode::Cuboids;
    }
    if options.dynamic_body.0 != 0 {
        return VoxelColliderMode::GreedyCuboids;
    }
    if solid_count >= options.mesh_voxel_limit as usize {
        return VoxelColliderMode::SurfaceMesh;
    }
    VoxelColliderMode::GreedyCuboids
}

fn push_cuboid(
    grid: &VoxelGrid<'_>,
    parts: &mut Vec<(Pose, SharedShape)>,
    min: (usize, usize, usize),
    max: (usize, usize, usize),
) {
    let (x, y, z) = min;
    let (max_x, max_y, max_z) = max;
    let size_x = (max_x - x) as f64 * grid.voxel_size;
    let size_y = (max_y - y) as f64 * grid.voxel_size;
    let size_z = (max_z - z) as f64 * grid.voxel_size;
    if size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0 {
        return;
    }

    let center = Vector::new(
        grid.origin.x + (x as f64 + (max_x - x) as f64 * 0.5) * grid.voxel_size,
        grid.origin.y + (y as f64 + (max_y - y) as f64 * 0.5) * grid.voxel_size,
        grid.origin.z + (z as f64 + (max_z - z) as f64 * 0.5) * grid.voxel_size,
    );
    parts.push((
        Pose::from_parts(center, Rotation::IDENTITY),
        SharedShape::cuboid(size_x * 0.5, size_y * 0.5, size_z * 0.5),
    ));
}

fn build_cuboids(grid: &VoxelGrid<'_>) -> Option<ColliderBuilder> {
    let mut parts = Vec::new();
    for z in 0..grid.size_z {
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                if grid.is_solid(x, y, z) {
                    push_cuboid(grid, &mut parts, (x, y, z), (x + 1, y + 1, z + 1));
                }
            }
        }
    }
    (!parts.is_empty()).then(|| ColliderBuilder::compound(parts))
}

#[derive(Clone, Copy, Hash, PartialEq, Eq)]
struct LayerRect {
    min_x: usize,
    min_y: usize,
    max_x: usize,
    max_y: usize,
}

struct CuboidRun {
    rect: LayerRect,
    min_z: usize,
    max_z: usize,
    continued: bool,
}

fn layer_rectangles(
    grid: &VoxelGrid<'_>,
    z: usize,
    visited: &mut Vec<bool>,
    rects: &mut Vec<LayerRect>,
) {
    visited.resize(grid.size_x * grid.size_y, false);
    visited.fill(false);
    rects.clear();

    for y in 0..grid.size_y {
        for x in 0..grid.size_x {
            let start = y * grid.size_x + x;
            if visited[start] || !grid.is_solid(x, y, z) {
                continue;
            }

            let mut max_x = x + 1;
            while max_x < grid.size_x {
                let i = y * grid.size_x + max_x;
                if visited[i] || !grid.is_solid(max_x, y, z) {
                    break;
                }
                max_x += 1;
            }

            let mut max_y = y + 1;
            'expand_y: while max_y < grid.size_y {
                for xx in x..max_x {
                    let i = max_y * grid.size_x + xx;
                    if visited[i] || !grid.is_solid(xx, max_y, z) {
                        break 'expand_y;
                    }
                }
                max_y += 1;
            }

            for yy in y..max_y {
                for xx in x..max_x {
                    visited[yy * grid.size_x + xx] = true;
                }
            }

            rects.push(LayerRect {
                min_x: x,
                min_y: y,
                max_x,
                max_y,
            });
        }
    }
}

fn build_greedy_cuboids(grid: &VoxelGrid<'_>) -> Option<ColliderBuilder> {
    let mut parts = Vec::new();
    let mut active: Vec<CuboidRun> = Vec::new();
    let mut visited = Vec::new();
    let mut rects = Vec::new();

    for z in 0..grid.size_z {
        for run in &mut active {
            run.continued = false;
        }
        let active_by_rect: HashMap<_, _> = active
            .iter()
            .enumerate()
            .filter_map(|(index, run)| (run.max_z == z).then_some((run.rect, index)))
            .collect();

        layer_rectangles(grid, z, &mut visited, &mut rects);
        for rect in rects.iter().copied() {
            if let Some(index) = active_by_rect.get(&rect).copied() {
                let run = &mut active[index];
                run.max_z = z + 1;
                run.continued = true;
            } else {
                active.push(CuboidRun {
                    rect,
                    min_z: z,
                    max_z: z + 1,
                    continued: true,
                });
            }
        }

        let mut i = 0;
        while i < active.len() {
            if active[i].continued {
                i += 1;
                continue;
            }
            let run = active.swap_remove(i);
            push_cuboid(
                grid,
                &mut parts,
                (run.rect.min_x, run.rect.min_y, run.min_z),
                (run.rect.max_x, run.rect.max_y, run.max_z),
            );
        }
    }

    for run in active {
        push_cuboid(
            grid,
            &mut parts,
            (run.rect.min_x, run.rect.min_y, run.min_z),
            (run.rect.max_x, run.rect.max_y, run.max_z),
        );
    }

    (!parts.is_empty()).then(|| ColliderBuilder::compound(parts))
}

fn push_face(
    vertices: &mut Vec<Vector>,
    indices: &mut Vec<[u32; 3]>,
    corners: [Vector; 4],
) -> Option<()> {
    let base = u32::try_from(vertices.len()).ok()?;
    vertices.extend(corners);
    indices.push([base, base + 1, base + 2]);
    indices.push([base, base + 2, base + 3]);
    Some(())
}

#[derive(Clone, Copy)]
enum FaceDir {
    XNeg,
    XPos,
    YNeg,
    YPos,
    ZNeg,
    ZPos,
}

#[derive(Clone, Copy)]
struct FaceRect {
    dir: FaceDir,
    plane: usize,
    min_u: usize,
    min_v: usize,
    max_u: usize,
    max_v: usize,
}

struct FaceMask {
    dir: FaceDir,
    plane: usize,
    width: usize,
    height: usize,
}

fn point(grid: &VoxelGrid<'_>, x: usize, y: usize, z: usize) -> Vector {
    Vector::new(
        grid.origin.x + x as f64 * grid.voxel_size,
        grid.origin.y + y as f64 * grid.voxel_size,
        grid.origin.z + z as f64 * grid.voxel_size,
    )
}

fn push_face_rect(
    grid: &VoxelGrid<'_>,
    vertices: &mut Vec<Vector>,
    indices: &mut Vec<[u32; 3]>,
    rect: FaceRect,
) -> Option<()> {
    let corners = match rect.dir {
        FaceDir::XNeg => [
            point(grid, rect.plane, rect.min_u, rect.min_v),
            point(grid, rect.plane, rect.min_u, rect.max_v),
            point(grid, rect.plane, rect.max_u, rect.max_v),
            point(grid, rect.plane, rect.max_u, rect.min_v),
        ],
        FaceDir::XPos => [
            point(grid, rect.plane, rect.min_u, rect.min_v),
            point(grid, rect.plane, rect.max_u, rect.min_v),
            point(grid, rect.plane, rect.max_u, rect.max_v),
            point(grid, rect.plane, rect.min_u, rect.max_v),
        ],
        FaceDir::YNeg => [
            point(grid, rect.min_u, rect.plane, rect.min_v),
            point(grid, rect.max_u, rect.plane, rect.min_v),
            point(grid, rect.max_u, rect.plane, rect.max_v),
            point(grid, rect.min_u, rect.plane, rect.max_v),
        ],
        FaceDir::YPos => [
            point(grid, rect.min_u, rect.plane, rect.min_v),
            point(grid, rect.min_u, rect.plane, rect.max_v),
            point(grid, rect.max_u, rect.plane, rect.max_v),
            point(grid, rect.max_u, rect.plane, rect.min_v),
        ],
        FaceDir::ZNeg => [
            point(grid, rect.min_u, rect.min_v, rect.plane),
            point(grid, rect.min_u, rect.max_v, rect.plane),
            point(grid, rect.max_u, rect.max_v, rect.plane),
            point(grid, rect.max_u, rect.min_v, rect.plane),
        ],
        FaceDir::ZPos => [
            point(grid, rect.min_u, rect.min_v, rect.plane),
            point(grid, rect.max_u, rect.min_v, rect.plane),
            point(grid, rect.max_u, rect.max_v, rect.plane),
            point(grid, rect.min_u, rect.max_v, rect.plane),
        ],
    };
    push_face(vertices, indices, corners)
}

fn push_greedy_face_mask(
    grid: &VoxelGrid<'_>,
    vertices: &mut Vec<Vector>,
    indices: &mut Vec<[u32; 3]>,
    face: FaceMask,
    mask: &mut [bool],
) -> Option<()> {
    for v in 0..face.height {
        for u in 0..face.width {
            let start = v * face.width + u;
            if !mask[start] {
                continue;
            }

            let mut max_u = u + 1;
            while max_u < face.width && mask[v * face.width + max_u] {
                max_u += 1;
            }

            let mut max_v = v + 1;
            'expand_v: while max_v < face.height {
                for uu in u..max_u {
                    if !mask[max_v * face.width + uu] {
                        break 'expand_v;
                    }
                }
                max_v += 1;
            }

            for vv in v..max_v {
                for uu in u..max_u {
                    mask[vv * face.width + uu] = false;
                }
            }

            push_face_rect(
                grid,
                vertices,
                indices,
                FaceRect {
                    dir: face.dir,
                    plane: face.plane,
                    min_u: u,
                    min_v: v,
                    max_u,
                    max_v,
                },
            )?;
        }
    }
    Some(())
}

fn reset_mask(mask: &mut Vec<bool>, len: usize) -> &mut [bool] {
    mask.resize(len, false);
    mask.fill(false);
    mask
}

fn build_surface_mesh(grid: &VoxelGrid<'_>) -> Option<ColliderBuilder> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut neg_mask = Vec::new();
    let mut pos_mask = Vec::new();

    for x in 0..grid.size_x {
        let neg = reset_mask(&mut neg_mask, grid.size_y * grid.size_z);
        let pos = reset_mask(&mut pos_mask, grid.size_y * grid.size_z);
        for z in 0..grid.size_z {
            for y in 0..grid.size_y {
                neg[z * grid.size_y + y] = grid.is_solid(x, y, z)
                    && !grid.is_solid_checked(x as isize - 1, y as isize, z as isize);
                pos[z * grid.size_y + y] = grid.is_solid(x, y, z)
                    && !grid.is_solid_checked(x as isize + 1, y as isize, z as isize);
            }
        }
        push_greedy_face_mask(
            grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::XNeg,
                plane: x,
                width: grid.size_y,
                height: grid.size_z,
            },
            neg,
        )?;
        push_greedy_face_mask(
            grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::XPos,
                plane: x + 1,
                width: grid.size_y,
                height: grid.size_z,
            },
            pos,
        )?;
    }

    for y in 0..grid.size_y {
        let neg = reset_mask(&mut neg_mask, grid.size_x * grid.size_z);
        let pos = reset_mask(&mut pos_mask, grid.size_x * grid.size_z);
        for z in 0..grid.size_z {
            for x in 0..grid.size_x {
                neg[z * grid.size_x + x] = grid.is_solid(x, y, z)
                    && !grid.is_solid_checked(x as isize, y as isize - 1, z as isize);
                pos[z * grid.size_x + x] = grid.is_solid(x, y, z)
                    && !grid.is_solid_checked(x as isize, y as isize + 1, z as isize);
            }
        }
        push_greedy_face_mask(
            grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::YNeg,
                plane: y,
                width: grid.size_x,
                height: grid.size_z,
            },
            neg,
        )?;
        push_greedy_face_mask(
            grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::YPos,
                plane: y + 1,
                width: grid.size_x,
                height: grid.size_z,
            },
            pos,
        )?;
    }

    for z in 0..grid.size_z {
        let neg = reset_mask(&mut neg_mask, grid.size_x * grid.size_y);
        let pos = reset_mask(&mut pos_mask, grid.size_x * grid.size_y);
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                neg[y * grid.size_x + x] = grid.is_solid(x, y, z)
                    && !grid.is_solid_checked(x as isize, y as isize, z as isize - 1);
                pos[y * grid.size_x + x] = grid.is_solid(x, y, z)
                    && !grid.is_solid_checked(x as isize, y as isize, z as isize + 1);
            }
        }
        push_greedy_face_mask(
            grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::ZNeg,
                plane: z,
                width: grid.size_x,
                height: grid.size_y,
            },
            neg,
        )?;
        push_greedy_face_mask(
            grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::ZPos,
                plane: z + 1,
                width: grid.size_x,
                height: grid.size_y,
            },
            pos,
        )?;
    }

    if vertices.is_empty() {
        return None;
    }

    ColliderBuilder::trimesh(vertices, indices).ok()
}

fn build_voxel_collider(
    grid: &VoxelGrid<'_>,
    options: VoxelColliderOptions,
) -> Option<ColliderBuilder> {
    let solid_count = grid.voxels.iter().filter(|voxel| **voxel != 0).count();
    if solid_count == 0 {
        return None;
    }

    match choose_mode(solid_count, options) {
        VoxelColliderMode::Auto => unreachable!(),
        VoxelColliderMode::Cuboids => build_cuboids(grid),
        VoxelColliderMode::GreedyCuboids => build_greedy_cuboids(grid),
        VoxelColliderMode::SurfaceMesh => build_surface_mesh(grid),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxels(
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    origin: Vec3,
    options: VoxelColliderOptions,
) -> *mut ColliderBuilderHandle {
    if voxels.is_null() || size_x == 0 || size_y == 0 || size_z == 0 || voxel_size <= 0.0 {
        return std::ptr::null_mut();
    }

    let Some(xy) = (size_x as usize).checked_mul(size_y as usize) else {
        return std::ptr::null_mut();
    };
    let Some(len) = xy.checked_mul(size_z as usize) else {
        return std::ptr::null_mut();
    };

    let voxels = unsafe { slice::from_raw_parts(voxels, len) };
    let grid = VoxelGrid {
        voxels,
        size_x: size_x as usize,
        size_y: size_y as usize,
        size_z: size_z as usize,
        voxel_size,
        origin,
    };

    let Some(builder) = build_voxel_collider(&grid, options) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ffi::Bool;

    fn options(mode: VoxelColliderMode) -> VoxelColliderOptions {
        VoxelColliderOptions {
            mode,
            dynamic_body: Bool::FALSE,
            small_voxel_limit: 128,
            mesh_voxel_limit: 20_000,
        }
    }

    #[test]
    fn empty_voxels_build_no_collider() {
        let grid = VoxelGrid {
            voxels: &[0; 8],
            size_x: 2,
            size_y: 2,
            size_z: 2,
            voxel_size: 1.0,
            origin: Vec3::default(),
        };

        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::Auto)).is_none());
    }

    #[test]
    fn solid_voxels_build_with_each_mode() {
        let voxels = [1; 8];
        let grid = VoxelGrid {
            voxels: &voxels,
            size_x: 2,
            size_y: 2,
            size_z: 2,
            voxel_size: 1.0,
            origin: Vec3::default(),
        };

        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::Cuboids)).is_some());
        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::GreedyCuboids)).is_some());
        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::SurfaceMesh)).is_some());
    }

    #[test]
    fn layer_rectangles_merge_2d_runs() {
        let voxels = [1, 1, 0, 1, 1, 0];
        let grid = VoxelGrid {
            voxels: &voxels,
            size_x: 3,
            size_y: 2,
            size_z: 1,
            voxel_size: 1.0,
            origin: Vec3::default(),
        };

        let mut visited = Vec::new();
        let mut rects = Vec::new();
        layer_rectangles(&grid, 0, &mut visited, &mut rects);
        assert_eq!(rects.len(), 1);
    }

    #[test]
    fn greedy_face_mask_merges_full_rect() {
        let voxels = [1; 4];
        let grid = VoxelGrid {
            voxels: &voxels,
            size_x: 2,
            size_y: 2,
            size_z: 1,
            voxel_size: 1.0,
            origin: Vec3::default(),
        };
        let mut mask = vec![true; 4];
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        push_greedy_face_mask(
            &grid,
            &mut vertices,
            &mut indices,
            FaceMask {
                dir: FaceDir::ZPos,
                plane: 1,
                width: 2,
                height: 2,
            },
            &mut mask,
        )
        .unwrap();

        assert_eq!(vertices.len(), 4);
        assert_eq!(indices.len(), 2);
        assert!(mask.iter().all(|value| !*value));
    }
}
