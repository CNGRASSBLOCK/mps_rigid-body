use std::cmp::Ordering;
use std::collections::BTreeMap;

use crate::ffi::{AabbDesc, Bool, CRbTreeHandle, Vec3, read_raw_slice, write_raw_slice};

#[derive(Clone, Copy, Debug)]
struct Aabb {
    mins: Vec3,
    maxs: Vec3,
}

impl Aabb {
    fn from_desc(desc: AabbDesc) -> Option<Self> {
        let mins = desc.mins;
        let maxs = desc.maxs;
        if !mins.x.is_finite()
            || !mins.y.is_finite()
            || !mins.z.is_finite()
            || !maxs.x.is_finite()
            || !maxs.y.is_finite()
            || !maxs.z.is_finite()
            || mins.x > maxs.x
            || mins.y > maxs.y
            || mins.z > maxs.z
        {
            return None;
        }

        Some(Self { mins, maxs })
    }

    fn intersects(self, other: Self) -> bool {
        self.mins.x <= other.maxs.x
            && self.maxs.x >= other.mins.x
            && self.mins.y <= other.maxs.y
            && self.maxs.y >= other.mins.y
            && self.mins.z <= other.maxs.z
            && self.maxs.z >= other.mins.z
    }

    fn min_axis(self, axis: usize) -> f64 {
        match axis {
            0 => self.mins.x,
            1 => self.mins.y,
            _ => self.mins.z,
        }
    }

    fn max_axis(self, axis: usize) -> f64 {
        match axis {
            0 => self.maxs.x,
            1 => self.maxs.y,
            _ => self.maxs.z,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct F64Key(f64);

impl Eq for F64Key {}

impl PartialOrd for F64Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for F64Key {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

pub(crate) struct CRbTreeIndex {
    entries: BTreeMap<u64, Aabb>,
    by_min_axis: [BTreeMap<(F64Key, u64), ()>; 3],
}

impl CRbTreeIndex {
    fn new() -> Self {
        Self {
            entries: BTreeMap::new(),
            by_min_axis: std::array::from_fn(|_| BTreeMap::new()),
        }
    }

    fn insert(&mut self, id: u64, bounds: Aabb) -> bool {
        if id == 0 {
            return false;
        }
        if let Some(previous) = self.entries.insert(id, bounds) {
            self.remove_axis_keys(id, previous);
        }
        self.insert_axis_keys(id, bounds);
        true
    }

    fn remove(&mut self, id: u64) -> bool {
        let Some(bounds) = self.entries.remove(&id) else {
            return false;
        };
        self.remove_axis_keys(id, bounds);
        true
    }

    fn clear(&mut self) {
        self.entries.clear();
        for index in &mut self.by_min_axis {
            index.clear();
        }
    }

    fn insert_axis_keys(&mut self, id: u64, bounds: Aabb) {
        for axis in 0..3 {
            self.by_min_axis[axis].insert((F64Key(bounds.min_axis(axis)), id), ());
        }
    }

    fn remove_axis_keys(&mut self, id: u64, bounds: Aabb) {
        for axis in 0..3 {
            self.by_min_axis[axis].remove(&(F64Key(bounds.min_axis(axis)), id));
        }
    }

    fn best_axis(&self, bounds: Aabb) -> usize {
        (0..3)
            .min_by_key(|axis| {
                self.by_min_axis[*axis]
                    .range(..=(F64Key(bounds.max_axis(*axis)), u64::MAX))
                    .count()
            })
            .unwrap_or(0)
    }

    fn candidate_ids(&self, bounds: Aabb) -> impl Iterator<Item = u64> + '_ {
        let axis = self.best_axis(bounds);
        self.by_min_axis[axis]
            .range(..=(F64Key(bounds.max_axis(axis)), u64::MAX))
            .map(|((_, id), ())| *id)
    }

    fn intersects_id(&self, id: u64, bounds: Aabb) -> bool {
        self.entries
            .get(&id)
            .map(|entry| entry.intersects(bounds))
            .unwrap_or(false)
    }

    fn contains(&self, id: u64) -> bool {
        self.entries.contains_key(&id)
    }

    fn len(&self) -> usize {
        self.entries.len()
    }

    #[cfg(test)]
    fn check_invariants(&self) -> bool {
        self.by_min_axis
            .iter()
            .all(|index| index.len() == self.entries.len())
            && self.entries.iter().all(|(id, bounds)| {
                (0..3).all(|axis| {
                    self.by_min_axis[axis].contains_key(&(F64Key(bounds.min_axis(axis)), *id))
                })
            })
    }

    fn query_count(&self, bounds: Aabb) -> u32 {
        self.candidate_ids(bounds)
            .filter(|id| self.intersects_id(*id, bounds))
            .count()
            .min(u32::MAX as usize) as u32
    }

    fn query(&self, bounds: Aabb, out_ids: &mut [u64]) -> u32 {
        let mut hits: Vec<_> = self
            .candidate_ids(bounds)
            .filter(|id| self.intersects_id(*id, bounds))
            .collect();
        hits.sort_unstable();

        let written = hits.len().min(out_ids.len());
        out_ids[..written].copy_from_slice(&hits[..written]);
        written as u32
    }

    fn query_unsorted(&self, bounds: Aabb, out_ids: &mut [u64]) -> u32 {
        let mut written = 0usize;
        for id in self.candidate_ids(bounds) {
            if written >= out_ids.len() {
                break;
            }
            if self.intersects_id(id, bounds) {
                out_ids[written] = id;
                written += 1;
            }
        }
        written as u32
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_create() -> *mut CRbTreeHandle {
    Box::into_raw(Box::new(CRbTreeHandle {
        inner: CRbTreeIndex::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_destroy(tree: *mut CRbTreeHandle) {
    if tree.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(tree));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_clear(tree: *mut CRbTreeHandle) {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return;
    };
    tree.inner.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_len(tree: *const CRbTreeHandle) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    tree.inner.len().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_insert(tree: *mut CRbTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_update(tree: *mut CRbTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    if !tree.inner.contains(id) {
        return Bool::FALSE;
    }
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_update_batch(
    tree: *mut CRbTreeHandle,
    ids: *const u64,
    aabbs: *const AabbDesc,
    count: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    let Some(ids) = read_raw_slice(ids, count as usize) else {
        return 0;
    };
    let Some(aabbs) = read_raw_slice(aabbs, count as usize) else {
        return 0;
    };

    let mut updated = 0u32;
    for (id, aabb) in ids.iter().zip(aabbs) {
        if !tree.inner.contains(*id) {
            continue;
        }
        let Some(bounds) = Aabb::from_desc(*aabb) else {
            continue;
        };
        if tree.inner.insert(*id, bounds) {
            updated += 1;
        }
    }
    updated
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_remove(tree: *mut CRbTreeHandle, id: u64) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    tree.inner.remove(id).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_remove_batch(
    tree: *mut CRbTreeHandle,
    ids: *const u64,
    count: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    let Some(ids) = read_raw_slice(ids, count as usize) else {
        return 0;
    };

    let mut removed = 0u32;
    for id in ids {
        if tree.inner.remove(*id) {
            removed += 1;
        }
    }
    removed
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabb_count(tree: *const CRbTreeHandle, aabb: AabbDesc) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };
    tree.inner.query_count(bounds)
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabb_counts(
    tree: *const CRbTreeHandle,
    aabbs: *const AabbDesc,
    count: u32,
    out_counts: *mut u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(aabbs) = read_raw_slice(aabbs, count as usize) else {
        return 0;
    };
    let Some(out_counts) = write_raw_slice(out_counts, count as usize) else {
        return 0;
    };

    let mut written = 0u32;
    for (aabb, out) in aabbs.iter().zip(out_counts) {
        *out = Aabb::from_desc(*aabb)
            .map(|bounds| tree.inner.query_count(bounds))
            .unwrap_or(0);
        written += 1;
    }
    written
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabb(
    tree: *const CRbTreeHandle,
    aabb: AabbDesc,
    out_ids: *mut u64,
    capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };

    let Some(out) = write_raw_slice(out_ids, capacity as usize) else {
        return 0;
    };
    tree.inner.query(bounds, out)
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabb_unsorted(
    tree: *const CRbTreeHandle,
    aabb: AabbDesc,
    out_ids: *mut u64,
    capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };

    let Some(out) = write_raw_slice(out_ids, capacity as usize) else {
        return 0;
    };
    tree.inner.query_unsorted(bounds, out)
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabbs(
    tree: *const CRbTreeHandle,
    aabbs: *const AabbDesc,
    count: u32,
    out_offsets: *mut u32,
    out_ids: *mut u64,
    id_capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(aabbs) = read_raw_slice(aabbs, count as usize) else {
        return 0;
    };
    let Some(offsets) = write_raw_slice(out_offsets, count as usize + 1) else {
        return 0;
    };
    let Some(ids) = write_raw_slice(out_ids, id_capacity as usize) else {
        return 0;
    };

    let mut written = 0usize;
    offsets[0] = 0;
    for (i, aabb) in aabbs.iter().enumerate() {
        if let Some(bounds) = Aabb::from_desc(*aabb) {
            written += tree.inner.query(bounds, &mut ids[written..]) as usize;
        }
        offsets[i + 1] = written.min(u32::MAX as usize) as u32;
        if written >= ids.len() {
            for offset in &mut offsets[(i + 2)..] {
                *offset = written.min(u32::MAX as usize) as u32;
            }
            break;
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aabb(min: f64, max: f64) -> AabbDesc {
        AabbDesc {
            mins: Vec3 {
                x: min,
                y: min,
                z: min,
            },
            maxs: Vec3 {
                x: max,
                y: max,
                z: max,
            },
        }
    }

    #[test]
    fn crb_tree_queries_intersections_in_id_order() {
        let tree = crb_tree_create();
        assert!(!tree.is_null());

        assert_eq!(crb_tree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 30, aabb(4.0, 5.0)), Bool::TRUE);

        assert_eq!(crb_tree_query_aabb_count(tree, aabb(0.5, 2.5)), 2);

        let mut ids = [0; 4];
        let written = crb_tree_query_aabb(tree, aabb(0.5, 2.5), ids.as_mut_ptr(), ids.len() as u32);
        assert_eq!(written, 2);
        assert_eq!(&ids[..2], &[10, 20]);

        crb_tree_destroy(tree);
    }

    #[test]
    fn crb_tree_update_remove_and_reject_invalid_bounds() {
        let tree = crb_tree_create();

        assert_eq!(crb_tree_insert(tree, 7, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_update(tree, 7, aabb(10.0, 11.0)), Bool::TRUE);
        assert_eq!(crb_tree_query_aabb_count(tree, aabb(0.0, 1.0)), 0);
        assert_eq!(crb_tree_query_aabb_count(tree, aabb(10.5, 10.6)), 1);
        assert_eq!(crb_tree_remove(tree, 7), Bool::TRUE);
        assert_eq!(crb_tree_remove(tree, 7), Bool::FALSE);
        assert_eq!(crb_tree_insert(tree, 0, aabb(0.0, 1.0)), Bool::FALSE);
        assert_eq!(
            crb_tree_insert(
                tree,
                1,
                AabbDesc {
                    mins: Vec3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0
                    },
                    maxs: Vec3 {
                        x: 0.0,
                        y: 1.0,
                        z: 1.0
                    },
                }
            ),
            Bool::FALSE
        );

        crb_tree_destroy(tree);
    }

    #[test]
    fn crb_tree_keeps_min_x_index_in_sync() {
        let mut index = CRbTreeIndex::new();
        assert!(index.insert(1, Aabb::from_desc(aabb(0.0, 1.0)).unwrap()));
        assert!(index.insert(2, Aabb::from_desc(aabb(2.0, 3.0)).unwrap()));
        assert!(index.insert(1, Aabb::from_desc(aabb(4.0, 5.0)).unwrap()));
        assert!(index.remove(2));
        assert!(index.check_invariants());
        assert_eq!(
            index.query_count(Aabb::from_desc(aabb(4.5, 4.6)).unwrap()),
            1
        );
    }

    #[test]
    fn crb_tree_matches_linear_scan_for_many_queries() {
        let mut index = CRbTreeIndex::new();
        let mut reference = BTreeMap::new();

        for i in 1..128u64 {
            let x = ((i * 37) % 41) as f64 * 0.25;
            let y = ((i * 17) % 31) as f64 * 0.25;
            let z = ((i * 23) % 29) as f64 * 0.25;
            let bounds = Aabb {
                mins: Vec3 { x, y, z },
                maxs: Vec3 {
                    x: x + 0.5,
                    y: y + 0.75,
                    z: z + 1.0,
                },
            };
            assert!(index.insert(i, bounds));
            reference.insert(i, bounds);
        }

        for q in 0..64u64 {
            let min = ((q * 13) % 23) as f64 * 0.25;
            let query = Aabb {
                mins: Vec3 {
                    x: min,
                    y: min * 0.5,
                    z: min * 0.25,
                },
                maxs: Vec3 {
                    x: min + 2.0,
                    y: min * 0.5 + 1.5,
                    z: min * 0.25 + 1.0,
                },
            };
            let expected: Vec<_> = reference
                .iter()
                .filter_map(|(id, bounds)| bounds.intersects(query).then_some(*id))
                .collect();
            let mut actual = vec![0; expected.len() + 8];
            let written = index.query(query, &mut actual) as usize;
            assert_eq!(&actual[..written], expected.as_slice());
            assert_eq!(index.query_count(query) as usize, expected.len());
        }
    }

    #[test]
    fn crb_tree_query_aabb_counts_batches_queries() {
        let tree = crb_tree_create();
        assert_eq!(crb_tree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);

        let queries = [aabb(0.5, 0.75), aabb(0.5, 2.5), aabb(4.0, 5.0)];
        let mut counts = [99u32; 3];
        assert_eq!(
            crb_tree_query_aabb_counts(tree, queries.as_ptr(), 3, counts.as_mut_ptr()),
            3
        );
        assert_eq!(counts, [1, 2, 0]);

        crb_tree_destroy(tree);
    }

    #[test]
    fn crb_tree_update_and_remove_batches_keep_index_in_sync() {
        let tree = crb_tree_create();
        assert_eq!(crb_tree_insert(tree, 1, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 2, aabb(2.0, 3.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 3, aabb(4.0, 5.0)), Bool::TRUE);

        let update_ids = [1, 3, 99];
        let updates = [aabb(10.0, 11.0), aabb(12.0, 13.0), aabb(0.0, 1.0)];
        assert_eq!(
            crb_tree_update_batch(tree, update_ids.as_ptr(), updates.as_ptr(), 3),
            2
        );
        assert_eq!(crb_tree_query_aabb_count(tree, aabb(0.0, 5.0)), 1);
        assert_eq!(crb_tree_query_aabb_count(tree, aabb(10.5, 12.5)), 2);

        let remove_ids = [2, 99, 1];
        assert_eq!(crb_tree_remove_batch(tree, remove_ids.as_ptr(), 3), 2);
        assert_eq!(crb_tree_len(tree), 1);
        assert!(unsafe { (*tree).inner.check_invariants() });

        crb_tree_destroy(tree);
    }

    #[test]
    fn crb_tree_query_aabbs_batches_hits_with_offsets() {
        let tree = crb_tree_create();
        assert_eq!(crb_tree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);

        let queries = [aabb(0.5, 0.75), aabb(0.5, 2.5), aabb(4.0, 5.0)];
        let mut offsets = [99u32; 4];
        let mut ids = [0u64; 8];
        assert_eq!(
            crb_tree_query_aabbs(
                tree,
                queries.as_ptr(),
                3,
                offsets.as_mut_ptr(),
                ids.as_mut_ptr(),
                ids.len() as u32,
            ),
            3
        );
        assert_eq!(offsets, [0, 1, 3, 3]);
        assert_eq!(&ids[..3], &[10, 10, 20]);

        crb_tree_destroy(tree);
    }
}
