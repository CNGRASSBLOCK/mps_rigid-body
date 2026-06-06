use std::collections::HashMap;

use crate::ffi::{AabbDesc, Bool, RTreeHandle, RTreeStats, Vec3, read_raw_slice, write_raw_slice};

const MAX_CHILDREN: usize = 8;
const LINEAR_SCAN_LIMIT: usize = 16;

#[derive(Clone, Copy, Debug)]
struct Aabb {
    mins: Vec3,
    maxs: Vec3,
}

impl Aabb {
    fn from_desc(desc: AabbDesc) -> Option<Self> {
        if !desc.is_valid() {
            return None;
        }
        Some(Self {
            mins: desc.mins,
            maxs: desc.maxs,
        })
    }

    fn union(self, other: Self) -> Self {
        Self {
            mins: Vec3 {
                x: self.mins.x.min(other.mins.x),
                y: self.mins.y.min(other.mins.y),
                z: self.mins.z.min(other.mins.z),
            },
            maxs: Vec3 {
                x: self.maxs.x.max(other.maxs.x),
                y: self.maxs.y.max(other.maxs.y),
                z: self.maxs.z.max(other.maxs.z),
            },
        }
    }

    fn intersects(self, other: Self) -> bool {
        self.mins.x <= other.maxs.x
            && self.maxs.x >= other.mins.x
            && self.mins.y <= other.maxs.y
            && self.maxs.y >= other.mins.y
            && self.mins.z <= other.maxs.z
            && self.maxs.z >= other.mins.z
    }

    fn center_axis(self, axis: usize) -> f64 {
        match axis {
            0 => (self.mins.x + self.maxs.x) * 0.5,
            1 => (self.mins.y + self.maxs.y) * 0.5,
            _ => (self.mins.z + self.maxs.z) * 0.5,
        }
    }

    fn extent_axis(self, axis: usize) -> f64 {
        match axis {
            0 => self.maxs.x - self.mins.x,
            1 => self.maxs.y - self.mins.y,
            _ => self.maxs.z - self.mins.z,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Entry {
    id: u64,
    bounds: Aabb,
}

#[derive(Clone, Debug)]
enum NodeKind {
    Leaf(Vec<usize>),
    Branch(Vec<Node>),
}

#[derive(Clone, Debug)]
struct Node {
    bounds: Aabb,
    kind: NodeKind,
}

pub(crate) struct RTreeIndex {
    entries: Vec<Entry>,
    id_to_index: HashMap<u64, usize>,
    root: Option<Node>,
    dirty: bool,
}

impl RTreeIndex {
    fn new() -> Self {
        Self {
            entries: Vec::new(),
            id_to_index: HashMap::new(),
            root: None,
            dirty: false,
        }
    }

    fn clear(&mut self) {
        self.entries.clear();
        self.id_to_index.clear();
        self.root = None;
        self.dirty = false;
    }

    fn insert(&mut self, id: u64, bounds: Aabb) -> bool {
        if id == 0 {
            return false;
        }

        if let Some(index) = self.id_to_index.get(&id).copied() {
            self.entries[index].bounds = bounds;
        } else {
            self.id_to_index.insert(id, self.entries.len());
            self.entries.push(Entry { id, bounds });
        }
        self.dirty = true;
        true
    }

    fn update(&mut self, id: u64, bounds: Aabb) -> bool {
        let Some(index) = self.id_to_index.get(&id).copied() else {
            return false;
        };
        self.entries[index].bounds = bounds;
        self.dirty = true;
        true
    }

    fn remove(&mut self, id: u64) -> bool {
        let Some(index) = self.id_to_index.remove(&id) else {
            return false;
        };
        self.entries.swap_remove(index);
        if let Some(moved) = self.entries.get(index) {
            self.id_to_index.insert(moved.id, index);
        }
        self.dirty = true;
        true
    }

    fn rebuild_if_needed(&mut self) {
        if !self.dirty {
            return;
        }
        let mut indices: Vec<_> = (0..self.entries.len()).collect();
        self.root = build_node(&self.entries, &mut indices);
        self.dirty = false;
    }

    fn query_count(&mut self, bounds: Aabb) -> u32 {
        if self.entries.len() <= LINEAR_SCAN_LIMIT {
            return self
                .entries
                .iter()
                .filter(|entry| entry.bounds.intersects(bounds))
                .count()
                .min(u32::MAX as usize) as u32;
        }
        self.rebuild_if_needed();
        let Some(root) = &self.root else {
            return 0;
        };
        count_node(root, &self.entries, bounds)
    }

    fn query(&mut self, bounds: Aabb, out_ids: &mut [u64]) -> u32 {
        if self.entries.len() <= LINEAR_SCAN_LIMIT {
            let mut written = 0usize;
            for entry in &self.entries {
                if written >= out_ids.len() {
                    break;
                }
                if entry.bounds.intersects(bounds) {
                    out_ids[written] = entry.id;
                    written += 1;
                }
            }
            return written as u32;
        }
        self.rebuild_if_needed();
        let Some(root) = &self.root else {
            return 0;
        };
        let mut written = 0usize;
        query_node(root, &self.entries, bounds, out_ids, &mut written);
        written as u32
    }

    fn contains(&self, id: u64) -> bool {
        self.id_to_index.contains_key(&id)
    }

    fn node_count(&self) -> usize {
        self.root.as_ref().map(count_nodes).unwrap_or(0)
    }

    fn height(&self) -> usize {
        self.root.as_ref().map(node_height).unwrap_or(0)
    }

    #[cfg(test)]
    fn check_invariants(&self) -> bool {
        self.entries.len() == self.id_to_index.len()
            && self
                .entries
                .iter()
                .enumerate()
                .all(|(index, entry)| self.id_to_index.get(&entry.id) == Some(&index))
    }
}

fn count_nodes(node: &Node) -> usize {
    match &node.kind {
        NodeKind::Leaf(_) => 1,
        NodeKind::Branch(children) => 1 + children.iter().map(count_nodes).sum::<usize>(),
    }
}

fn node_height(node: &Node) -> usize {
    match &node.kind {
        NodeKind::Leaf(_) => 1,
        NodeKind::Branch(children) => 1 + children.iter().map(node_height).max().unwrap_or(0),
    }
}

fn entries_bounds(entries: &[Entry], indices: &[usize]) -> Option<Aabb> {
    let mut iter = indices.iter();
    let first = entries[*iter.next()?].bounds;
    Some(iter.fold(first, |acc, index| acc.union(entries[*index].bounds)))
}

fn nodes_bounds(nodes: &[Node]) -> Option<Aabb> {
    let mut iter = nodes.iter();
    let first = iter.next()?.bounds;
    Some(iter.fold(first, |acc, node| acc.union(node.bounds)))
}

fn longest_axis(bounds: Aabb) -> usize {
    let x = bounds.extent_axis(0);
    let y = bounds.extent_axis(1);
    let z = bounds.extent_axis(2);
    if x >= y && x >= z {
        0
    } else if y >= z {
        1
    } else {
        2
    }
}

fn build_node(entries: &[Entry], indices: &mut [usize]) -> Option<Node> {
    let bounds = entries_bounds(entries, indices)?;
    if indices.len() <= MAX_CHILDREN {
        return Some(Node {
            bounds,
            kind: NodeKind::Leaf(indices.to_vec()),
        });
    }

    let axis = longest_axis(bounds);
    indices.sort_by(|a, b| {
        entries[*a]
            .bounds
            .center_axis(axis)
            .total_cmp(&entries[*b].bounds.center_axis(axis))
            .then_with(|| entries[*a].id.cmp(&entries[*b].id))
    });

    let mut children = Vec::new();
    for chunk in indices.chunks_mut(MAX_CHILDREN) {
        if let Some(child) = build_node(entries, chunk) {
            children.push(child);
        }
    }

    let bounds = nodes_bounds(&children)?;
    Some(Node {
        bounds,
        kind: NodeKind::Branch(children),
    })
}

fn count_node(node: &Node, entries: &[Entry], bounds: Aabb) -> u32 {
    if !node.bounds.intersects(bounds) {
        return 0;
    }

    match &node.kind {
        NodeKind::Leaf(indices) => indices
            .iter()
            .filter(|index| entries[**index].bounds.intersects(bounds))
            .count() as u32,
        NodeKind::Branch(children) => children
            .iter()
            .map(|child| count_node(child, entries, bounds))
            .sum::<u32>(),
    }
}

fn query_node(
    node: &Node,
    entries: &[Entry],
    bounds: Aabb,
    out_ids: &mut [u64],
    written: &mut usize,
) {
    if *written >= out_ids.len() || !node.bounds.intersects(bounds) {
        return;
    }

    match &node.kind {
        NodeKind::Leaf(indices) => {
            for index in indices {
                if *written >= out_ids.len() {
                    return;
                }
                let entry = entries[*index];
                if entry.bounds.intersects(bounds) {
                    out_ids[*written] = entry.id;
                    *written += 1;
                }
            }
        }
        NodeKind::Branch(children) => {
            for child in children {
                query_node(child, entries, bounds, out_ids, written);
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_create() -> *mut RTreeHandle {
    Box::into_raw(Box::new(RTreeHandle {
        inner: RTreeIndex::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_destroy(tree: *mut RTreeHandle) {
    if tree.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(tree));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_clear(tree: *mut RTreeHandle) {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return;
    };
    tree.inner.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_len(tree: *const RTreeHandle) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    tree.inner.entries.len().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_node_count(tree: *mut RTreeHandle) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    tree.inner.rebuild_if_needed();
    tree.inner.node_count().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_stats(tree: *mut RTreeHandle) -> RTreeStats {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return RTreeStats::default();
    };
    tree.inner.rebuild_if_needed();
    RTreeStats {
        len: tree.inner.entries.len().min(u32::MAX as usize) as u32,
        node_count: tree.inner.node_count().min(u32::MAX as usize) as u32,
        height: tree.inner.height().min(u32::MAX as usize) as u32,
        dirty: tree.inner.dirty.into(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_height(tree: *mut RTreeHandle) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    tree.inner.rebuild_if_needed();
    tree.inner.height().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_is_dirty(tree: *const RTreeHandle) -> Bool {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return Bool::FALSE;
    };
    tree.inner.dirty.into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_contains(tree: *const RTreeHandle, id: u64) -> Bool {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return Bool::FALSE;
    };
    tree.inner.contains(id).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_contains_batch(
    tree: *const RTreeHandle,
    ids: *const u64,
    count: u32,
    out_values: *mut Bool,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(ids) = read_raw_slice(ids, count as usize) else {
        return 0;
    };
    let Some(out_values) = write_raw_slice(out_values, count as usize) else {
        return 0;
    };
    for (id, out) in ids.iter().zip(out_values) {
        *out = tree.inner.contains(*id).into();
    }
    count
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_insert(tree: *mut RTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_insert_batch(
    tree: *mut RTreeHandle,
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
    let mut inserted = 0u32;
    for (id, aabb) in ids.iter().zip(aabbs) {
        let Some(bounds) = Aabb::from_desc(*aabb) else {
            continue;
        };
        if tree.inner.insert(*id, bounds) {
            inserted += 1;
        }
    }
    inserted
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_update_batch(
    tree: *mut RTreeHandle,
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
        let Some(bounds) = Aabb::from_desc(*aabb) else {
            continue;
        };
        if tree.inner.update(*id, bounds) {
            updated += 1;
        }
    }
    updated
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_remove_batch(tree: *mut RTreeHandle, ids: *const u64, count: u32) -> u32 {
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
pub extern "C" fn rtree_update(tree: *mut RTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    if !tree.inner.contains(id) {
        return Bool::FALSE;
    }
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.update(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_remove(tree: *mut RTreeHandle, id: u64) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    tree.inner.remove(id).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_rebuild(tree: *mut RTreeHandle) {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return;
    };
    tree.inner.dirty = true;
    tree.inner.rebuild_if_needed();
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_query_aabb_count(tree: *mut RTreeHandle, aabb: AabbDesc) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };
    tree.inner.query_count(bounds)
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_query_aabb_counts(
    tree: *mut RTreeHandle,
    aabbs: *const AabbDesc,
    count: u32,
    out_counts: *mut u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    let Some(aabbs) = read_raw_slice(aabbs, count as usize) else {
        return 0;
    };
    let Some(out_counts) = write_raw_slice(out_counts, count as usize) else {
        return 0;
    };

    tree.inner.rebuild_if_needed();
    let mut written = 0u32;
    for (aabb, out) in aabbs.iter().zip(out_counts) {
        *out = Aabb::from_desc(*aabb)
            .and_then(|bounds| {
                tree.inner
                    .root
                    .as_ref()
                    .map(|root| count_node(root, &tree.inner.entries, bounds))
            })
            .unwrap_or(0);
        written += 1;
    }
    written
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_query_aabb(
    tree: *mut RTreeHandle,
    aabb: AabbDesc,
    out_ids: *mut u64,
    capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
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
pub extern "C" fn rtree_query_aabbs(
    tree: *mut RTreeHandle,
    aabbs: *const AabbDesc,
    count: u32,
    out_offsets: *mut u32,
    out_ids: *mut u64,
    id_capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
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

    tree.inner.rebuild_if_needed();
    let Some(root) = &tree.inner.root else {
        offsets.fill(0);
        return count;
    };

    let mut written = 0usize;
    offsets[0] = 0;
    for (i, aabb) in aabbs.iter().enumerate() {
        if let Some(bounds) = Aabb::from_desc(*aabb) {
            query_node(root, &tree.inner.entries, bounds, ids, &mut written);
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
    fn rtree_queries_intersections() {
        let tree = rtree_create();
        assert!(!tree.is_null());
        assert_eq!(rtree_is_dirty(tree), Bool::FALSE);

        assert_eq!(rtree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 30, aabb(4.0, 5.0)), Bool::TRUE);
        assert_eq!(rtree_is_dirty(tree), Bool::TRUE);
        assert_eq!(rtree_contains(tree, 20), Bool::TRUE);
        assert_eq!(rtree_contains(tree, 99), Bool::FALSE);

        assert_eq!(rtree_query_aabb_count(tree, aabb(0.5, 2.5)), 2);
        assert_eq!(rtree_is_dirty(tree), Bool::TRUE);

        let mut ids = [0; 4];
        let written = rtree_query_aabb(tree, aabb(0.5, 2.5), ids.as_mut_ptr(), ids.len() as u32);
        assert_eq!(written, 2);
        assert_eq!(&ids[..2], &[10, 20]);
        assert_eq!(rtree_node_count(tree), 1);
        assert_eq!(rtree_height(tree), 1);
        assert_eq!(rtree_is_dirty(tree), Bool::FALSE);
        let stats = rtree_stats(tree);
        assert_eq!(stats.len, 3);
        assert_eq!(stats.node_count, 1);
        assert_eq!(stats.height, 1);
        assert_eq!(stats.dirty, Bool::FALSE);

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_update_and_remove() {
        let tree = rtree_create();

        assert_eq!(rtree_insert(tree, 7, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_update(tree, 7, aabb(10.0, 11.0)), Bool::TRUE);
        assert_eq!(rtree_query_aabb_count(tree, aabb(0.0, 1.0)), 0);
        assert_eq!(rtree_query_aabb_count(tree, aabb(10.5, 10.6)), 1);

        assert_eq!(rtree_remove(tree, 7), Bool::TRUE);
        assert_eq!(rtree_remove(tree, 7), Bool::FALSE);
        assert_eq!(rtree_len(tree), 0);
        assert!(unsafe { (*tree).inner.check_invariants() });

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_rejects_invalid_bounds() {
        let tree = rtree_create();
        assert_eq!(
            rtree_insert(
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
        assert_eq!(rtree_insert(tree, 0, aabb(0.0, 1.0)), Bool::FALSE);
        rtree_destroy(tree);
    }

    #[test]
    fn rtree_insert_batch_skips_invalid_entries() {
        let tree = rtree_create();
        let ids = [1, 2, 0, 3];
        let aabbs = [
            aabb(0.0, 1.0),
            AabbDesc {
                mins: Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 0.0,
                },
                maxs: Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
            },
            aabb(2.0, 3.0),
            aabb(4.0, 5.0),
        ];

        assert_eq!(rtree_insert_batch(tree, ids.as_ptr(), aabbs.as_ptr(), 4), 2);
        assert_eq!(rtree_len(tree), 2);
        assert_eq!(rtree_query_aabb_count(tree, aabb(0.5, 4.5)), 2);
        assert!(unsafe { (*tree).inner.check_invariants() });

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_update_and_remove_batches_keep_index_in_sync() {
        let tree = rtree_create();
        let ids = [1, 2, 3];
        let aabbs = [aabb(0.0, 1.0), aabb(2.0, 3.0), aabb(4.0, 5.0)];
        assert_eq!(rtree_insert_batch(tree, ids.as_ptr(), aabbs.as_ptr(), 3), 3);

        let update_ids = [1, 3, 99];
        let updates = [aabb(10.0, 11.0), aabb(12.0, 13.0), aabb(0.0, 1.0)];
        assert_eq!(
            rtree_update_batch(tree, update_ids.as_ptr(), updates.as_ptr(), 3),
            2
        );
        assert_eq!(rtree_query_aabb_count(tree, aabb(0.0, 5.0)), 1);
        assert_eq!(rtree_query_aabb_count(tree, aabb(10.5, 12.5)), 2);

        let remove_ids = [2, 99, 1];
        assert_eq!(rtree_remove_batch(tree, remove_ids.as_ptr(), 3), 2);
        assert_eq!(rtree_len(tree), 1);
        assert!(unsafe { (*tree).inner.check_invariants() });

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_contains_batch_writes_flags() {
        let tree = rtree_create();
        assert_eq!(rtree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);

        let ids = [10, 11, 20];
        let mut values = [Bool::FALSE; 3];
        assert_eq!(
            rtree_contains_batch(tree, ids.as_ptr(), 3, values.as_mut_ptr()),
            3
        );
        assert_eq!(values, [Bool::TRUE, Bool::FALSE, Bool::TRUE]);

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_query_aabb_counts_batches_queries() {
        let tree = rtree_create();
        assert_eq!(rtree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);

        let queries = [aabb(0.5, 0.75), aabb(0.5, 2.5), aabb(4.0, 5.0)];
        let mut counts = [99u32; 3];
        assert_eq!(
            rtree_query_aabb_counts(tree, queries.as_ptr(), 3, counts.as_mut_ptr()),
            3
        );
        assert_eq!(counts, [1, 2, 0]);

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_query_aabbs_batches_hits_with_offsets() {
        let tree = rtree_create();
        assert_eq!(rtree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);

        let queries = [aabb(0.5, 0.75), aabb(0.5, 2.5), aabb(4.0, 5.0)];
        let mut offsets = [99u32; 4];
        let mut ids = [0u64; 8];
        assert_eq!(
            rtree_query_aabbs(
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

        rtree_destroy(tree);
    }
}
