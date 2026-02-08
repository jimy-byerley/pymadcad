/*!
    Spatial hashing utilities for efficient proximity queries in 3D

    Port of madcad.hashing
*/

use crate::math::*;
use crate::mesh::simplex_roll;
use crate::rasterize::{self, IVec3};
use rustc_hash::FxHashMap;

// ---- Asso: Multimap ----

/// Associative container that maps each key to multiple values.
///
/// Same flat `{(index, key): value}` scheme as the Python Asso: all entries live
/// in one hash table with no per-key heap allocation. Designed for mappings where
/// each key has few values (1–3 typically), so the short index walk is cheaper
/// than pointer-chasing into separate Vec allocations.
pub struct Asso<K, V> {
    table: FxHashMap<(usize, K), V>,
}

impl<K, V> Asso<K, V>
where
    K: std::hash::Hash + Eq + Clone,
    V: PartialEq,
{
    pub fn new() -> Self {
        Asso {
            table: FxHashMap::default(),
        }
    }

    /// Associate a new value with the given key
    pub fn add(&mut self, key: K, value: V) {
        let i = self.connexity(&key);
        self.table.insert((i, key), value);
    }

    /// Iterate over all values associated with the given key
    pub fn get<'a>(&'a self, key: &'a K) -> AssoIter<'a, K, V> {
        AssoIter {
            table: &self.table,
            key,
            index: 0,
        }
    }

    /// Remove one occurrence of `value` from the values associated with `key`.
    /// Panics if the (key, value) pair is not found.
    pub fn remove(&mut self, key: &K, value: &V) {
        let last = self.connexity(key) - 1;
        for i in (0..=last).rev() {
            if self.table[&(i, key.clone())] == *value {
                if i < last {
                    let moved = self.table.remove(&(last, key.clone())).unwrap();
                    self.table.insert((i, key.clone()), moved);
                } else {
                    self.table.remove(&(i, key.clone()));
                }
                return;
            }
        }
        panic!("key not in table");
    }

    /// Remove all occurrences of `value` from the values associated with `key` (if any)
    pub fn discard(&mut self, key: &K, value: &V) {
        let mut last = match self.connexity(key).checked_sub(1) {
            Some(l) => l,
            None => return,
        };
        let mut i = last;
        loop {
            if self.table[&(i, key.clone())] == *value {
                if i < last {
                    let moved = self.table.remove(&(last, key.clone())).unwrap();
                    self.table.insert((i, key.clone()), moved);
                } else {
                    self.table.remove(&(i, key.clone()));
                }
                last = match last.checked_sub(1) {
                    Some(l) => l,
                    None => break,
                };
            }
            if i == 0 {
                break;
            }
            i -= 1;
        }
    }

    /// Return the number of values associated with the given key
    pub fn connexity(&self, key: &K) -> usize {
        let mut i = 0;
        while self.table.contains_key(&(i, key.clone())) {
            i += 1;
        }
        i
    }

    /// Return true if the key has any associated values
    pub fn contains(&self, key: &K) -> bool {
        self.table.contains_key(&(0, key.clone()))
    }

    /// Clear all associations
    pub fn clear(&mut self) {
        self.table.clear();
    }

    /// Iterator over all (key, value) pairs
    pub fn items(&self) -> impl Iterator<Item = (&K, &V)> {
        self.table.iter().map(|((_, k), v)| (k, v))
    }

    /// Iterator over all keys (may yield duplicates for keys with multiple values)
    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.table.keys().map(|(_, k)| k)
    }

    /// Iterator over all values
    pub fn values(&self) -> impl Iterator<Item = &V> {
        self.table.values()
    }

    /// Append all key-value associations from an iterator
    pub fn update(&mut self, iter: impl IntoIterator<Item = (K, V)>) {
        for (k, v) in iter {
            self.add(k, v);
        }
    }
}

impl<K, V> Default for Asso<K, V>
where
    K: std::hash::Hash + Eq + Clone,
    V: PartialEq,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Iterator over the values associated with a single key in an Asso
pub struct AssoIter<'a, K, V> {
    table: &'a FxHashMap<(usize, K), V>,
    key: &'a K,
    index: usize,
}

impl<'a, K, V> Iterator for AssoIter<'a, K, V>
where
    K: std::hash::Hash + Eq + Clone,
{
    type Item = &'a V;

    fn next(&mut self) -> Option<Self::Item> {
        let v = self.table.get(&(self.index, self.key.clone()))?;
        self.index += 1;
        Some(v)
    }
}


// ---- PositionMap: Spatial Hash Map ----

/// Hash key type for spatial hashing — plain array that implements Hash + Eq
type CellKey = [i64; 3];

/// Convert rasterize::IVec3 (IVec3) to CellKey
#[inline]
fn to_cell_key(k: IVec3) -> CellKey {
    *k.as_array()
}

/// Holds objects associated with their spatial location.
///
/// Every object can be bound to multiple locations, and each location can hold
/// multiple objects. `cellsize` defines the box size for location hashing.
pub struct PositionMap<T> {
    pub cellsize: Float,
    pub dict: FxHashMap<CellKey, Vec<T>>,
}

impl<T> PositionMap<T> {
    pub fn new(cellsize: Float) -> Self {
        PositionMap {
            cellsize,
            dict: FxHashMap::default(),
        }
    }

    /// Hash key for a single point
    pub fn keysfor_point(&self, p: Vec3) -> CellKey {
        let cell = self.cellsize;
        let k = (p / cell).map(|x| x.floor() as i64);
        *k.as_array()
    }

    /// Hash keys for a segment (delegates to rasterize)
    pub fn keysfor_segment(&self, s: &[Vec3; 2]) -> Vec<CellKey> {
        match rasterize::rasterize_segment(s, self.cellsize) {
            Ok(keys) => keys.into_iter().map(to_cell_key).collect(),
            Err(_) => Vec::new(),
        }
    }

    /// Hash keys for a triangle (delegates to rasterize)
    pub fn keysfor_triangle(&self, t: &[Vec3; 3]) -> Vec<CellKey> {
        match rasterize::rasterize_triangle(t, self.cellsize) {
            Ok(keys) => keys.into_iter().map(to_cell_key).collect(),
            Err(_) => Vec::new(),
        }
    }

    /// Add an object at a single cell key
    pub fn add_at(&mut self, key: CellKey, obj: T)
    where
        T: Clone,
    {
        self.dict.entry(key).or_default().push(obj);
    }

    /// Add an object associated with a point
    pub fn add_point(&mut self, p: Vec3, obj: T)
    where
        T: Clone,
    {
        let key = self.keysfor_point(p);
        self.add_at(key, obj);
    }

    /// Add an object associated with a segment
    pub fn add_segment(&mut self, s: &[Vec3; 2], obj: T)
    where
        T: Clone,
    {
        let keys = self.keysfor_segment(s);
        for key in keys {
            self.add_at(key, obj.clone());
        }
    }

    /// Add an object associated with a triangle
    pub fn add_triangle(&mut self, t: &[Vec3; 3], obj: T)
    where
        T: Clone,
    {
        let keys = self.keysfor_triangle(t);
        for key in keys {
            self.add_at(key, obj.clone());
        }
    }

    /// Add an object associated with a list of cell keys
    pub fn add(&mut self, keys: &[CellKey], obj: T)
    where
        T: Clone,
    {
        for key in keys {
            self.dict.entry(*key).or_default().push(obj.clone());
        }
    }

    /// Get all objects potentially at the given cell keys (may contain duplicates)
    pub fn get<'a>(&'a self, keys: &'a [CellKey]) -> impl Iterator<Item = &'a T> + 'a {
        keys.iter().flat_map(|k| {
            self.dict
                .get(k)
                .map(|v| v.as_slice())
                .unwrap_or(&[])
                .iter()
        })
    }

    /// Check if any object is stored at the given cell keys
    pub fn contains(&self, keys: &[CellKey]) -> bool {
        keys.iter().any(|k| {
            self.dict
                .get(k)
                .map(|v| !v.is_empty())
                .unwrap_or(false)
        })
    }

    /// Merge another PositionMap into this one
    pub fn update(&mut self, other: &PositionMap<T>)
    where
        T: Clone,
    {
        assert!(
            (self.cellsize - other.cellsize).abs() < NUMPREC,
            "cellsize mismatch"
        );
        for (k, v) in &other.dict {
            self.dict
                .entry(*k)
                .or_default()
                .extend(v.iter().cloned());
        }
    }

    /// Add elements from an iterator of (keys, obj) pairs
    pub fn update_iter(&mut self, iter: impl IntoIterator<Item = (Vec<CellKey>, T)>)
    where
        T: Clone,
    {
        for (keys, obj) in iter {
            self.add(&keys, obj);
        }
    }

    /// Clear all stored objects
    pub fn clear(&mut self) {
        self.dict.clear();
    }
}


// ---- PointSet: Spatial Point Deduplication ----

/// Holds a deduplicated list of points indexed by spatial hashing.
///
/// `cellsize` is the distance below which two points are considered equivalent.
/// Points are stored in a contiguous Vec and indexed by hash keys.
pub struct PointSet {
    cellsize: Float,
    points: Vec<Vec3>,
    dict: FxHashMap<CellKey, Index>,
}

impl PointSet {
    pub fn new(cellsize: Float) -> Self {
        PointSet {
            cellsize,
            points: Vec::new(),
            dict: FxHashMap::default(),
        }
    }

    /// Build a PointSet by managing an existing point buffer (builds the index)
    pub fn wrap(cellsize: Float, points: Vec<Vec3>) -> Self {
        let mut dict = FxHashMap::default();
        // Insert in reverse order so earlier indices win (matching Python behavior)
        for i in (0..points.len()).rev() {
            let key = Self::keyfor_static(cellsize, points[i]);
            dict.insert(key, i as Index);
        }
        PointSet {
            cellsize,
            points,
            dict,
        }
    }
    
    pub fn unwrap(self) -> Vec<Vec3> {
        self.points
    }

    /// Primary cell key for a point
    pub fn keyfor(&self, pt: Vec3) -> CellKey {
        Self::keyfor_static(self.cellsize, pt)
    }

    fn keyfor_static(cellsize: Float, pt: Vec3) -> CellKey {
        let k = (pt / cellsize).map(|x| x.floor() as i64);
        *k.as_array()
    }

    /// 8 adjacent cell keys for tolerance-based lookup.
    ///
    /// Any point equivalent to `pt` (within cellsize tolerance) must be in
    /// one of these 8 cells.
    pub fn keysfor(&self, pt: Vec3) -> [CellKey; 8] {
        let vox = pt / self.cellsize;
        let lo = vox.map(|x| (x - 0.5 + NUMPREC).floor() as i64);
        let hi = vox.map(|x| (x + 0.5 - NUMPREC).floor() as i64);
        [
            [lo[0], lo[1], lo[2]],
            [hi[0], lo[1], lo[2]],
            [lo[0], hi[1], lo[2]],
            [hi[0], hi[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [hi[0], lo[1], hi[2]],
            [lo[0], hi[1], hi[2]],
            [hi[0], hi[1], hi[2]],
        ]
    }

    /// Add a point to the set. Returns the index of the existing equivalent point
    /// or the newly inserted point.
    pub fn add(&mut self, pt: Vec3) -> Index {
        for key in self.keysfor(pt) {
            if let Some(&idx) = self.dict.get(&key) {
                return idx;
            }
        }
        let idx = self.points.len() as Index;
        self.dict.insert(self.keyfor(pt), idx);
        self.points.append(&mut vec![pt]);
        idx
    }

    /// Remove the point at the given position, returning its former index.
    /// Returns Err if no point exists at this position.
    pub fn remove(&mut self, pt: Vec3) -> Result<Index, &'static str> {
        for key in self.keysfor(pt) {
            if let Some(idx) = self.dict.remove(&key) {
                return Ok(idx);
            }
        }
        Err("position doesn't exist in set")
    }

    /// Remove all points at positions equivalent to the given location (if any)
    pub fn discard(&mut self, pt: Vec3) {
        for key in self.keysfor(pt) {
            self.dict.remove(&key);
        }
    }

    /// Check if there is a point at the given location
    pub fn contains(&self, pt: Vec3) -> bool {
        self.keysfor(pt)
            .iter()
            .any(|key| self.dict.contains_key(key))
    }

    /// Get the index of the point at the given location, if any
    pub fn get(&self, pt: Vec3) -> Option<Index> {
        for key in self.keysfor(pt) {
            if let Some(&idx) = self.dict.get(&key) {
                return Some(idx);
            }
        }
        None
    }

    /// Add points from an iterator
    pub fn update(&mut self, iter: impl IntoIterator<Item = Vec3>) {
        for pt in iter {
            self.add(pt);
        }
    }

    /// Remove points from an iterator
    pub fn difference_update(&mut self, iter: impl IntoIterator<Item = Vec3>) {
        for pt in iter {
            self.discard(pt);
        }
    }

    /// Access the point buffer
    pub fn points(&self) -> &[Vec3] {
        &self.points
    }
}


// ---- Connectivity Helpers ----

/// Return a key for a non-directional edge (sorted pair)
pub fn edgekey(a: Index, b: Index) -> [Index; 2] {
    if a < b {
        [a, b]
    } else {
        [b, a]
    }
}

/// Return a key for an oriented face (rotated so smallest index is first,
/// preserving cyclic order)
pub fn facekeyo(a: Index, b: Index, c: Index) -> [Index; 3] {
    if a < b && b < c {
        [a, b, c]
    } else if a < b {
        [c, a, b]
    } else {
        [b, c, a]
    }
}

/// Point to point connectivity.
///
/// For each ngon, creates bidirectional links between consecutive vertices
/// (including wrap-around from last to first).
pub fn connpp(ngons: &[impl AsRef<[Index]>]) -> FxHashMap<Index, Vec<Index>> {
    let mut conn: FxHashMap<Index, Vec<Index>> = FxHashMap::default();
    for ngon in ngons {
        let loop_ = ngon.as_ref();
        let n = loop_.len();
        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            // edge (loop[i-1], loop[i]) — add loop[i] to loop[i-1]'s neighbors
            for &(a, b) in &[(loop_[prev], loop_[i]), (loop_[i], loop_[prev])] {
                let neighbors = conn.entry(a).or_default();
                if !neighbors.contains(&b) {
                    neighbors.push(b);
                }
            }
        }
    }
    conn
}

/// Oriented edge to face connectivity.
///
/// Maps each directed edge (as a pair of vertex indices) to the face index
/// that contains it. Later faces overwrite earlier ones for the same edge.
pub fn connef(faces: &[[Index; 3]]) -> FxHashMap<[Index; 2], usize> {
    let mut conn = FxHashMap::default();
    for (i, f) in faces.iter().enumerate() {
        for rolled in simplex_roll(*f) {
            conn.insert([rolled[0], rolled[1]], i);
        }
    }
    conn
}

/// Point to edge connectivity
pub fn connpe(edges: &[[Index; 2]]) -> Asso<Index, usize> {
    let mut conn = Asso::new();
    for (i, edge) in edges.iter().enumerate() {
        for &p in edge {
            conn.add(p, i);
        }
    }
    conn
}

/// Return the number of links referencing each point
pub fn connexity(links: &[impl AsRef<[Index]>]) -> FxHashMap<Index, usize> {
    let mut reach: FxHashMap<Index, usize> = FxHashMap::default();
    for l in links {
        for &p in l.as_ref() {
            *reach.entry(p).or_insert(0) += 1;
        }
    }
    reach
}


// ---- suites: Edge Sequence Extraction ----

/// Return a list of the suites (sequences) that can be formed from edges.
///
/// Parameters:
///   - `oriented`: if true, only assemble (a,b) with (b,c), not (a,b) with (c,b)
///   - `cut`: cut suites at crossing points
///   - `do_loop`: stop assembling when a loop is formed
pub fn suites(
    lines: &[[Index; 2]],
    oriented: bool,
    cut: bool,
    do_loop: bool,
) -> Vec<Vec<Index>> {
    let mut lines: Vec<[Index; 2]> = lines.to_vec();
    let mut result: Vec<Vec<Index>> = Vec::new();

    while let Some(edge) = lines.pop() {
        let mut suite = vec![edge[0], edge[1]];

        let mut found = true;
        while found {
            found = false;
            for i in 0..lines.len() {
                let e = lines[i];
                if e[1] == suite[0] {
                    // prepend: edge[-1] == suite[0]
                    suite.insert(0, e[0]);
                    lines.swap_remove(i);
                    found = true;
                    break;
                } else if e[0] == *suite.last().unwrap() {
                    // append: edge[0] == suite[-1]
                    suite.push(e[1]);
                    lines.swap_remove(i);
                    found = true;
                    break;
                } else if !oriented && e[0] == suite[0] {
                    // prepend reversed
                    suite.insert(0, e[1]);
                    lines.swap_remove(i);
                    found = true;
                    break;
                } else if !oriented && e[1] == *suite.last().unwrap() {
                    // append reversed
                    suite.push(e[0]);
                    lines.swap_remove(i);
                    found = true;
                    break;
                }
            }
            if do_loop && *suite.last().unwrap() == suite[0] {
                break;
            }
        }
        result.push(suite);
    }

    // Cut at suite intersections
    if cut {
        let mut reach: FxHashMap<Index, usize> = FxHashMap::default();
        for suite in &result {
            for &p in suite {
                *reach.entry(p).or_insert(0) += 1;
            }
        }
        let mut i = 0;
        while i < result.len() {
            let mut split_at = None;
            #[allow(clippy::needless_range_loop)]
            for j in 1..result[i].len() - 1 {
                if reach.get(&result[i][j]).copied().unwrap_or(0) > 1 {
                    split_at = Some(j);
                    break;
                }
            }
            if let Some(j) = split_at {
                let tail = result[i][j..].to_vec();
                result[i].truncate(j + 1);
                result.push(tail);
            }
            i += 1;
        }
    }

    result
}


/// Returns a good cell size to index primitives of a mesh with a PositionMap.
///
/// Takes pre-computed bounding box size and number of points to avoid coupling
/// to mesh lifetime.
pub fn meshcellsize(bounds_size: Vec3, npoints: usize) -> Float {
    let length = bounds_size.length();
    length / (npoints as Float).sqrt()
}


#[cfg(test)]
mod tests {
    use super::*;

    // ---- Asso tests ----

    #[test]
    fn test_asso_basic() {
        let mut a: Asso<u32, u32> = Asso::new();
        a.add(1, 10);
        a.add(1, 20);
        a.add(2, 30);

        let v1: Vec<_> = a.get(&1).copied().collect();
        assert_eq!(v1, vec![10, 20]);
        let v2: Vec<_> = a.get(&2).copied().collect();
        assert_eq!(v2, vec![30]);
        assert_eq!(a.get(&3).count(), 0);
        assert!(a.contains(&1));
        assert!(!a.contains(&3));
        assert_eq!(a.connexity(&1), 2);
        assert_eq!(a.connexity(&3), 0);
    }

    #[test]
    fn test_asso_remove() {
        let mut a: Asso<u32, u32> = Asso::new();
        a.add(1, 10);
        a.add(1, 20);
        a.add(1, 30);
        a.remove(&1, &20);
        // swap_remove: 30 replaces 20
        assert_eq!(a.connexity(&1), 2);
        let vals: Vec<_> = a.get(&1).copied().collect();
        assert!(vals.contains(&10));
        assert!(vals.contains(&30));
    }

    #[test]
    fn test_asso_discard() {
        let mut a: Asso<u32, u32> = Asso::new();
        a.add(1, 10);
        a.add(1, 10);
        a.add(1, 20);
        a.discard(&1, &10);
        let vals: Vec<_> = a.get(&1).copied().collect();
        assert_eq!(vals, vec![20]);
    }

    #[test]
    fn test_asso_clear() {
        let mut a: Asso<u32, u32> = Asso::new();
        a.add(1, 10);
        a.clear();
        assert!(!a.contains(&1));
    }

    #[test]
    fn test_asso_update() {
        let mut a: Asso<u32, u32> = Asso::new();
        a.update(vec![(1, 10), (1, 20), (2, 30)]);
        assert_eq!(a.connexity(&1), 2);
        assert_eq!(a.connexity(&2), 1);
    }

    // ---- PositionMap tests ----

    #[test]
    fn test_positionmap_point() {
        let mut pm: PositionMap<usize> = PositionMap::new(1.0);
        pm.add_point(Vec3::from([0.5, 0.5, 0.5]), 42);
        let key = pm.keysfor_point(Vec3::from([0.5, 0.5, 0.5]));
        assert!(pm.contains(&[key]));
        let items: Vec<_> = pm.get(&[key]).copied().collect();
        assert_eq!(items, vec![42]);
    }

    #[test]
    fn test_positionmap_segment() {
        let mut pm: PositionMap<usize> = PositionMap::new(1.0);
        let seg = [Vec3::from([0.0, 0.0, 0.0]), Vec3::from([0.0, 0.0, 2.0])];
        pm.add_segment(&seg, 7);
        // Should have entries along the z-axis
        assert!(pm.contains(&[[0, 0, 0]]));
        assert!(pm.contains(&[[0, 0, 1]]));
    }

    #[test]
    fn test_positionmap_triangle() {
        let mut pm: PositionMap<usize> = PositionMap::new(1.0);
        let tri = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
            Vec3::from([0.0, 2.0, 0.0]),
        ];
        pm.add_triangle(&tri, 99);
        assert!(pm.contains(&[[0, 0, 0]]));
    }

    #[test]
    fn test_positionmap_update() {
        let mut pm1: PositionMap<usize> = PositionMap::new(1.0);
        pm1.add_point(Vec3::from([0.5, 0.5, 0.5]), 1);
        let mut pm2: PositionMap<usize> = PositionMap::new(1.0);
        pm2.add_point(Vec3::from([0.5, 0.5, 0.5]), 2);
        pm1.update(&pm2);
        let key = pm1.keysfor_point(Vec3::from([0.5, 0.5, 0.5]));
        let items: Vec<_> = pm1.get(&[key]).copied().collect();
        assert_eq!(items, vec![1, 2]);
    }

    // ---- PointSet tests ----

    #[test]
    fn test_pointset_add_dedup() {
        let mut ps = PointSet::new(0.1);
        let i0 = ps.add(Vec3::from([1.0, 2.0, 3.0]));
        let i1 = ps.add(Vec3::from([1.0, 2.0, 3.0]));
        assert_eq!(i0, i1);
        assert_eq!(ps.points().len(), 1);
    }

    #[test]
    fn test_pointset_distinct_points() {
        let mut ps = PointSet::new(0.1);
        let i0 = ps.add(Vec3::from([0.0, 0.0, 0.0]));
        let i1 = ps.add(Vec3::from([1.0, 1.0, 1.0]));
        assert_ne!(i0, i1);
        assert_eq!(ps.points().len(), 2);
    }

    #[test]
    fn test_pointset_contains() {
        let mut ps = PointSet::new(0.1);
        ps.add(Vec3::from([1.0, 2.0, 3.0]));
        assert!(ps.contains(Vec3::from([1.0, 2.0, 3.0])));
        assert!(!ps.contains(Vec3::from([5.0, 5.0, 5.0])));
    }

    #[test]
    fn test_pointset_remove() {
        let mut ps = PointSet::new(0.1);
        ps.add(Vec3::from([1.0, 2.0, 3.0]));
        let idx = ps.remove(Vec3::from([1.0, 2.0, 3.0]));
        assert!(idx.is_ok());
        assert!(!ps.contains(Vec3::from([1.0, 2.0, 3.0])));
    }

    #[test]
    fn test_pointset_remove_nonexistent() {
        let mut ps = PointSet::new(0.1);
        let result = ps.remove(Vec3::from([1.0, 2.0, 3.0]));
        assert!(result.is_err());
    }

    #[test]
    fn test_pointset_get() {
        let mut ps = PointSet::new(0.1);
        let i0 = ps.add(Vec3::from([1.0, 2.0, 3.0]));
        assert_eq!(ps.get(Vec3::from([1.0, 2.0, 3.0])), Some(i0));
        assert_eq!(ps.get(Vec3::from([5.0, 5.0, 5.0])), None);
    }

    #[test]
    fn test_pointset_wrap() {
        let pts = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 1.0, 1.0]),
        ];
        let ps = PointSet::wrap(0.1, pts);
        assert!(ps.contains(Vec3::from([0.0, 0.0, 0.0])));
        assert!(ps.contains(Vec3::from([1.0, 1.0, 1.0])));
        assert_eq!(ps.get(Vec3::from([0.0, 0.0, 0.0])), Some(0));
        assert_eq!(ps.get(Vec3::from([1.0, 1.0, 1.0])), Some(1));
    }

    #[test]
    fn test_pointset_near_boundary() {
        // Two points that are close but on different sides of a cell boundary
        // should still be deduplicated via the 8-cell keysfor lookup
        let mut ps = PointSet::new(1.0);
        let i0 = ps.add(Vec3::from([0.99, 0.0, 0.0]));
        let i1 = ps.add(Vec3::from([1.01, 0.0, 0.0]));
        // Within cellsize tolerance, these should be the same point
        assert_eq!(i0, i1);
    }

    // ---- Connectivity helper tests ----

    #[test]
    fn test_edgekey() {
        assert_eq!(edgekey(3, 1), [1, 3]);
        assert_eq!(edgekey(1, 3), [1, 3]);
    }

    #[test]
    fn test_facekeyo() {
        assert_eq!(facekeyo(0, 1, 2), [0, 1, 2]);
        assert_eq!(facekeyo(1, 2, 0), [0, 1, 2]);
        assert_eq!(facekeyo(2, 0, 1), [0, 1, 2]);
    }

    #[test]
    fn test_connpp() {
        // Triangle (0,1,2) should create bidirectional links
        let faces: Vec<Vec<Index>> = vec![vec![0, 1, 2]];
        let conn = connpp(&faces);
        assert!(conn[&0].contains(&1));
        assert!(conn[&0].contains(&2));
        assert!(conn[&1].contains(&0));
        assert!(conn[&1].contains(&2));
    }

    #[test]
    fn test_connef() {
        let faces = vec![[0u32, 1, 2]];
        let conn = connef(&faces);
        assert_eq!(conn[&[0, 1]], 0);
        assert_eq!(conn[&[1, 2]], 0);
        assert_eq!(conn[&[2, 0]], 0);
    }

    #[test]
    fn test_connexity() {
        let faces: Vec<[Index; 3]> = vec![[0, 1, 2], [1, 2, 3]];
        let reach = connexity(&faces);
        assert_eq!(reach[&0], 1);
        assert_eq!(reach[&1], 2);
        assert_eq!(reach[&2], 2);
        assert_eq!(reach[&3], 1);
    }

    // ---- suites tests ----

    #[test]
    fn test_suites_simple_chain() {
        let lines: Vec<[Index; 2]> = vec![[0, 1], [1, 2], [2, 3]];
        let result = suites(&lines, true, false, false);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], vec![0, 1, 2, 3]);
    }

    #[test]
    fn test_suites_two_chains() {
        let lines: Vec<[Index; 2]> = vec![[0, 1], [1, 2], [5, 6]];
        let result = suites(&lines, true, false, false);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_suites_loop() {
        let lines: Vec<[Index; 2]> = vec![[0, 1], [1, 2], [2, 0]];
        let result = suites(&lines, true, false, true);
        assert_eq!(result.len(), 1);
        assert_eq!(*result[0].first().unwrap(), *result[0].last().unwrap());
    }

    #[test]
    fn test_suites_unoriented() {
        // Edges where direction is reversed — should still assemble
        let lines: Vec<[Index; 2]> = vec![[0, 1], [2, 1]];
        let result = suites(&lines, false, false, false);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].len(), 3);
    }

    #[test]
    fn test_suites_cut() {
        // Two chains sharing a crossing point (1)
        let lines: Vec<[Index; 2]> = vec![[0, 1], [1, 2], [3, 1], [1, 4]];
        let result = suites(&lines, true, true, false);
        // Should be cut at point 1
        assert!(result.len() >= 2);
    }

    // ---- meshcellsize test ----

    #[test]
    fn test_meshcellsize() {
        let size = Vec3::from([10.0, 10.0, 10.0]);
        let cs = meshcellsize(size, 100);
        assert!(cs > 0.0);
        // sqrt(300) / sqrt(100) = sqrt(3) ≈ 1.73
        assert!((cs - (300.0_f64).sqrt() / 10.0).abs() < 0.01);
    }
}
