# Report: OBB Tree Collision Detection with Local Search for CHAI3D

## 1. Introduction

In a haptic application, the program must find out very quickly if the pointer touches an object. A common way is to test a line segment (from the last pointer position to the new one) against every triangle of a mesh. This is too slow for big meshes, because haptic loops often need to run at about 1000 Hz.

A **bounding volume hierarchy (BVH)** solves this. I put triangles inside boxes, and put boxes inside bigger boxes. If a segment misses a big box, i skip every triangle inside it.

CHAI3D already has an AABB tree (axis-aligned boxes). In this project i build an **OBB tree** (oriented boxes) that fits the mesh more tightly. I also add a **Local Search** step to use the fact that the pointer moves smoothly from one frame to the next.

**Goals**

1. Implement an OBB tree that works as a CHAI3D collision detector.
2. Add Local Search using neighbor triangles.
3. Show the tree in a viewer.
4. Compare it with CHAI3D's AABB tree.

## 2. Background

**AABB vs. OBB.** An AABB is aligned with the world axes. An OBB can rotate, so it can fit a long, tilted shape much better. Tighter boxes mean fewer false hits, but each box test costs a bit more.

**PCA.** To find a good rotation for the box, i use Principal Component Analysis. We compute the covariance matrix of the vertices. Its eigenvectors point in the directions where the points spread the most. I use these as the box axes.

**Jacobi method.** The covariance matrix is 3×3 and symmetric. The Jacobi method finds its eigenvalues and eigenvectors by rotating the matrix step by step until it becomes diagonal.

## 3. Design

### 3.1 Class Overview

| Class | Role |
|---|---|
| `cTriangle` | Stores 3 vertices, an index, and a neighbor list. Does the fine test. |
| `cCollisionOBBBox` | Stores center, half-sizes (`m_extent`) and 3 axes (`u[3]`). Does the coarse test and drawing. |
| `cCollisionOBBNode` | Abstract base class for tree nodes. |
| `cCollisionOBBInternal` | Node with a left and a right child. |
| `cCollisionOBBLeaf` | Node that holds one triangle. |
| `cCollisionOBB` | Main class. Builds the tree, builds neighbors, and answers queries. |
| `OBBMath` (functions) | Centroid, covariance, Jacobi eigen solver, and OBB fitting. |

Nodes use inheritance and virtual functions, so the tree code does not need to know if a node is internal or a leaf.

The code follows the standard C++ style: `.h` files have declarations, and `.cpp` files have the implementation.

### 3.2 Building the Tree

For a list of triangles:

1. Collect all vertices (3 per triangle).
2. Compute the centroid and the covariance matrix.
3. Use Jacobi to get eigenvalues and eigenvectors. Sort them from large to small and make the axes right-handed.
4. For each axis, project all vertices on it. The box half-size is half of (max − min). The center is built from the middle values.
5. If there is only one triangle, make a leaf.
6. Otherwise, choose the longest box axis and split by the triangle centroid: if its projection from the box center is >= 0 it goes left, otherwise right.
7. If one side is empty, split the list in half instead. This prevents endless recursion.
8. Build the left and right children the same way.

### 3.3 Collision Query

**Coarse test (segment vs. OBB):** I move the segment into the box's own coordinates. Then i use the slab method: for each axis, i find where the segment enters and leaves the pair of planes. If the entering and leaving ranges do not overlap, there is no hit.

**Fine test (segment vs. triangle):** I use the Möller-Trumbore algorithm. It gives the hit point, the normal, and the parameter t on the segment.

**Tree traversal:** At each node, do the coarse test. If it fails, return. If the node is internal, visit both children. If it is a leaf, run the fine test and save a cCollisionEvent when there is a hit.

### 3.4 Local Search

The pointer usually moves a small distance each frame, so it tends to hit the same triangle or one next to it.

1. **Neighbor graph.** Two triangles are neighbors if they share an edge. We find them by making a text key for each edge (vertex coordinates rounded to 4 decimals, in a fixed order) and putting triangles with the same key together.
2. **Query steps.**
   1. Test the last hit triangle.
   2. If it fails, test its neighbors.
   3. If it still fails, do the normal tree traversal (fallback).
3. After a tree traversal, we sort hits by distance from the segment start and remember the closest triangle for the next frame. If nothing was hit, we forget it.

## 4. Implementation Details

- **Language / build:** C++17, CMake, linked with the static CHAI3D library.
- **Jacobi solver:** at most 50 iterations, stops when the largest off-diagonal value is smaller than `1e-10`.
- **Parallel segment:** if the segment is almost parallel to a slab (`|d| < 1e-9`), we only check if the start point is inside that slab.
- **Viewer:** `cCollisionOBB::render()` draws the boxes in OpenGL. A depth number controls which tree level is shown (`-1` means all levels).
- **Memory:** deleting the root deletes the whole tree, because each internal node deletes its children in its destructor. Triangles are owned by the caller.

## 5. Testing

We wrote simple tests with `assert`.

| Test | What it checks | Expected |
|---|---|---|
| Phase 1 | Box data can be stored | Values are saved correctly |
| Math (`test_math.cpp`) | A box of size 4×2×1, rotated by 45° | Axes are perpendicular, extents are (2, 1, 0.5), center at origin |
| Tree (`runTreeTests`) | A tetrahedron with 4 triangles | 4 leaves, depth between 2 and 4 |
| Coarse test | One segment through a box, one segment beside it | Hit / no hit |
| Fine test | Segment through a triangle in the z = 0 plane | Hit point has z ≈ 0 |
| Tree query | Two separate triangles, segment goes through the second | Hit with triangle index 2 |

All tests pass.

## 6. Benchmark

### 6.1 Setup

We compare three methods:

1. CHAI3D AABB tree (`createAABBCollisionDetector`)
2. OBB tree, Local Search **off** (always traverses the tree)
3. OBB tree, Local Search **on**

**Meshes.** A sphere of radius 1.0 made with `cCreateSphere` using 16, 45, and 130 slices/stacks. This gives meshes of roughly 1k, 10k, and 100k triangles (the program prints the exact number).

**Queries.** 5000 segments, each 0.3 long in the −x direction. The start point begins at (1.1, 0, 0) and moves by (0.0002, 0.0003, 0.0001) each query. This imitates a pointer that moves smoothly.

**Measured:** build time (ms) and average time per query (µs).

### 6.2 Results

_Run `./obb_tests` and copy the numbers here._

**Build time**

| Triangles | AABB (ms) | OBB (ms) |
|---|---|---|
| ~512 |0.73 |35.49 |
| ~4050 | 1.07| 125.05|
| ~33800 | 16.98| 1093.78|

**Average query time**

| Triangles | AABB (µs) | OBB (DFS) (µs) | OBB (Local Search) (µs) |
|---|---|---|---|
| ~512 | 0.11 | 1.60 | 0.44 |
| ~4050 | 0.12 | 1.84 | 0.39 |
| ~33800 |0.14 | 1.92 | 0.38 |

### 6.3 Discussion

- OBB build time is usually higher than AABB, because it needs a covariance matrix and Jacobi at every node.
- Does OBB (DFS) beat AABB on query time? On a sphere, boxes are not very different, so the gain may be small.
- Does Local Search reduce the query time? It should help most when the pointer stays on the surface.
- How does each method grow when the mesh gets 10× bigger?

**Note about the test path.** The start point moves away from the sphere over time (it reaches about x = 2.1, y = 1.5). By our estimate, only the first part of the 5000 queries actually crosses the sphere surface, and the rest are misses. Local Search can only help on hits, so its gain is probably under-measured. A path that follows the sphere surface would be a fairer test and can be listed as future work.

## 7. Limitations

- Triangle `m_index` must match its position in the triangle list.
- No transform support: local and global positions are the same.
- Local Search returns the first hit it finds, which may not be the closest if the surface folds back.
- The tree is static. If the mesh deforms, it must be rebuilt.
- Neighbor finding uses string keys and rounding. It is simple, but slow for big meshes and can fail if vertices differ by tiny amounts.
- The split uses the box center, not the median, so the tree can be unbalanced.

## 8. Future Work

- Use the median for splitting to get a more balanced tree.
- Build neighbors from vertex indices instead of strings.
- Add object transforms and support for deforming meshes (refit the boxes).
- Test with real meshes and a real haptic device.
- Use a surface-following path in the benchmark.

## 9. Conclusion

We built an OBB tree collision detector for CHAI3D using PCA, Jacobi eigen solving, a slab-method coarse test, and a Möller-Trumbore fine test. We added a Local Search step based on neighbor triangles, and a viewer to inspect the tree. The code passes all unit tests, and the benchmark compares it with CHAI3D's AABB tree.

## 10. References

- Gottschalk, S., Lin, M. C., Manocha, D. (1996). *OBBTree: A Hierarchical Structure for Rapid Interference Detection.* SIGGRAPH.
- Möller, T., Trumbore, B. (1997). *Fast, Minimum Storage Ray/Triangle Intersection.* Journal of Graphics Tools.
- Ericson, C. (2004). *Real-Time Collision Detection.* Morgan Kaufmann.
- CHAI3D documentation: https://www.chai3d.org/
- Artificial Intelligence model: Gemini 3.8 Flash