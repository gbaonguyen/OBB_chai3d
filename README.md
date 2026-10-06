# OBB Tree Collision Detection for CHAI3D

A C++17 implementation of an **Oriented Bounding Box (OBB) tree** collision detector that plugs into the [CHAI3D](https://www.chai3d.org/) haptics framework. It checks whether a line segment (for example, the path of a haptic device pointer) hits a triangle mesh.

The project also includes a **Local Search** speed-up. It uses the triangle that was hit in the previous frame, and its neighbors, to avoid walking the whole tree every time.

## Features

- OBB tree built top-down using PCA (covariance matrix + Jacobi eigenvalue method)
- Two-step collision test:
  - **Coarse test:** segment vs. OBB (slab method)
  - **Fine test:** segment vs. triangle (Möller-Trumbore)
- Local Search using neighbor triangles (triangles that share an edge)
- Real-time OpenGL/GLFW viewer to see the boxes at each tree level
- Unit tests and a benchmark against CHAI3D's built-in AABB tree

## Project Structure

```
.
├── CMakeLists.txt
├── include/
│   ├── CTriangle.h              # Triangle + segment/triangle test
│   ├── CCollisionOBBBox.h       # OBB data + segment/OBB test + drawing
│   ├── CCollisionOBBNode.h      # Abstract base class of tree nodes
│   ├── CCollisionOBBInternal.h  # Internal node (has two children)
│   ├── CCollisionOBBLeaf.h      # Leaf node (holds one triangle)
│   ├── CCollisionOBB.h          # Main detector (builds and queries the tree)
│   └── OBBMath.h                # Centroid, covariance, eigen system, OBB fitting
├── src/                         # One .cpp for each header above
└── tests/
    ├── main.cpp                 # Runs all tests, benchmark, and viewer
    ├── test_math.cpp            # PCA / Jacobi tests
    ├── test_traversal.cpp       # Coarse, fine, and tree query tests
    └── test_bench_mark.cpp      # AABB vs. OBB vs. OBB + Local Search
```

All headers contain declarations only. All code is in the `.cpp` files.

## Requirements

- Linux with a C++17 compiler
- CMake 3.15 or newer
- OpenGL and GLU
- GLFW, libusb-1.0, ALSA (`asound`)
- CHAI3D, already built as a static library

`CMakeLists.txt` looks for CHAI3D here:

```
../Chai3D-Tower-Defense/externals/chai3d/build/libchai3d.a
```

If your CHAI3D is somewhere else, change `CHAI3D_ROOT` in `CMakeLists.txt`.

## Build and Run

```bash
mkdir build && cd build
cmake ..
make
./obb_tests
```

When you run `obb_tests`, it does these things in order:

1. Runs the unit tests (data structures, math, tree, traversal).
2. Runs the benchmark and prints the results to the terminal.
3. Opens the viewer window.

## Viewer Controls

| Input | Action |
|---|---|
| `B` | Show / hide the OBBs |
| `UP` | Show one deeper tree level |
| `DOWN` | Show one higher tree level (down to `-1` = all levels) |
| Left mouse drag | Rotate the camera |
| `ESC` | Quit |

## Basic Usage

```cpp
#include "CCollisionOBB.h"

// 1. Make a list of triangles (index must match the position in the list)
std::vector<cTriangle*> triangles;
triangles.push_back(new cTriangle(v0, v1, v2, 0));
// ...

// 2. Build the tree
cCollisionOBB* detector = new cCollisionOBB();
detector->initialize(triangles);

// 3. Attach it to a CHAI3D mesh
mesh->setCollisionDetector(detector);

// 4. Or query it directly
cCollisionRecorder recorder;
cCollisionSettings settings;
cVector3d pA(...), pB(...);
bool hit = detector->computeCollision(mesh, pA, pB, recorder, settings);
```

To turn Local Search on or off:

```cpp
detector->m_useNeighbors = false;   // always search the whole tree
```

## How It Works (Short Version)

1. **Build.** For a group of triangles, collect all vertices, compute the covariance matrix, and find its eigenvectors. These become the box axes. The box is then sized to fit all the points.
2. **Split.** Cut the box in the middle along its longest axis. Triangles go left or right depending on their centroid. Repeat until each leaf has one triangle.
3. **Query.** Start at the root. If the segment misses a box, skip that whole branch. If it reaches a leaf, test the triangle exactly.
4. **Local Search.** Before using the tree, test the last hit triangle and its neighbors. Most of the time the pointer stays on the same area, so this is fast.

See `REPORT.md` for more details.

## Known Limitations

- Triangle `m_index` must be equal to its position in the input list (Local Search relies on this).
- Positions are used as they are (local = global). There is no object transform.
- Local Search returns the first hit it finds, which may not be the closest one.
- The tree is static. If the mesh changes shape, call `initialize()` again.