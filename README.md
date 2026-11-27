# LaplacianDeformer

Interactive Laplacian mesh deformation: drag handle vertices on a triangle mesh while other vertices stay fixed, and the rest of the surface follows by minimising a stretching + bending energy.

Implementation of the method surveyed in:

> Mario Botsch and Olga Sorkine. **On Linear Variational Surface Deformation Methods.**
> *IEEE Transactions on Visualization and Computer Graphics*, 14(1):213–230, 2008.
> [doi:10.1109/TVCG.2007.1054](https://doi.org/10.1109/TVCG.2007.1054) · [PDF](https://cg.cs.tu-dortmund.de/publications/2008-survey.pdf)

Originally written in C++ in 2017 (Qt + libQGLViewer); in 2026 the same `Deformer` was compiled to WebAssembly with a browser viewer for side-by-side comparisons.

## Method

The displacement field **d** minimises

&nbsp;&nbsp;&nbsp;&nbsp;E(**d**) = ½ Ks ∫ ‖∇**d**‖² + ½ Kb ∫ ‖Δ**d**‖²

subject to the handle (prescribed displacement) and fixed (zero displacement) vertices. Discretised with the cotangent Laplacian **L** = **M**⁻¹**L**ₛ this gives the linear system

&nbsp;&nbsp;&nbsp;&nbsp;(−Ks **L**ₛ + Kb **L**ₛ **M**⁻¹ **L**ₛ) **d** = 0

solved per coordinate for the free vertices (`Deformer` in `deform.cpp`).

- **Ks** (stretching / membrane): Ks = 1, Kb = 0 gives a harmonic field — sharp, cone-like deformations with creases at the constraints.
- **Kb** (bending / thin plate): Ks = 0, Kb = 1 gives a biharmonic field — smooth bumps that blend into the surface.
- Only the ratio Ks : Kb matters; √(Kb/Ks) is a length scale, so the balance depends on the mesh size.

## Web version (`web/`)

A single self-contained page, `web/deformer.html` — open it in Chrome or Safari, nothing to install.

- `Deformer` is taken from `deform.h` / `deform.cpp` at build time (unchanged apart from one `friend` declaration for the fast path) and compiled to WebAssembly with the ElasticFEM TriMesh/LinearAlgebra libraries (which also load the OBJ files).
- The page solves the same system as `Deformer::deform`, but builds it sparse and reuses the factorisation for a whole drag, so each mouse move is a back-substitution. This makes ~50k-vertex meshes interactive; `web/test/verify.mjs` checks it against the original `deform()` (agreement to ~1e-7).
- **Side by side**: two deformers with their own Ks/Kb share the camera, the fixed set and the handle; every drag is applied to both.

### Controls

| Key | Action |
|---|---|
| Drag / right-drag / scroll | orbit / pan / zoom (view mode) |
| Shift-drag, Ctrl/⌘-drag | select / deselect vertices in a box |
| S / U | fix the selection / unfix it (or clear all fixed if nothing is selected) |
| W | move mode: drag to deform the red handle |
| ⌘Z, R | undo the last drag, reset the shape |
| 1, 2, 3 | demos: plane with one edge fixed, plane with all edges fixed, kitten |
| Shift + 1/2/3 | save the current fixed set, handle and camera as that demo |
| C | side by side on/off |
| V | display: shaded / displacement heat map / zebra stripes |
| E, M, F, H | wireframe, markers, frame the mesh, hide the UI for recording |

Demo 3 uses `kitten.obj`, which is not included in the repository: load it once with **Load OBJ**, or embed it at build time with `KITTEN=…`.

### Building

Requires [Emscripten](https://emscripten.org), Eigen 3, Node.js and the ElasticFEM sources (`TriMesh/`, `LinearAlgebra/`).

```bash
cd web
ELASTICFEM=/path/to/ElasticFEM ./build.sh                    # -> deformer.html
ELASTICFEM=/path/to/ElasticFEM KITTEN=/path/to/kitten.obj ./build.sh   # embed the kitten for demo 3
node test/verify.mjs                                         # fast solve vs original deform()
```

## Desktop version (Qt)

`deform.pro` builds the original viewer with Qt, libQGLViewer 2.7 and Eigen, linking ElasticFEM's TriMesh and LinearAlgebra (paths are set relative to this folder in `deform.pro`). The mesh path is hard-coded in the `DeformViewer` constructor in `deform.cpp`.

Controls: Shift-drag / Ctrl-drag to select, **S** to fix the selection, **W** to toggle move mode, then drag the handle.
