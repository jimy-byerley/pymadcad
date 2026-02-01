# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## migration to rust

- 1. add support for rust in library build
- 2. reimplement few python/cython functions in pure rust
- 3. wrap rust function to python, introduce needed conversions
- 4. fix library build

## Build & Test Commands

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Build and install Rust extension in development mode
maturin develop

# Build release wheel
maturin build --release

# Run all tests
pytest tests/

# Run a specific test file
pytest tests/test_mesh.py -v

# Run a specific test
pytest tests/test_mesh.py::test_function_name -v

# Rust-only checks
cargo fmt --check
cargo clippy
cargo test
```

## Project Structure

```
pymadcad/
├── Cargo.toml          # Rust crate configuration
├── pyproject.toml      # Python package config (maturin backend)
├── src/                # Rust source code
│   └── lib.rs          # Rust module entry point → madcad/core.so
├── madcad/             # Python package
│   ├── __init__.py
│   ├── core.pyi        # Type stubs for Rust module (optional)
│   ├── mesh/           # Mesh data structures
│   ├── boolean.py      # Boolean operations (migrating to Rust)
│   └── ...
└── tests/
```

The Rust extension compiles to `madcad/core.so` and is imported as `from madcad import core`.

## Architecture Overview

PyMADCAD is a script-based CAD library (not parametric). The module dependency hierarchy flows downward with no circular dependencies:

### Core Layer
- **core** (Rust) - Performance-critical operations: boolean, triangulation, spatial hashing
- **mathutils** - Extends pyglm types (vec3, mat4, quat) with CAD-specific operations
- **mesh/** - Geometry containers with shared buffer references:
  - `Mesh` - 2D topology (triangulated surfaces with faces + points)
  - `Web` - 1D topology (edge networks)
  - `Wire` - 0D topology (point sequences, indices only)

### Generation Layer
- **primitives** - Parametric curves: `Segment`, `ArcThrough`, `ArcCentered`, `Circle`, `Ellipsis`, `Interpolated`
- **generation** - Surface operations: extrusion, revolution, tube, helix, screw, plus standard shapes (brick, cylinder, cone, icosahedron)

### Operations Layer
- **boolean** - Union/intersection/difference using "syandana" algorithm with spatial hashing
- **constraints** - Solver system: `Tangent`, `Distance`, `Angle`, `Parallel`, `Radius`, `PointOn`, `OnPlane`
- **triangulation**, **bevel**, **blending**, **offseting** - Mesh processing operations

### Higher-Level
- **kinematic/** - Joint systems and Screw theory for movement/forces
- **assembly** - `Solid` class for organizing parts with pose matrices
- **rendering** - OpenGL via moderngl, Qt integration in `qt/` module
- **standard** - ISO/metric parts (nuts, screws, bearings)

## Key Design Principles

**Data Ownership**: Mesh classes reference buffers, they don't own them. Buffers are often shared between `Mesh`, `Web`, and `Wire` instances. Copy explicitly when needed.

**Rust/Python Buffer Sharing**: Python mesh buffers (arrex typedlist) can be passed to Rust without copying. Rust code should borrow when possible, only copy when modification is needed.

**Polymorphism via Duck Typing**:
- Primitives implement `mesh()` and `slvvars` for solving
- Constraints implement `fit()` returning squared error and `slvvars`
- Displayable objects implement `display(scene)`

**In-place vs New**: Methods modify in-place for small changes, return new instances for large changes.

## Code Style

### Python
- snake_case for functions/variables, CamelCase for classes (except common ones like `vec3`)
- Tab indentation
- No 80-column limit - prioritize readability
- Keep names short but unambiguous - use keywords, not abbreviations or synonyms

### Rust
- Standard Rust conventions (rustfmt)
- Use `vecmat` crate for linear algebra (Vec3<f64> matches Python's dvec3)
- PyO3 for Python bindings

## Dependencies

**Python**: `pyglm~=2.5`, `numpy`, `scipy~=1.3`, `moderngl~=5.6`, `arrex~=0.5.1`
**Rust**: `pyo3`, `vecmat`, `rustc-hash`
**Optional**: `plyfile` (PLY), `numpy-stl` (STL), `pywavefront` (OBJ), `pyside6`/`pyqt6`/`pyqt5` (GUI)
