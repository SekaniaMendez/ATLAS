# ATLAS

**ATLAS** is a high-performance, cross-platform **LiDAR mapping and SLAM engine** designed to run **without ROS**, with a strong focus on **topographic-grade accuracy**, **low-level optimization**, and long-term **Computer Science / AI research value**.

ATLAS is developed as a **solo-developer research & engineering project**, but built with a clean, modular architecture suitable for collaboration, academic work, and future commercialization.

---

## Project Vision

ATLAS aims to own the full pipeline from **raw sensor data** to **georeferenced 3D deliverables** suitable for real-world use:

- Topography and surveying
- Civil engineering and infrastructure inspection
- Robotics and autonomous systems
- 3D reconstruction and visualization
- Research in SLAM, sensor fusion, and 3D perception

Unlike many existing solutions, ATLAS intentionally avoids heavy middleware dependencies and prioritizes:

- **Deterministic performance**
- **Explicit control over memory and data flow**
- **Cross-platform portability (macOS, Linux, Windows)**
- **Hardware-aware optimization**

---

## Core Objectives

### 1) LiDAR Driver Layer (ROS-independent)

- Direct integration with vendor SDKs (starting with **Livox Mid-360 / Livox-SDK2**)
- No ROS dependency
- Explicit control over:
  - network sockets
  - packet parsing
  - timestamp handling
- Support for:
  - point cloud packets
  - IMU packets
  - multi-LiDAR (future)

This layer is the **foundation** for everything else.

---

### 2) High-Performance LiDAR–Inertial SLAM

ATLAS targets a modern LiDAR–IMU pipeline inspired by state-of-the-art systems (e.g., FAST-LIO / FAST-LIO2 class of methods):

- tight LiDAR + IMU fusion
- incremental state estimation
- real-time operation
- deterministic memory usage

The SLAM core is written in **modern C++** and structured so internal components can be replaced and/or optimized independently.

---

### 3) Topographic-Grade Accuracy & GNSS Integration

A central goal of ATLAS is to operate at **surveying/topography precision levels**.

Planned features:

- Integration with **professional GNSS receivers** (e.g., CHCNAV i73)
- Support for:
  - RTK Fixed / Float
  - DGNSS
  - PPP (future)
- Proper coordinate frame handling:
  - local ENU
  - projected systems
  - global geodetic coordinates

This enables **georeferenced point clouds** directly usable in CAD/GIS workflows.

---

### 4) Ultra-Low-Level Optimization (Multi-Architecture)

ATLAS will explore performance optimization at multiple levels:

#### C++ level
- cache-friendly data structures
- explicit memory ownership
- minimal overhead abstractions

#### SIMD / vectorization
- compiler intrinsics where beneficial
- alignment-aware math routines

#### Assembly-level optimization
- architecture-specific code paths via conditional compilation
- target architectures:
  - **x86-64** (SSE/AVX/AVX2, optionally AVX-512 where available)
  - **ARM64** (NEON)

The intent is to push deterministic, real-time throughput by controlling the entire stack.

---

### 5) Artificial Intelligence & 3D Perception

ATLAS is not limited to geometric SLAM. A major long-term objective is integrating **AI-based 3D understanding**, including:

- semantic segmentation of point clouds
- object filtering (people, vehicles, furniture, vegetation)
- structural classification (walls, floors, terrain)
- change detection between scans

The AI pipeline is designed to:

- be optional (not required for core SLAM)
- run locally (offline-capable)
- be deployable on embedded systems

---

### 6) Visualization & Playback

ATLAS will provide multiple visualization paths:

- lightweight internal viewers (debug/development)
- export to standard formats (LAS/LAZ, PCD, etc.)
- optional **Unreal Engine** integration for:
  - high-fidelity visualization
  - path replay
  - simulation and analysis

---

## Repository Layout

```
ATLAS/
├── core/          # math, geometry, time, utilities
├── modules/       # lidar, gnss, slam, mapping modules
├── apps/          # executables (probes, tools, demos)
├── third_party/   # vendor SDKs (as submodules)
├── tools/         # offline processing utilities
└── docs/          # technical documentation
```

---

## Status

- [x] Cross-platform build system (CMake + Ninja)
- [x] Vendor SDK integration (Livox-SDK2)
- [x] Hardware communication probe (device discovery / IMU enable)
- [ ] Real-time point cloud ingestion pipeline
- [ ] LiDAR–Inertial SLAM core
- [ ] GNSS fusion & georeferencing
- [ ] AI perception modules
- [ ] Visualization pipeline

---

## Research & Academic Relevance

ATLAS is suitable as a foundation for a Bachelor/Master thesis in **Computer Science / AI**, and for research topics such as:

- real-time SLAM
- sensor fusion
- high-performance computing
- embedded perception systems

The project prioritizes **clarity, documentation, and reproducibility** over quick demos.

---

## Philosophy

> **Own the entire stack.**

ATLAS is built on the idea that:

- understanding performance requires controlling all layers
- clean architecture outlives short-term frameworks
- research code should be readable, testable, and explainable

---

## License

ATLAS is licensed under the repository license.

Some submodules (e.g., vendor SDKs) are subject to their own licenses.