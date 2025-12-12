# ATLAS Design Manifesto

1. **Engine-first, tool-second**: ATLAS is a core library first; apps are consumers.
2. **ROS-independent by design**: no ROS required; adapters are optional.
3. **Cross-platform as a requirement**: macOS/Windows/Linux supported from day one.
4. **Performance is a feature**: zero-copy, aligned memory, pools, SPSC queues, profiling-first.
5. **Determinism & reproducibility**: record/replay and controlled execution paths.
6. **Strict separation of concerns**: IO, Core, SLAM, Viz, App layers stay independent.
7. **Minimal, stable public API**: fewer abstractions, stronger interfaces.
8. **AI as an optional plugin**: core must run without AI; ONNX-based integration later.
9. **Built-in observability**: logging, metrics, debugging tools are first-class.
10. **Light, maintainable documentation**: Doxygen for interfaces and architecture decisions only.