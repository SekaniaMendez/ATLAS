# ATLAS Design Manifesto

This manifesto defines the long-term architectural, engineering, and research  
principles of the ATLAS project. These rules are intentional and opinionated;  
they exist to keep the system understandable, performant, and academically  
rigorous over time.

1. **Engine-first, tool-second**  
   ATLAS is a core engine and research platform first. Applications, demos, and  
   tools are consumers of the engine, never the other way around.

2. **ROS-independent by design**  
   ATLAS does not require ROS. Adapters may exist, but the core must remain  
   standalone, lightweight, and framework-agnostic.

3. **Cross-platform as a requirement**  
   macOS, Linux, and Windows are first-class targets from day one. No platform  
   is treated as an afterthought.

4. **Performance is a feature**  
   Deterministic performance matters. Zero-copy paths, aligned memory,  
   preallocated pools, SPSC queues, and profiling-first development are  
   preferred over convenience abstractions.

5. **Determinism & reproducibility**  
   ATLAS must support controlled execution paths, record/replay, and  
   reproducible experiments suitable for academic evaluation.

6. **Strict separation of concerns**  
   IO, Core, SLAM, GNSS, AI, Visualization, and Application layers remain  
   logically and physically independent.

7. **Minimal, stable public API**  
   Fewer abstractions with stronger contracts are preferred. Public interfaces  
   should change rarely and be documented rigorously.

8. **Topographic-grade accuracy is a goal**  
   The system is designed to support LiDAR–IMU–GNSS fusion with professional  
   receivers and RTK-level precision, suitable for surveying and civil use.

9. **AI as an optional plugin**  
   The core engine must function without AI. AI-based perception is treated as  
   an optional, pluggable layer (e.g., ONNX-based) that enhances—not defines—the  
   system.

10. **Built-in observability**  
    Logging, metrics, diagnostics, and debugging facilities are first-class  
    features, not afterthoughts.

11. **Research-grade codebase**  
    ATLAS is written with academic scrutiny in mind. Algorithms, assumptions,  
    and trade-offs should be explicit, traceable, and explainable.

12. **Architecture-aware optimization**  
    Performance work may include SIMD and assembly-level optimization with  
    explicit support for multiple architectures (x86-64 and ARM64) via  
    conditional compilation.

13. **Visualization is not the engine**  
    High-level visualization (including Unreal Engine) is optional and  
    external. It must never leak dependencies or design constraints into the  
    core engine.

14. **Small, readable source files**  
    No production source file should exceed ~400 lines of code. Large logic  
    must be decomposed into smaller, focused translation units to preserve  
    readability, reviewability, and long-term maintainability.

15. **Documentation with intent**  
    Doxygen is used for public interfaces, core architecture, and design  
    decisions. Documentation should explain *why* something exists, not merely  
    *what* it does.