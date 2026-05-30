# AI Assistance Log

This file documents AI assistant usage as required by §11 of the course spec.

## Session 1

**Tool used:** Claude (claude.ai)

**Prompts given:**
- Asked Claude to read the spec and generate the full project file structure including `StudentController.cpp`, `CMakeLists.txt`, `standalone_test.cpp`, `design.md`, and `README.md`.

**What was accepted:**
- Overall file structure and CMake setup
- The PD controller formula structure (`τ = Kp * err − Kd * (ω_cur − ω_ref)`)
- Quaternion math helpers (normalize, slerp, mul, conjugate)
- Locomotion direction calculation (yaw-based forward/right vectors)
- Standalone test harness structure with timing and quaternion validation

**What was rejected / modified:**
- PD gains were reviewed against the spec's suggested starting values (`Kp ≈ 50`, `Kd ≈ 5` from §14 step 6) and adjusted per joint based on biomechanical reasoning
- Mission goal handling was reviewed manually against the spec's goal-type semantics table (§10)

**Why AI was used:**
- To accelerate the boilerplate setup and ensure ABI compliance with the spec
- The physics concepts (PD controller, quaternion integration) were taught in course week 9–10 lectures and are understood independently
