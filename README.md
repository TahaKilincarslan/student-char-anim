# Character Plugin — Build & Install Guide

## Requirements

| Tool | Version |
|------|---------|
| Visual Studio 2022 (or Build Tools) | MSVC v143+ |
| CMake | 3.20+ |
| APP (Arkheon) | Licensed install |

---

## Project Structure

```
character-plugin-student/
├── src/
│   └── StudentController.cpp   ← YOUR CODE (the plugin)
├── tests/
│   └── standalone_test.cpp     ← test harness (no APP needed)
├── docs/
│   ├── design.md               ← PD gains, motion choices
│   └── ai_assistance.md        ← AI usage log
├── include/                    ← empty (SDK header comes from APP)
├── CMakeLists.txt
└── README.md                   ← this file
```

---

## Step 1 — Locate the SDK header

After installing APP, the SDK header is at:

```
C:\Program Files\APP\sdk\include\arkheon\character\ICharacterController.h
```

> The exact path may differ. Check: **APP → Settings → Plugins → Character → "SDK path"**

You do **not** copy the header into this repo — CMake points to it directly.

---

## Step 2 — Configure & Build

Open **Developer Command Prompt for VS 2022** and run:

```bat
cd character-plugin-student

cmake -B build ^
      -G "Visual Studio 17 2022" -A x64 ^
      -DARKHEON_CHAR_SDK_DIR="C:\Program Files\APP\sdk"

cmake --build build --config Release
```

The compiled DLL will be at:

```
build\Release\character_plugin_student.dll
```

---

## Step 3 — Run the Standalone Test (recommended first)

```bat
cmake --build build --config Release --target standalone_test
build\Release\standalone_test.exe
```

Expected output:
```
RESULT: PASS — 1000 ticks, all quaternions valid, timing OK
```

---

## Step 4 — Install into APP

Copy the DLL to APP's plugin folder:

```bat
copy build\Release\character_plugin_student.dll ^
     "C:\Program Files\APP\bin\plugins\character\"
```

> If the `character` folder does not exist, create it manually.

Then:
1. **Restart APP**
2. Go to **View → Logs** and confirm:
   ```
   [character] loaded plugin "Student Plugin v0.1" SDK 0x00010000
   ```

---

## Step 5 — Run a Mission

1. Open the **Character** dock window in APP
2. Mission dropdown → select `training_01`
3. Click **Start**
4. Character spawns and the 50 Hz tick begins

**Manual controls:**

| Key | Action |
|-----|--------|
| W / A / S / D | Move |
| Mouse | Look / turn |
| Left Shift | Sprint |
| Q | Motion A (walk_forward) |
| E | Motion B (push_two_handed) |
| R | Motion C (climb_low_step) |
| Esc | Cancel current mission goal |

---

## Motion Choices

| Hotkey | Clip ID | Name | Purpose |
|--------|---------|------|---------|
| Q | 12 | `walk_forward` | Locomotion base |
| E | 47 | `push_two_handed` | PUSH goals, door interaction |
| R | 83 | `climb_low_step` | CLIMB goals |

## PD Gains

See `docs/design.md` for full gain table and rationale.

Summary: Kp range 40–70, Kd range 4–7 depending on joint.  
Tighter joints (knees, ankles) use higher gains; looser joints (shoulders, hips) use lower gains.

!!My DLL files are inside the SIM folder.
