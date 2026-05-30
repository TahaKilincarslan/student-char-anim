// tests/standalone_test.cpp
// Standalone harness — NO APP required.
//
// Build with CMake (recommended):
//   cmake -B build -DARKHEON_CHAR_SDK_DIR="C:/path/to/sdk"
//   cmake --build build --config Release --target standalone_test
//   build\Release\standalone_test.exe
//
// Or directly with MSVC:
//   cl /std:c++17 /EHsc /I"%ARKHEON_CHAR_SDK_DIR%\include" ^
//      tests\standalone_test.cpp src\StudentController.cpp /Fe:standalone_test.exe

#include "arkheon/character/ICharacterController.h"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <cstring>

// ─── Mock env_api ─────────────────────────────────────────────────────────────

static int32_t mock_raycast(void*, arkheon_vec3, arkheon_vec3, float,
                             arkheon_vec3* h, arkheon_vec3* n, int32_t* id) {
    if (h) *h = {0, 0, 0};
    if (n) *n = {0, 1, 0};
    if (id) *id = -1;
    return 0;
}

static int32_t mock_aabb(void*, int32_t, arkheon_vec3* mn, arkheon_vec3* mx) {
    if (mn) *mn = {-1, -1, -1};
    if (mx) *mx = {1,  1,  1};
    return 1;
}

static int32_t mock_find(void*, const char*) { return 0; }

static int32_t mock_nav(void*, arkheon_vec3 /*from*/, arkheon_vec3 to,
                         arkheon_vec3* path, int32_t /*max*/) {
    if (path) path[0] = to;
    return 1;
}

static void mock_done(void*, int32_t seq, int32_t res) {
    std::printf("[goal] sequence_id=%d completed, result=%d\n", seq, res);
}

static arkheon_vec3 mock_grav(void*) { return {0, -9.81f, 0}; }

// ─── Helpers ──────────────────────────────────────────────────────────────────

static bool is_unit_quat(const arkheon_quat& q) {
    float n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(n2) && n2 >= 0.5f && n2 <= 1.5f;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main() {
    std::printf("=== Arkheon Character Plugin — Standalone Test ===\n\n");

    // 1. SDK version handshake
    uint32_t ver = arkheon_character_sdk_version();
    std::printf("[check] SDK version = 0x%08x  (expected 0x%08x)\n",
                ver, ARKHEON_CHARACTER_SDK_VERSION);
    if (ver != ARKHEON_CHARACTER_SDK_VERSION) {
        std::printf("FAIL: SDK version mismatch\n");
        return 1;
    }

    // 2. Plugin name
    const char* name = arkheon_character_plugin_name();
    std::printf("[check] plugin name = \"%s\"\n", name);

    // 3. Motion clips
    int32_t clips[3] = {-1, -1, -1};
    arkheon_character_get_motion_clips(nullptr, clips);
    std::printf("[check] motion clips = [%d, %d, %d]  (expected [12, 47, 83])\n",
                clips[0], clips[1], clips[2]);

    // 4. Create controller
    float segs[10] = {
        0.0f,       0.0f,       // upperarm_l/r  (no segment above shoulder)
        0.296595f,  0.296595f,  // lowerarm_l/r
        0.0f,       0.0f,       // thigh_l/r     (root to hip)
        0.406626f,  0.406626f,  // calf_l/r
        0.433194f,  0.433194f   // foot_l/r
    };
    void* h = arkheon_character_create(segs);
    if (!h) {
        std::printf("FAIL: arkheon_character_create returned null\n");
        return 1;
    }
    std::printf("[check] create OK\n");

    // 5. Build mock environment
    arkheon_env_api env = {};
    env.host_ctx           = nullptr;
    env.raycast            = mock_raycast;
    env.get_object_aabb    = mock_aabb;
    env.find_object_by_name= mock_find;
    env.navmesh_query      = mock_nav;
    env.report_goal_complete = mock_done;
    env.get_gravity        = mock_grav;

    // 6. Identity bone array (rest pose — all rotations are identity quaternions)
    arkheon_bone_state bones[66] = {};
    for (auto& b : bones) {
        b.local_rotation    = {0, 0, 0, 1};
        b.local_translation = {0, 0, 0};
        b.world_position    = {0, 0, 0};
    }

    // 7. Input state — simulate pressing W for half the ticks
    arkheon_input_state input = {};

    // 8. A simple GOTO goal
    arkheon_mission_goal goal = {};
    goal.sequence_id    = 1;
    goal.type           = ARK_GOAL_GOTO;
    goal.target_position= {3.0f, 0.0f, 5.0f};
    goal.tolerance_m    = 0.3f;
    goal.timeout_s      = 30.0f;
    goal.target_object_id = -1;

    // 9. Run 1000 ticks and measure time
    arkheon_frame frame = {};
    frame.delta_time_s  = 0.02;
    frame.is_paused     = 0;

    constexpr int TICK_COUNT = 1000;
    double total_us = 0.0;
    double max_us   = 0.0;
    int    fail_tick = -1;

    std::printf("\n[run ] Running %d ticks...\n", TICK_COUNT);

    for (int i = 0; i < TICK_COUNT; ++i) {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number      = (uint64_t)i;

        // Simulate W key held for first 500 ticks
        input.keys[0x1A] = (i < 500) ? 1 : 0;  // HID_W
        // Test hotkey on tick 100
        input.hotkey_motion_a = (i == 100) ? 1 : 0;
        input.hotkey_motion_b = (i == 300) ? 1 : 0;
        input.hotkey_motion_c = (i == 600) ? 1 : 0;
        input.look_yaw_rad    = (float)i * 0.002f;  // slowly rotate

        arkheon_bone_override out[10] = {};
        arkheon_vec3 dt_pos = {0, 0, 0};
        arkheon_quat dt_rot = {0, 0, 0, 1};

        auto t0 = std::chrono::high_resolution_clock::now();

        int rc = arkheon_character_tick(h, &frame, bones, out,
                                         &dt_pos, &dt_rot,
                                         &input, &goal, &env);

        auto t1 = std::chrono::high_resolution_clock::now();
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        total_us += us;
        if (us > max_us) max_us = us;

        if (rc != 0) {
            std::printf("FAIL: tick %d returned rc=%d\n", i, rc);
            fail_tick = i;
            break;
        }

        // Validate quaternions
        for (int j = 0; j < 10; ++j) {
            if (out[j].apply && !is_unit_quat(out[j].local_rotation)) {
                std::printf("FAIL: bad quaternion at tick %d joint %d "
                            "(x=%.4f y=%.4f z=%.4f w=%.4f)\n",
                            i, j,
                            out[j].local_rotation.x, out[j].local_rotation.y,
                            out[j].local_rotation.z, out[j].local_rotation.w);
                fail_tick = i;
                break;
            }
        }
        if (fail_tick >= 0) break;
    }

    // 10. Destroy
    arkheon_character_destroy(h);
    std::printf("[check] destroy OK\n\n");

    // 11. Results
    double avg_us = total_us / TICK_COUNT;
    double avg_ms = avg_us / 1000.0;
    double max_ms = max_us / 1000.0;

    std::printf("=== Results ===\n");
    std::printf("  Ticks run       : %d\n", (fail_tick >= 0) ? fail_tick : TICK_COUNT);
    std::printf("  Avg tick time   : %.3f ms  (budget: < 18 ms)\n", avg_ms);
    std::printf("  Max tick time   : %.3f ms  (budget: < 20 ms)\n", max_ms);

    if (fail_tick >= 0) {
        std::printf("\nRESULT: FAIL (first failure at tick %d)\n", fail_tick);
        return 1;
    }

    if (avg_ms >= 18.0) {
        std::printf("\nRESULT: FAIL (avg tick time exceeds 18 ms budget)\n");
        return 1;
    }

    std::printf("\nRESULT: PASS — %d ticks, all quaternions valid, timing OK\n",
                TICK_COUNT);
    return 0;
}
