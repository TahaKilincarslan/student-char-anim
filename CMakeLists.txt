cmake_minimum_required(VERSION 3.20)
project(student_character_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── SDK header path ──────────────────────────────────────────────────────────
# Pass via:  cmake -DARKHEON_CHAR_SDK_DIR="C:/path/to/sdk" ..
# OR set an environment variable: set ARKHEON_CHAR_SDK_DIR=C:\path\to\sdk

if(NOT DEFINED ARKHEON_CHAR_SDK_DIR)
    set(ARKHEON_CHAR_SDK_DIR "$ENV{ARKHEON_CHAR_SDK_DIR}")
endif()

if(NOT EXISTS "${ARKHEON_CHAR_SDK_DIR}/include/arkheon/character/ICharacterController.h")
    message(FATAL_ERROR
        "ARKHEON_CHAR_SDK_DIR is not set or the header is missing.\n"
        "Set it with: cmake -DARKHEON_CHAR_SDK_DIR=C:/path/to/sdk .."
    )
endif()

# ─── DLL target ───────────────────────────────────────────────────────────────
add_library(student_character_plugin SHARED
    src/StudentController.cpp
)

target_include_directories(student_character_plugin PRIVATE
    ${ARKHEON_CHAR_SDK_DIR}/include
)

# Output name: replace <student_id> with your real student ID
set_target_properties(student_character_plugin PROPERTIES
    OUTPUT_NAME "character_plugin_student"
    PREFIX ""        # no "lib" prefix
    SUFFIX ".dll"
)

# MSVC-specific: disable RTTI and exceptions across ABI boundary is fine
# (we keep /EHsc internally for safe C++ usage within the DLL)
if(MSVC)
    target_compile_options(student_character_plugin PRIVATE
        /W4        # high warning level
        /WX-       # warnings not errors (relax for student build)
        /EHsc      # standard exception handling
        /utf-8     # UTF-8 source
    )
endif()

# ─── Standalone test harness ──────────────────────────────────────────────────
add_executable(standalone_test
    tests/standalone_test.cpp
    src/StudentController.cpp   # link source directly (no DLL needed for test)
)

target_include_directories(standalone_test PRIVATE
    ${ARKHEON_CHAR_SDK_DIR}/include
)

if(MSVC)
    target_compile_options(standalone_test PRIVATE /W4 /WX- /EHsc /utf-8)
endif()
