
rosbuild_add_executable(openni_tracker.exe openni_tracker.cpp)
set_target_properties(openni_tracker.exe PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas")

### rules for NiteSkill
add_maggie_lib( nite_skill
    SOURCES     nite_skill.cpp
    LINK
)
set_target_properties(nite_skill PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas")

add_maggie_test(test_nite_skill.exe
    SOURCES     test_nite_skill.cpp
    LINK        nite_skill
)
set_target_properties(test_nite_skill.exe PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas")

### rules for BroadcastSkeletonSkill
add_maggie_lib( broadcast_skeleton_skill
    SOURCES     broadcast_skeleton_skill.cpp
    LINK
)

add_maggie_test(test_broadcast_skeleton_skill.exe
    SOURCES     test_broadcast_skeleton_skill.cpp
    LINK        broadcast_skeleton_skill
)
