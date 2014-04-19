#include "nite_skill.h"
#include "utils/debug/error.h"

/** test */
int main(int argc, char** argv) {
    maggieDebug2("main()");
    ros::init(argc, argv, "test_nite_skill");
    NiteSkill skill;
    return 0;
}

