########################################################################
### options
########################################################################
#OPTION(FOO_OPTION           "a string explaining it"     OFF)

########################################################################
### find deps
########################################################################
#AD_CHECK_PROGRAM_INSTALLED(mplayer)

########################################################################
### then add our folders
########################################################################

#add_maggie_test(twist_stamper.exe
#    SOURCES     twist_stamper.cpp
#)

#add_maggie_test(robot_wanderer.exe
#    SOURCES     robot_wanderer.cpp
#)

#add_maggie_test(robot_wanderer_with_moving_goal.exe
#    SOURCES     robot_wanderer_with_moving_goal.cpp
#    LINK        etts_api event mcp args_parser robot_config
#)

#add_maggie_test(image_subscriber_template.exe
#    SOURCES     image_subscriber_template.cpp
#    LINK        opencv
#)

#add_maggie_test(test_conditional_particle_filter.exe
#    SOURCES     test_conditional_particle_filter.cpp
#                conditional_particle_filter.h
#                conditional_particle_filter_laser.h
#                conditional_particle_filter_laser_ros.h
#    LINK
#)

################################################################################

#ADD_SUBDIRECTORY(nite)
#ADD_SUBDIRECTORY(openni)
#ADD_SUBDIRECTORY(face_recognizer)

