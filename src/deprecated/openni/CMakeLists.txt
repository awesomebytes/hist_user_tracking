
add_maggie_test(cloud_filter.exe
    SOURCES     cloud_filter.cpp
)

add_maggie_test(floor_removal.exe
    SOURCES     floor_removal.cpp
)

add_maggie_test(cluster_detector.exe
    SOURCES     cluster_detector.cpp
)

#~ add_maggie_test(kinect_template.exe
    #~ SOURCES     kinect_template.cpp
#~ )


################################################################################

add_maggie_test(fast_cluster_detector.exe
    SOURCES     fast_cluster_detector.cpp
                fast_cluster_functions.h
    LINK        disjoint_sets2
)

add_maggie_test(fast_cluster_selector.exe
    SOURCES     fast_cluster_selector.cpp
                fast_cluster_functions.h
)

################################################################################

add_maggie_test(opencv_people_detector.exe
    SOURCES     opencv_people_detector.cpp
    LINK        opencv
)

################################################################################

add_maggie_test(face_detector.exe
    SOURCES     face_detector.cpp
    LINK        opencv
)

################################################################################

add_maggie_test(test_connected_components_matcher.exe
    SOURCES     test_connected_components_matcher.cpp
                connected_components_matcher.h
    LINK        disjoint_sets2
)


### rules for PedestrianDetectionClustering
#add_maggie_lib( pedestrian_detection_clustering
#    SOURCES     pedestrian_detection_clustering.cpp
#)

#add_maggie_test(test_pedestrian_detection_clustering.exe
#    SOURCES     test_pedestrian_detection_clustering.cpp
#    LINK        pedestrian_detection_clustering
#)

