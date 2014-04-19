### rules for FaceRecognizer
add_maggie_lib( face_recognizer
    SOURCES     face_recognizer.cpp
    LINK
)

add_maggie_test(test_face_recognizer.exe
    SOURCES     test_face_recognizer.cpp
    LINK        face_recognizer
)
