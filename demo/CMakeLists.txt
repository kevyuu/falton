file(GLOB TESTBED_SOURCES gl_main.cpp)

add_executable(testbed ${TESTBED_SOURCES} ${IMGUI_SOURCES})
target_link_libraries(testbed ${OPENGL_LIBRARIES} user32 gdi32 kernel32 shell32 falton ${GLFW_LIBRARY})
target_include_directories(testbed PUBLIC ${GLFW_INCLUDE_DIR} ${OPENGL_INCLUDE_DIRS} ${FALTON_INCLUDE_DIR} ${IMGUI_INCLUDE_DIR})

link_libraries(falton)
include_directories(${FALTON_INCLUDE_DIRS})

add_executable(perfmain perfmain.cpp)

add_executable(chartgen chartgen.cpp)

