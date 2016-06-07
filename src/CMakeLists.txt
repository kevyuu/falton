file (GLOB_RECURSE SRC imgui/*.cpp)
file (GLOB_RECURSE SRC window/*.cpp)
file (GLOB_RECURSE IMGUI_SOURCES imgui/*.cpp imgui/*.h)
file (GLOB_RECURSE GUI_SOURCES window/*.cpp window/*.h)

add_executable(main main.cpp ${IMGUI_SOURCES} ${GUI_SOURCES} Demo.h DrawPhysics.h window/DrawerConfigWindow.h)


