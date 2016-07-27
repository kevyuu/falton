file (GLOB_RECURSE SRC imgui/*.cpp)
file (GLOB_RECURSE SRC window/*.cpp)
file (GLOB_RECURSE IMGUI_SOURCES imgui/*.cpp imgui/*.h)
file (GLOB_RECURSE GUI_SOURCES window/*.cpp window/*.h)

add_executable(main main.cpp ${IMGUI_SOURCES} ${GUI_SOURCES} Demo.h DrawPhysics.h window/DrawerConfigWindow.h)

target_link_libraries(main allegro_acodec
        allegro_audio
        allegro_color
        allegro_dialog
        allegro_image
        allegro_main
        allegro_memfile
        allegro_physfs
        allegro_primitives
        allegro_ttf
        allegro_font
        allegro)

add_executable(perfmain perfmain.cpp)

add_executable(chartgen chartgen.cpp)

