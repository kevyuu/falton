rm -r bin
mkdir bin

pushd bin

g++ -std=c++11 -Wall -g -c ../Falton/unity.cpp -I../Falton/include
ar rcs libfalton.a *.o

EXTERNAL_LIB="-ldl -lglfw3 -lGL -lX11 -lXi -lXrandr -lXxf86vm -lXinerama -lXcursor -lrt -lm -pthread"

g++ -std=c++11 -g \
    ../demo/gl_main.cpp \
    ../external/ImGui/src/*.c \
    ../external/ImGui/src/*.cpp \
    libfalton.a \
    -I../external/GLFW/include \
    -I../external/ImGui/include \
    -I../Falton/include \
    $EXTERNAL_LIB
popd