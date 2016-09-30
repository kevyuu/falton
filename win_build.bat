@echo off


set faltonIncludeDir=/I..\Falton\include
set faltonSources=..\Falton\src\*.cpp ..\Falton\src\container\*.cpp ..\Falton\src\collision\*.cpp ..\Falton\src\collision\broadphase\*.cpp ..\Falton\src\dynamic\*.cpp ..\Falton\src\joint\*.cpp ..\Falton\src\shape\*.cpp ..\Falton\src\*.cpp

set externalIncludeDir=/I..\external\GLFW\include /I..\external\ImGui\include
set externallib=opengl32.lib user32.lib gdi32.lib kernel32.lib shell32.lib ..\external\GLFW\lib\glfw3.lib
set cppfiles=..\src\gl_main.cpp ..\external\Imgui\src\*.c ..\external\ImGui\src\*.cpp
set outputfile=Testbed

rmdir /S /Q bin
mkdir bin

pushd bin

cl /Zi /c /MD ..\Falton\unity.cpp %faltonIncludeDir% /EHsc
lib *.obj /OUT:falton.lib

cl /Zi /EHsc %cppfiles% %externalIncludeDir% %faltonIncludeDir% /MD %externallib% falton.lib /Fe%outputfile% /link %additinalLibPath% /NODEFAULTLIB:libcmt.lib

popd