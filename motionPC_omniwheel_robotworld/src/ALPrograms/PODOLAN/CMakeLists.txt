set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/ALBuild/PODOLAN)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
add_executable(PODOLAN
		main.cpp
		LAN/RBTCPClient.cpp
		LAN/RBTCPServer.cpp
		CoreThread.cpp
		PODOServer.cpp
		ROSThread.cpp)
qt5_use_modules(PODOLAN Core Network)
target_link_libraries(PODOLAN rt)