set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exe)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Sql REQUIRED)
add_definitions(-D_GNU_SOURCE -D_REENTRANT -D__COBALT__)
add_executable(Daemon
		main.cpp
		RBDataBase.cpp
		RBMotorController.cpp
		RBCAN.cpp
		RBProcessManager.cpp
		RBFTSensor.cpp
		RBIMUSensor.cpp
		RBSmartPower.cpp
		RBOpticFlowSensor.cpp
		RBFOGSensor.cpp
		RBRawLAN.cpp)
qt5_use_modules(Daemon Core Sql)
target_include_directories(Daemon PRIVATE /usr/xenomai/include /usr/xenomai/include/cobalt)
find_library(COPPERPLATE copperplate /usr/xenomai/lib)
find_library(COBALT cobalt /usr/xenomai/lib)
find_library(ALCHEMY alchemy /usr/xenomai/lib)
target_link_libraries(Daemon ${ALCHEMY} ${COPPERPLATE} ${COBALT} pthread rt pcan)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld")
add_custom_command(TARGET Daemon POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/exe/Core_Config.db ${CMAKE_BINARY_DIR}/exe)