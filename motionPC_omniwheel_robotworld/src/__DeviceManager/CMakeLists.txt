set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exe)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORRC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Sql REQUIRED)
add_definitions(-D_GNU_SOURCE -D_REENTRANT -D__COBALT__)
add_executable(DeviceManager
		main.cpp
		MainWindow.cpp
		Panel_MC.cpp
		RBCAN.cpp
		RBDataBase.cpp
		RBMotorController.cpp
		Panel_FT.cpp
		RBFTSensor.cpp
		RBGraph.cpp
		qcustomplot.cpp
		Panel_FT.ui
		Panel_MC.ui
		RBGraph.ui
		MainWindow.ui)
qt5_use_modules(DeviceManager Core Sql Widgets PrintSupport)
target_include_directories(DeviceManager PRIVATE /usr/xenomai/include /usr/xenomai/include/cobalt)
find_library(COPPERPLATE copperplate /usr/xenomai/lib)
find_library(COBALT cobalt /usr/xenomai/lib)
find_library(ALCHEMY alchemy /usr/xenomai/lib)
target_link_libraries(DeviceManager ${ALCHEMY} ${COPPERPLATE} ${COBALT} pthread rt pcan)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld")