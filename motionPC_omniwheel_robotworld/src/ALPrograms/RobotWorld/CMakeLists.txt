set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
add_executable(OmniWheel main.cpp
		joint.cpp
		BasicTrajectory.cpp
		taskmotion.cpp
		ManualCAN.cpp)
find_library(COPPERPLATE copperplate /usr/xenomai/lib)
find_library(COBALT cobalt /usr/xenomai/lib)
find_library(ALCHEMY alchemy /usr/xenomai/lib)
find_library(IK_MATH2 NAMES libik_math2.a libik_math2 PATHS ${podo_SOURCE_DIR}/share/Libs)
find_library(KINE_DRC_HUBO2 NAMES libKINE_DRC_HUBO2.a KINE_DRC_HUBO2 PATHS ${podo_SOURCE_DIR}/share/Libs)
target_include_directories(OmniWheel PRIVATE /usr/xenomai/include /usr/xenomai/include/cobalt)
qt5_use_modules(OmniWheel Core)
target_link_libraries(OmniWheel ${ALCHEMY} ${COPPERPLATE} ${COBALT} pthread rt pcan ${KINE_DRC_HUBO2} ${IK_MATH2})
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld")