set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exe)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORRC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Sql REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5OpenGL REQUIRED)
add_executable(PODOGUI
		main.cpp
		GUIMainWindow.cpp
		LAN/RBTCPClient.cpp
		LAN/RBTCPServer.cpp
		BasicFiles/RBDataBase.cpp
		BasicFiles/LANDialog.cpp
		BasicFiles/PODOALDialog.cpp
		BasicFiles/JointDialog.cpp
		BasicFiles/SensorDialog.cpp
		BasicFiles/ModelDialog.cpp
		BasicFiles/SettingDialog.cpp
		TutorialDialog.cpp
		omniwheeldialog.cpp
		walkingdialog.cpp
		joystickdialog.cpp
		JoyStick/RBJoystick.cpp
		cartaskdialog.cpp
		GUIMainWindow.ui
		BasicFiles/LANDialog.ui
		BasicFiles/PODOALDialog.ui
		BasicFiles/JointDialog.ui
		BasicFiles/SensorDialog.ui
		BasicFiles/ModelDialog.ui
		BasicFiles/SettingDialog.ui
		TutorialDialog.ui
		omniwheeldialog.ui
		walkingdialog.ui
		joystickdialog.ui
		cartaskdialog.ui)
qt5_use_modules(PODOGUI Core Widgets Sql Network OpenGL)
target_link_libraries(PODOGUI GL RBModel pthread)
include_directories(../__Lib_RBModel)
add_custom_command(TARGET PODOGUI POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/share/GUI ${CMAKE_BINARY_DIR}/share/GUI)
add_custom_command(TARGET PODOGUI POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/share/configs ${CMAKE_BINARY_DIR}/exe/configs)