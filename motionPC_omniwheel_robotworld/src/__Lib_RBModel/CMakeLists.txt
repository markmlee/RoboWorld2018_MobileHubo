set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5OpenGL REQUIRED)
add_library(RBModel STATIC
		isnl_util_skeleton.cpp
		isnl_opengl_globject.cpp
		isnl_opengl_glskeleton.cpp
		isnl_opengl_globject_STL.cpp
		isnl_util_reqserver.cpp
		tinythread.cpp)
qt5_use_modules(RBModel OpenGL)