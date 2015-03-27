FIND_PACKAGE(Qt4 REQUIRED QtCore QtGui QtOpenGL)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

pkg_check_modules(control_ui_deps base-types base-lib QtCore QtGui urdfdom yaml-cpp)
find_library(control_ui_link_flags ${control_ui_deps_LIBRARIES} ${control_ui_deps_LIBRARY_DIRS})

include_directories(${control_ui_deps_INCLUDE_DIRS})

add_executable(test_control_ui test_control_ui.cpp)
target_link_libraries(test_control_ui control_ui ${control_ui_link_flags} ${QT_LIBRARIES})
