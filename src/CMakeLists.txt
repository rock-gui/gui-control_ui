rock_find_qt5(Core Gui Widgets)
include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_QTCORE_INCLUDE_DIR})
include_directories(${QT_QTGUI_INCLUDE_DIR})
set(CMAKE_INCLUDE_CURRENT_DIR ON)

pkg_check_modules(SDFORMAT QUIET sdformat)
pkg_check_modules(SDFORMAT9 QUIET sdformat9)

if(${SDFORMAT_FOUND})
  set(SDFORMAT_PKGCONFIG_NAME "sdformat")
elseif(${SDFORMAT9_FOUND})
  set(SDFORMAT_PKGCONFIG_NAME "sdformat9")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
  message(SEND_ERROR "Could not find sdformat or sdformat9 with pkg-config")
endif()

SET(MOC_HDRS
    ControlUi.h
    ControlUiPlugin.h
    jointform.h
)

SET(HDRS
    ControlUi.h
    jointform.h
)

FILE(GLOB SRCS
    *.cc
    *.cpp
)

SET(QtApp_RCCS resources.qrc)
QT5_ADD_RESOURCES(QtApp_RCC_SRCS ${QtApp_RCCS})

rock_vizkit_widget(control_ui
    SOURCES ${SRCS} ${QtApp_RCC_SRCS} 
    HEADERS ${HDRS}
    MOC ${MOC_HDRS}
    UI jointform.ui
    DEPS_PKGCONFIG base-types base-lib urdfdom yaml-cpp base-logging ${SDFORMAT_PKGCONFIG_NAME}
    LIBS Qt5::Core Qt5::Gui Qt5::Widgets
)
