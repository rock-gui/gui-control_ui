rock_find_qt5(Core Gui Widgets)
include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_QTCORE_INCLUDE_DIR})
include_directories(${QT_QTGUI_INCLUDE_DIR})
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
    DEPS_PKGCONFIG base-types base-lib urdfdom yaml-cpp base-logging sdformat9
    LIBS Qt5::Core Qt5::Gui Qt5::Widgets
)
