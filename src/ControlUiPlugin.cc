#include "ControlUiPlugin.h"
#include "ControlUi.h"

Q_EXPORT_PLUGIN2(ControlUi, ControlUiPlugin)

ControlUiPlugin::ControlUiPlugin(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}

ControlUiPlugin::~ControlUiPlugin()
{
}

bool ControlUiPlugin::isContainer() const
{
    return false;
}

bool ControlUiPlugin::isInitialized() const
{
    return initialized;
}

QIcon ControlUiPlugin::icon() const
{
    return QIcon("");
}

QString ControlUiPlugin::domXml() const
{
        return "<ui language=\"c++\">\n"
            " <widget class=\"ControlUi\" name=\"controlbot\">\n"
            "  <property name=\"geometry\">\n"
            "   <rect>\n"
            "    <x>0</x>\n"
            "    <y>0</y>\n"
            "     <width>300</width>\n"
            "     <height>120</height>\n"
            "   </rect>\n"
            "  </property>\n"
//            "  <property name=\"toolTip\" >\n"
//            "   <string>ControlUi</string>\n"
//            "  </property>\n"
//            "  <property name=\"whatsThis\" >\n"
//            "   <string>ControlUi</string>\n"
//            "  </property>\n"
            " </widget>\n"
            "</ui>\n";
}

QString ControlUiPlugin::group() const {
    return "Rock-Robotics";
}

QString ControlUiPlugin::includeFile() const {
    return "ControlUi/ControlUi.h";
}

QString ControlUiPlugin::name() const {
    return "ControlUi";
}

QString ControlUiPlugin::toolTip() const {
    return whatsThis();
}

QString ControlUiPlugin::whatsThis() const
{
    return "";
}

QWidget* ControlUiPlugin::createWidget(QWidget *parent)
{
    return new ControlUi(parent);
}

void ControlUiPlugin::initialize(QDesignerFormEditorInterface *core)
{
     if (initialized)
         return;
     initialized = true;
}
