#include "ControlBotPlugin.h"
#include "ControlBot.h"

Q_EXPORT_PLUGIN2(ControlBot, ControlBotPlugin)

ControlBotPlugin::ControlBotPlugin(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}

ControlBotPlugin::~ControlBotPlugin()
{
}

bool ControlBotPlugin::isContainer() const
{
    return false;
}

bool ControlBotPlugin::isInitialized() const
{
    return initialized;
}

QIcon ControlBotPlugin::icon() const
{
    return QIcon("");
}

QString ControlBotPlugin::domXml() const
{
        return "<ui language=\"c++\">\n"
            " <widget class=\"ControlBot\" name=\"controlbot\">\n"
            "  <property name=\"geometry\">\n"
            "   <rect>\n"
            "    <x>0</x>\n"
            "    <y>0</y>\n"
            "     <width>300</width>\n"
            "     <height>120</height>\n"
            "   </rect>\n"
            "  </property>\n"
//            "  <property name=\"toolTip\" >\n"
//            "   <string>ControlBot</string>\n"
//            "  </property>\n"
//            "  <property name=\"whatsThis\" >\n"
//            "   <string>ControlBot</string>\n"
//            "  </property>\n"
            " </widget>\n"
            "</ui>\n";
}

QString ControlBotPlugin::group() const {
    return "Rock-Robotics";
}

QString ControlBotPlugin::includeFile() const {
    return "ControlBot/ControlBot.h";
}

QString ControlBotPlugin::name() const {
    return "ControlBot";
}

QString ControlBotPlugin::toolTip() const {
    return whatsThis();
}

QString ControlBotPlugin::whatsThis() const
{
    return "";
}

QWidget* ControlBotPlugin::createWidget(QWidget *parent)
{
    return new ControlBot(parent);
}

void ControlBotPlugin::initialize(QDesignerFormEditorInterface *core)
{
     if (initialized)
         return;
     initialized = true;
}
