#include "../src/ControlUi.h"

int main(int argc, char** argv){
    QApplication app(argc, argv);
    ControlUi* widget = new ControlUi();
    //widget->configureUi(0,false,true,true);
    widget->initFromURDF("../../test_data/spacebot_arm/spacebot_arm.urdf");
    widget->show();

    app.exec();
    delete widget;
}
