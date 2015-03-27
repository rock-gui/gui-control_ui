#include "../src/ControlUi.h"

int main(int argc, char** argv){
    QApplication app(argc, argv);
    ControlUi* widget = new ControlUi();
    widget->initFromURDF("../../test_data/spacebot_arm/spacebot_arm.urdf");
    widget->show();

    app.exec();
    delete widget;
}
