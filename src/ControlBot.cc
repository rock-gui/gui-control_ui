#include "ControlBot.h"

ControlBot::ControlBot(QWidget *parent)
    : QWidget(parent)
{
    resize(300,120);

    show();
}

ControlBot::~ControlBot()
{
}

void ControlBot::handleNewVal(double val)
{
    QString name = QObject::sender()->property("name").toString();

    //Create joint command from base-types
    base::commands::Joints val_joint_command;
    val_joint_command.resize(1);
    val_joint_command.names[0] = name.toLatin1().data();
    val_joint_command.states[0].position = val;

    //Tell everyone that there's something new
    emit(newVal(name, rad(val)));
    emit(newVal(val_joint_command));
}


void ControlBot::initModel(QString filepath)
{
    //Read the urdf file
    urdf::Model urdf_model;
    urdf_model.initFile(filepath.toLatin1().data());

    //Set up user interface
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
    QGridLayout *layout = new QGridLayout;
    int i=0;
    const int columns=5;
    for (it=urdf_model.joints_.begin(); it!=urdf_model.joints_.end(); ++it){
        boost::shared_ptr<urdf::Joint> joint = it->second;
        std::string name = it->first;
        if(joint->type != urdf::Joint::FIXED){
            QDoubleSpinBox *val = new QDoubleSpinBox();
            val->setProperty("name", QString(name.c_str()));
            if(joint->limits){
                val->setMinimum(deg(joint->limits->lower));
                val->setMaximum(deg(joint->limits->upper));
            }
            else{
                val->setMinimum(-360);
                val->setMaximum(360);
            }
            val->setSingleStep(1);
            val->setValue(0);
            connect(val, SIGNAL(valueChanged(double)), this, SLOT(handleNewVal(double)));

            QLabel *label = new QLabel(QString(name.c_str()));
            label->setAlignment(Qt::AlignRight);

            int row=i/columns;
            int column=i%columns;
            layout->addWidget(label, row, column*2);
            layout->addWidget(val, row, column*2+1);

            i++;
        }
    }
    setLayout(layout);
}
