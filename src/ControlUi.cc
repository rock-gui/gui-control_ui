#include <urdf_parser/urdf_parser.h>
#include "ControlUi.h"
#include <base/Logging.hpp>
#include <fstream>

ControlUi::ControlUi(QWidget *parent)
    : QWidget(parent)
{
    resize(300,120);

    show();
}

ControlUi::~ControlUi()
{
}

const base::commands::Joints& ControlUi::getJoints()
{
    return currentJoints_;
}

void ControlUi::handleNewVal(double val)
{
    QString name = QObject::sender()->property("name").toString();

    //Update currentJoints_
    size_t idx;
    try{
        idx = currentJoints_.mapNameToIndex(name.toLatin1().data());
    }
    catch(base::commands::Joints::InvalidName ex){
        LOG_ERROR("Invalid name was passed from QObject. This should never happen, probably there is something wrong in initModel. Expeption: %s", ex.what());
        return;
    }

    val = val * 0.0174532925; //Degrees to radian;

    currentJoints_[idx].position = val;
    currentJoints_.time = base::Time::now();


    //Create joint command from base-types
    base::commands::Joints val_joint_command;
    val_joint_command.resize(1);
    val_joint_command.names[0] = name.toLatin1().data();
    val_joint_command[0].position = val;

    //Tell everyone that there's something new
    emit(newVal(name, val));
    emit(newVal(val_joint_command));
}


void ControlUi::initModel(QString filepath)
{
    //Read the urdf file
    std::ifstream file(filepath.toStdString().c_str());
    std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml);

    //Set up user interface
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
    QGridLayout *layout = new QGridLayout;
    int i=0;
    const int columns=5;
    for (it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it){
        boost::shared_ptr<urdf::Joint> joint = it->second;
        std::string name = it->first;
        if(joint->type != urdf::Joint::FIXED){
            //Create user interface elements
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

            LOG_DEBUG("Created GUI elements for %s", name.c_str());

            //Fill current joint configuration
            currentJoints_.names.push_back(name);
            currentJoints_.elements.push_back(base::JointState());
            currentJoints_.elements.back().position = val->value() * 0.0174532925; //Degrees to radian

            i++;
        }
    }
    setLayout(layout);
}
