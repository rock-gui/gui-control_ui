#include <urdf_parser/urdf_parser.h>
#include "ControlUi.h"
#include <base/Logging.hpp>
#include <fstream>
#include <QVBoxLayout>
#include <QScrollArea>

ControlUi::ControlUi(QWidget *parent)
    : QWidget(parent)
{
    resize(300,120);
    show();
    sendTimer = new QTimer(this);
}

ControlUi::~ControlUi()
{
}

base::commands::Joints ControlUi::getJoints()
{
    return currentJointCommand;
}

void ControlUi::handleUserInput(std::string name, base::JointState state)
{
    size_t idx;
    try{
        idx = currentJointCommand.mapNameToIndex(name);
    }
    catch(base::commands::Joints::InvalidName ex){
        LOG_ERROR("Invalid name was passed from QObject. This should never happen, probably there is something wrong in initModel. Expeption: %s", ex.what());
        return;
    }

    currentJointCommand[idx] = state;

    //Create joint command from base-types
    base::commands::Joints val_joint_command;
    val_joint_command.resize(1);
    val_joint_command.names[0] = name;
    val_joint_command[0] = state;

    //Tell everyone that there's something new
    emit(newVal(val_joint_command));
}


void ControlUi::initModel(QString filepath)
{
    //Read the urdf file
    std::ifstream file(filepath.toStdString().c_str());
    std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml);

    //
    // Set up user interface
    //
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
    QVBoxLayout *vertical_layout = new QVBoxLayout;

    //Check boxes
    QHBoxLayout *horizontal_layout = new QHBoxLayout;
    QCheckBox *cb_update = new QCheckBox;
    cb_update->setText("Update joint state");
    cb_update->setCheckState(Qt::Checked);
    connect(cb_update, SIGNAL(toggled(bool)), this, SLOT(handleUpdateCheckbox(bool)));

    QCheckBox *cb_keep_sending = new QCheckBox;
    cb_keep_sending->setText("Keep sending command");
    cb_keep_sending->setCheckState(Qt::Unchecked);
    connect(cb_keep_sending, SIGNAL(toggled(bool)), this, SLOT(handleKeepSendingCheckbox(bool)));

    horizontal_layout->addWidget(cb_update);
    horizontal_layout->addWidget(cb_keep_sending);
    vertical_layout->addItem(horizontal_layout);

    QScrollArea* scrollarea = new QScrollArea();
    QWidget* joints = new QWidget();
    QGridLayout* layout = new QGridLayout(joints);

//    QGridLayout *layout = new QGridLayout;
    int i=0;
    const int columns=5;
    for (it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it){
        boost::shared_ptr<urdf::Joint> joint = it->second;
        std::string name = it->first;
        if(joint->type != urdf::Joint::FIXED){
            //Create user interface elements
            JointForm *j_form = new JointForm();
            j_form->setProperty("name", QString(name.c_str()));
            if(joint->limits){
                j_form->initFromJointLimits(*(joint->limits), name);
            }
            else{
                j_form->setName(name);
            }
            connect(j_form, SIGNAL(valueChanged(std::string, base::JointState)), this, SLOT(handleUserInput(std::string, base::JointState)));

            int row=i/columns;
            int column=i%columns;
            layout->addWidget(j_form, row, column);

            LOG_DEBUG("Created GUI elements for %s", name.c_str());

            //Fill current joint configuration
            currentJointCommand.names.push_back(name);
            currentJointCommand.elements.push_back(base::JointState());

            currentJointsState.names.push_back(name);
            currentJointsState.elements.push_back(base::JointState());
            joint_forms.push_back(j_form);

            i++;
        }
    }
    joints->setLayout(layout);
    joints->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Minimum);
    scrollarea->setWidget(joints);
    vertical_layout->addWidget(scrollarea);

    //Send what?
    QHBoxLayout* send_what_layout = new QHBoxLayout;
    send_pos = new QCheckBox;
    send_pos->setCheckState(Qt::Checked);
    send_pos->setText("Send Position");
    send_what_layout->addWidget(send_pos);

    send_vel = new QCheckBox;
    send_vel->setCheckState(Qt::Checked);
    send_vel->setText("Send Velocity");
    send_what_layout->addWidget(send_vel);

    send_eff = new QCheckBox;
    send_eff->setCheckState(Qt::Checked);
    send_eff->setText("Send Effort");
    send_what_layout->addWidget(send_eff);

    vertical_layout->addItem(send_what_layout);

    //Send button
    QPushButton* send_button = new QPushButton;
    send_button->setText("Send Joint Command");
    vertical_layout->addWidget(send_button);

    connect(send_button, SIGNAL(clicked()), this, SLOT(triggerSend()));

    setLayout(vertical_layout);

    handleUpdateCheckbox(cb_update->checkState());
    handleKeepSendingCheckbox(cb_keep_sending->checkState());
}

void ControlUi::handleUpdateCheckbox(bool update)
{
    doUpdate = update;
    for(uint i=0; i<joint_forms.size(); i++){
        joint_forms[i]->activate(!update);
    }
}

void ControlUi::handleKeepSendingCheckbox(bool doSend)
{
    if(doSend){
        sendTimer->start(10);
        connect(sendTimer, SIGNAL(timeout()), this, SLOT(triggerSend()));
    }
    else{
        disconnect(sendTimer, SIGNAL(timeout()), this, SLOT(triggerSend()));
        sendTimer->stop();
    }
}

void ControlUi::triggerSend(){
    currentJointCommand.time = base::Time::now();
    for(uint i=0; i<joint_forms.size(); i++){
        base::JointState state;
        joint_forms[i]->getJointState(state);
        if(!send_pos->isChecked())
            state.position = base::unset<double>();
        if(!send_vel->isChecked())
            state.speed = base::unset<double>();
        if(!send_eff->isChecked())
            state.effort = base::unset<double>();
        currentJointCommand.elements[i] = state;
    }
    emit(ControlUi::sendSignal());
}

void ControlUi::setJointState(const base::samples::Joints &sample)
{
    if(doUpdate){
        for(int i=0; i<sample.size(); i++){
            std::string name = sample.names[i];
            std::vector<std::string>::iterator it =
                    std::find(currentJointsState.names.begin(), currentJointsState.names.end(), name);
            if(it == currentJointsState.names.end()){
                LOG_DEBUG("Received joint sample contains unknown joint name '%s'. Will ignore it's value.", name.c_str());
                continue;
            }

            int index = currentJointsState.mapNameToIndex(name);
            currentJointsState.elements[index] = sample.elements[i];
            joint_forms[index]->setJointState(sample.elements[i]);
        }
    }
}
