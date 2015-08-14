#include <urdf_parser/urdf_parser.h>
#include "ControlUi.h"
#include <base/Logging.hpp>
#include <fstream>
#include <QVBoxLayout>
#include <QScrollArea>
#include <yaml-cpp/yaml.h>

ControlUi::ControlUi(QWidget *parent)
    : QWidget(parent)
{
    resize(850,275);
    show();
    sendTimer = new QTimer(this);
    joints_layout=0;
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

void ControlUi::configureUi(double override_vel_limit, bool positive_vel_only, bool no_effort, bool no_velocity, double command_noise_std_dev){
    config.override_vel_limit = override_vel_limit;
    config.positive_vel_only = positive_vel_only;
    config.no_effort = no_effort;
    config.no_velocity = no_velocity;
    config.command_noise_std_dev = command_noise_std_dev;
}

void ControlUi::initFromURDF(QString filepath){
    std::ifstream file(filepath.toStdString().c_str());
    std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml);

    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
    base::JointLimits limits;
    for (it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it){
        boost::shared_ptr<urdf::Joint> joint = it->second;
        base::JointLimitRange range;

        if(joint->type != urdf::Joint::FIXED && !joint->mimic){
            if(joint->limits){
                if (joint->type == urdf::Joint::CONTINUOUS) {
                    range.max.position = 3.14;
                    range.min.position = -3.14;
                } else {
                    range.max.position = joint->limits->upper;
                    range.min.position = joint->limits->lower;
                }
                range.max.speed = joint->limits->velocity;
                range.min.speed = -joint->limits->velocity;
                range.max.effort = joint->limits->effort;
                range.min.effort = -joint->limits->effort;
                limits.names.push_back(it->first);
                limits.elements.push_back(range);
            }
        }
    }

    initModel(limits);
}

void ControlUi::initFromYaml(QString filepath){

    std::ifstream file(filepath.toStdString().c_str());
    YAML::Parser parser(file);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    try{
        doc["limits"];
    }
    catch(std::exception e){
        std::stringstream ss;
        ss<<"Yaml parsing error: File "<<filepath.toStdString()<<" either doesn't exist or is not a valid joint limits file"<<std::endl;
        throw std::invalid_argument(ss.str());
    }

    const YAML::Node &names_node = doc["limits"]["names"];
    const YAML::Node &elements_node = doc["limits"]["elements"];

    if(elements_node.size() != names_node.size()){
        LOG_ERROR("Invalid limits yaml file. Size of names is different than size of elements");
        throw std::invalid_argument("Invalid yaml file");
    }

    base::JointLimits limits;
    for(uint i = 0; i < names_node.size(); i++){
        std::string name;
        names_node[i] >> name;
        limits.names.push_back(name);

        base::JointLimitRange range;
        elements_node[i]["max"]["position"] >> range.max.position;
        elements_node[i]["min"]["position"] >> range.min.position;
        elements_node[i]["max"]["speed"] >> range.max.speed;
        try{
            elements_node[i]["min"]["speed"] >> range.min.speed;
        }catch(...){
            range.min.speed = -range.max.speed;
        }
        elements_node[i]["max"]["effort"] >> range.max.effort;
        try{
            elements_node[i]["min"]["effort"] >> range.min.effort;
        }catch(...){
            range.min.effort = -range.max.effort;
        }

        limits.elements.push_back(range);
    }

    initModel(limits);
}

void ControlUi::checkKeepSendingCB(bool checked){
    cb_keep_sending->setChecked(checked);
}

void ControlUi::checkUpdateCB(bool checked){
    cb_update->setChecked(checked);
}

void ControlUi::enableUpdateCB(bool enable){
    cb_update->setEnabled(enable);
}

void ControlUi::enableSendCBs(bool enable){
    cb_keep_sending->setEnabled(enable);
    send_button->setEnabled(enable);
    send_pos->setEnabled(enable);
    send_vel->setEnabled(enable);
    send_eff->setEnabled(enable);
}

void ControlUi::layoutJointForms(int columns){
    //Clear old layout
    //If there is at elast one column, there was created a layout before.
    if(joints_layout){
        delete joints_layout;
    }
    joints_layout = new QGridLayout(joints_widget);

    //Crete new layout
    assert(columns>0);
    for (uint i = 0; i < joint_forms.size(); i++){
        int row=i/columns;
        int column=i%columns;
        joints_layout->addWidget(joint_forms[i], row, column);
    }

    joints_widget->setLayout(joints_layout);
    joints_scroll->setWidget(joints_widget);
}

void ControlUi::initModel(const base::JointLimits &limits){

    //
    // Set up user interface
    //
    QVBoxLayout *vertical_layout = new QVBoxLayout;

    //Check boxes
    QHBoxLayout *horizontal_layout = new QHBoxLayout;
    cb_update = new QCheckBox;
    cb_update->setText("Update joint state");
    cb_update->setCheckState(Qt::Checked);
    connect(cb_update, SIGNAL(toggled(bool)), this, SLOT(handleUpdateCheckbox(bool)));

    cb_keep_sending = new QCheckBox;
    cb_keep_sending->setText("Keep sending command");
    cb_keep_sending->setCheckState(Qt::Unchecked);
    connect(cb_keep_sending, SIGNAL(toggled(bool)), this, SLOT(handleKeepSendingCheckbox(bool)));

    QLabel *l_columns = new QLabel();
    l_columns->setText("Columns:");
    l_columns->setAlignment(Qt::AlignRight|Qt::AlignCenter);
    sb_columns = new QSpinBox;
    sb_columns->setMinimum(1);
    sb_columns->setMaximum(99);
    sb_columns->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    horizontal_layout->addWidget(cb_update);
    horizontal_layout->addWidget(cb_keep_sending);
    horizontal_layout->addWidget(l_columns);
    horizontal_layout->addWidget(sb_columns);
    vertical_layout->addItem(horizontal_layout);

    joints_scroll = new QScrollArea();
    joints_widget = new QWidget();
    for (uint i = 0; i < limits.size(); i++){
        std::string name = limits.names[i];

        //Create user interface elements
        JointForm *j_form = new JointForm(this, config);
        j_form->setMinimumWidth(190);
        j_form->setMinimumHeight(20);
        j_form->setProperty("name", QString(name.c_str()));
        j_form->initFromJointRange(limits[i], name);
        connect(j_form, SIGNAL(valueChanged(std::string, base::JointState)), this, SLOT(handleUserInput(std::string, base::JointState)));
        //Fill current joint configuration
        currentJointCommand.names.push_back(name);
        currentJointCommand.elements.push_back(base::JointState());

        currentJointsState.names.push_back(name);
        currentJointsState.elements.push_back(base::JointState());
        joint_forms.push_back(j_form);
    }

    joints_widget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Minimum);
    joints_scroll->setWidgetResizable(true);
    vertical_layout->addWidget(joints_scroll);
    connect(sb_columns, SIGNAL(valueChanged(int)), this, SLOT(layoutJointForms(int)));
    sb_columns->setValue(4);
    layoutJointForms(4);


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
    send_button = new QPushButton;
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
    base::JointState js;
    for(uint i=0; i<joint_forms.size(); i++){
        //When switching from updating mode (joint forms are disabled) to non-updating mode
        //(joint forms are enabled to generate commands), ensure to set speeds to zero. They
        //are treated as reference values from now on. We want the system to stand still
        //initially
        if(update == false){
            joint_forms[i]->getJointState(js);
            js.speed = 0;
            joint_forms[i]->setJointState(js);
        }
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
        state.position += whiteNoise(config.command_noise_std_dev);
        state.speed += whiteNoise(config.command_noise_std_dev);
        state.effort += whiteNoise(config.command_noise_std_dev);
        if(!send_pos->isChecked())
            state.position = base::unset<double>();
        if(!send_vel->isChecked())
            state.speed = base::unset<double>();
        if(!send_eff->isChecked())
            state.effort = base::unset<double>();
        currentJointCommand.elements[i] = state;
    }
    emit(ControlUi::sendSignal());
    emit(newVal(currentJointCommand));
}

void ControlUi::setJointState(const base::samples::Joints &sample)
{
    if(doUpdate){
        for(uint i=0; i<sample.size(); i++){
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

void ControlUi::setReference(const base::samples::Joints &sample){
    cb_update->setChecked(false);
    for(uint i=0; i<sample.size(); i++){
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


double ControlUi::whiteNoise(const double std_dev)
{
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}

