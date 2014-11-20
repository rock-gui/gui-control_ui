#include "jointform.h"
#include "ui_jointform.h"
#include <math.h>

JointForm::JointForm(QWidget *parent, Config config) :
    QWidget(parent), config(config),
    ui(new Ui::JointForm)
{
    ui->setupUi(this);

    this->ui->slPos->setMinimum(-M_PI_4 * SLIDER_POS_SCALE_FACTOR);
    this->ui->slPos->setMaximum(M_PI_4 * SLIDER_POS_SCALE_FACTOR);
    this->ui->slVel->setMinimum(-M_PI_4 * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slVel->setMaximum(M_PI_4 * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slEff->setMinimum(-M_PI_4 * SLIDER_EFF_SCALE_FACTOR);
    this->ui->slEff->setMaximum(M_PI_4 * SLIDER_EFF_SCALE_FACTOR);

    this->ui->dsbPos->setMinimum(-M_PI_4);
    this->ui->dsbPos->setMaximum(M_PI_4);
    this->ui->dsbVel->setMinimum(-M_PI_4);
    this->ui->dsbVel->setMaximum(M_PI_4);
    this->ui->dsbEff->setMinimum(-M_PI_4);
    this->ui->dsbEff->setMaximum(M_PI_4);

    connect(this->ui->slPos, SIGNAL(valueChanged(int)), this, SLOT(handlePosSliderChange(int)));
    connect(this->ui->slVel, SIGNAL(valueChanged(int)), this, SLOT(handleVelSliderChange(int)));
    connect(this->ui->slEff, SIGNAL(valueChanged(int)), this, SLOT(handleEffSliderChange(int)));

    connect(this->ui->dsbPos, SIGNAL(valueChanged(double)), this, SLOT(handlePosBoxChange(double)));
    connect(this->ui->dsbVel, SIGNAL(valueChanged(double)), this, SLOT(handleVelBoxChange(double)));
    connect(this->ui->dsbEff, SIGNAL(valueChanged(double)), this, SLOT(handleEffBoxChange(double)));

    if(config.no_velocity){
        this->ui->slVel->setEnabled(false);
        this->ui->slVel->hide();
        this->ui->dsbVel->setEnabled(false);
        this->ui->dsbVel->hide();
        this->ui->lblVel->hide();
    }
    if(config.no_effort){
        this->ui->slEff->setEnabled(false);
        this->ui->slEff->hide();
        this->ui->dsbEff->setEnabled(false);
        this->ui->dsbEff->hide();
        this->ui->lblEff->hide();
    }
}

JointForm::~JointForm()
{
    delete ui;
}

void JointForm::handlePosSliderChange(int val){
    this->ui->dsbPos->setValue((double)val/SLIDER_POS_SCALE_FACTOR);
}

void JointForm::handleVelSliderChange(int val){
    this->ui->dsbVel->setValue((double)val/SLIDER_VEL_SCALE_FACTOR);
}

void JointForm::handleEffSliderChange(int val){
    this->ui->dsbEff->setValue((double)val/SLIDER_EFF_SCALE_FACTOR);
}

void JointForm::handlePosBoxChange(double val){
    this->ui->slPos->setValue(val*SLIDER_POS_SCALE_FACTOR);
}

void JointForm::handleVelBoxChange(double val){
    this->ui->slVel->setValue(val*SLIDER_VEL_SCALE_FACTOR);
}

void JointForm::handleEffBoxChange(double val){
    this->ui->slEff->setValue(val*SLIDER_EFF_SCALE_FACTOR);
}

void JointForm::handleValueChange()
{
    base::JointState state;
    getJointState(state);
    emit(this->valueChanged(this->ui->lblName->text().toStdString(), state));
}

void JointForm::setName(std::string name)
{
    this->ui->lblName->setText(QString::fromStdString(name));
}

void JointForm::setJointLimit(double min, double max){
    if(config.override_vel_limit){
        min = -fabs(config.override_vel_limit);
        max = fabs(min);
    }
    if(config.positive_vel_only){
        min = 0.;
    }

    this->ui->slVel->setMinimum(min * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slVel->setMaximum(max * SLIDER_VEL_SCALE_FACTOR);

    this->ui->dsbVel->setMinimum(min);
    this->ui->dsbVel->setMaximum(max);
}

void JointForm::initFromJointRange(const base::JointLimitRange& rng, std::string name){
    setName(name);

    setJointLimit(rng.min.speed, rng.max.speed);

    this->ui->slPos->setMinimum(rng.min.position * SLIDER_POS_SCALE_FACTOR);
    this->ui->slPos->setMaximum(rng.max.position * SLIDER_POS_SCALE_FACTOR);

    this->ui->slEff->setMinimum(rng.min.effort * SLIDER_EFF_SCALE_FACTOR);
    this->ui->slEff->setMaximum(rng.max.effort * SLIDER_EFF_SCALE_FACTOR);

    this->ui->dsbPos->setMinimum(rng.min.position);
    this->ui->dsbPos->setMaximum(rng.max.position);

    this->ui->dsbEff->setMinimum(rng.min.effort);
    this->ui->dsbEff->setMaximum(rng.max.effort);
}

void JointForm::initFromJointLimits(const urdf::JointLimits& limits, std::string name){
    setName(name);

    setJointLimit(-limits.velocity, limits.velocity);

    this->ui->slPos->setMinimum(limits.lower * SLIDER_POS_SCALE_FACTOR);
    this->ui->slPos->setMaximum(limits.upper * SLIDER_POS_SCALE_FACTOR);

    this->ui->slEff->setMinimum(-limits.effort * SLIDER_EFF_SCALE_FACTOR);
    this->ui->slEff->setMaximum(limits.effort * SLIDER_EFF_SCALE_FACTOR);

    this->ui->dsbPos->setMinimum(limits.lower);
    this->ui->dsbPos->setMaximum(limits.upper);

    this->ui->dsbEff->setMinimum(-limits.effort);
    this->ui->dsbEff->setMaximum(limits.effort);
}


void JointForm::setJointState(const base::JointState& state)
{
    if(state.hasPosition())
        this->ui->dsbPos->setValue(state.position);
    if(state.hasSpeed())
        this->ui->dsbVel->setValue(state.speed);
    if(state.hasEffort())
        this->ui->dsbEff->setValue(state.effort);
}

void JointForm::activate(bool active)
{
    this->ui->slPos->setEnabled(active);
    this->ui->slVel->setEnabled(active);
    this->ui->slEff->setEnabled(active);

    this->ui->dsbPos->setEnabled(active);
    this->ui->dsbVel->setEnabled(active);
    this->ui->dsbEff->setEnabled(active);
}

void JointForm::getJointState(base::JointState& state)
{
    state.position = this->ui->dsbPos->value();
    state.speed = this->ui->dsbVel->value();
    state.effort = this->ui->dsbEff->value();
}

base::JointState JointForm::getJointState()
{
    base::JointState state;
    getJointState(state);
    return state;
}
