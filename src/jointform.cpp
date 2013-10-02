#include "jointform.h"
#include "ui_jointform.h"
#include <math.h>

JointForm::JointForm(QWidget *parent) :
    QWidget(parent),
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

void JointForm::initFromJointRange(const base::JointLimitRange& rng, std::string name){
    setName(name);

    this->ui->slPos->setMinimum(rng.min.position * SLIDER_POS_SCALE_FACTOR);
    this->ui->slPos->setMaximum(rng.max.position * SLIDER_POS_SCALE_FACTOR);
    this->ui->slVel->setMinimum(rng.min.speed * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slVel->setMaximum(rng.max.speed * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slEff->setMinimum(rng.min.effort * SLIDER_EFF_SCALE_FACTOR);
    this->ui->slEff->setMaximum(rng.max.effort * SLIDER_EFF_SCALE_FACTOR);

    this->ui->dsbPos->setMinimum(rng.min.position);
    this->ui->dsbPos->setMaximum(rng.max.position);
    this->ui->dsbVel->setMinimum(rng.min.speed);
    this->ui->dsbVel->setMaximum(rng.max.speed);
    this->ui->dsbEff->setMinimum(rng.min.effort);
    this->ui->dsbEff->setMaximum(rng.max.effort);
}

void JointForm::initFromJointLimits(const urdf::JointLimits& limits, std::string name){
    setName(name);

    this->ui->slPos->setMinimum(limits.lower * SLIDER_POS_SCALE_FACTOR);
    this->ui->slPos->setMaximum(limits.upper * SLIDER_POS_SCALE_FACTOR);
    this->ui->slVel->setMinimum(-limits.velocity * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slVel->setMaximum(limits.velocity * SLIDER_VEL_SCALE_FACTOR);
    this->ui->slEff->setMinimum(-limits.effort * SLIDER_EFF_SCALE_FACTOR);
    this->ui->slEff->setMaximum(limits.effort * SLIDER_EFF_SCALE_FACTOR);

    this->ui->dsbPos->setMinimum(limits.lower);
    this->ui->dsbPos->setMaximum(limits.upper);
    this->ui->dsbVel->setMinimum(-limits.velocity);
    this->ui->dsbVel->setMaximum(limits.velocity);
    this->ui->dsbEff->setMinimum(-limits.effort);
    this->ui->dsbEff->setMaximum(limits.effort);
}


void JointForm::setJointState(const base::JointState& state)
{
    this->ui->dsbPos->setValue(state.position);
    this->ui->dsbVel->setValue(state.speed);
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
