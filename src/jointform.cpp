#include "jointform.h"
#include "ui_jointform.h"
#include <math.h>

JointForm::JointForm(QWidget *parent, Config config) :
    QWidget(parent), config(config),
    ui(new Ui::JointForm)
{
    ui->setupUi(this);

    this->ui->slPos->setMinimum(0);
    this->ui->slPos->setMaximum(SLIDER_INCREMENTS);
    this->ui->slVel->setMinimum(0);
    this->ui->slVel->setMaximum(SLIDER_INCREMENTS);
    this->ui->slEff->setMinimum(0);
    this->ui->slEff->setMaximum(SLIDER_INCREMENTS);

    jointLimits.min.position = -M_PI_4;
    jointLimits.max.position = +M_PI_4;
    jointLimits.min.speed    = -M_PI_4;
    jointLimits.max.speed    = +M_PI_4;
    jointLimits.min.effort   = -1.0;
    jointLimits.max.effort   = +1.0;
    
    setJointLimits();

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


double JointForm::sliderValueToRealValue(const double& min, const double& max, const int& sliderValue) {
    double value = (max - min) * (double)sliderValue / SLIDER_INCREMENTS + min;
    return value;
}

double JointForm::sliderValueToRealPos(const int& sliderValue) {
    return sliderValueToRealValue( jointLimits.min.position, jointLimits.max.position, sliderValue );
}

double JointForm::sliderValueToRealVel(const int& sliderValue) {
    return sliderValueToRealValue( jointLimits.min.speed, jointLimits.max.speed, sliderValue );
}

double JointForm::sliderValueToRealEff(const int& sliderValue) {
    return sliderValueToRealValue( jointLimits.min.effort, jointLimits.max.effort, sliderValue );
}


int JointForm::realValueTosliderValue(const double& min, const double& max, const double& realValue) {
    double value = (realValue - min)/(max - min) * SLIDER_INCREMENTS;
    return value;
}

int JointForm::realPosToSliderValue(const double& realValue) {
    return realValueTosliderValue( jointLimits.min.position, jointLimits.max.position, realValue );
}

int JointForm::realVelToSliderValue(const double& realValue) {
    return realValueTosliderValue( jointLimits.min.speed, jointLimits.max.speed, realValue );
}

int JointForm::realEffToSliderValue(const double& realValue) {
    return realValueTosliderValue( jointLimits.min.effort, jointLimits.max.effort, realValue );
}


void JointForm::handlePosSliderChange(int val){
    this->ui->dsbPos->blockSignals(true);
    this->ui->dsbPos->setValue(sliderValueToRealPos(val));
    this->ui->dsbPos->blockSignals(false);
    handleValueChange();
}

void JointForm::handleVelSliderChange(int val){
    this->ui->dsbVel->blockSignals(true);
    this->ui->dsbVel->setValue(sliderValueToRealVel(val));
    this->ui->dsbVel->blockSignals(false);
    handleValueChange();
}

void JointForm::handleEffSliderChange(int val){
    this->ui->dsbEff->blockSignals(true);
    this->ui->dsbEff->setValue(sliderValueToRealEff(val));
    this->ui->dsbEff->blockSignals(false);
    handleValueChange();
}

void JointForm::handlePosBoxChange(double val){
    this->ui->slPos->blockSignals(true);
    this->ui->slPos->setValue(realPosToSliderValue(val));
    this->ui->slPos->blockSignals(false);
    handleValueChange();
}

void JointForm::handleVelBoxChange(double val){
    this->ui->slVel->blockSignals(true);
    this->ui->slVel->setValue(realVelToSliderValue(val));
    this->ui->slVel->blockSignals(false);
    handleValueChange();
}

void JointForm::handleEffBoxChange(double val){
    this->ui->slEff->blockSignals(true);
    this->ui->slEff->setValue(realEffToSliderValue(val));
    this->ui->slEff->blockSignals(false);
    handleValueChange();
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


double JointForm::calcSpinBoxStep(const double& min, const double& max) {
    double step = pow(10, -calcSpinBoxDecimals(min, max));
    return step;
}

int JointForm::calcSpinBoxDecimals(const double& min, const double& max) {
    double decimals = 0;
    double tmp = (max-min) / SPINBOX_STEPS;
    
    while ( tmp < 1 ) {
        decimals++;
        tmp *= 10;
    }
    
    return decimals;
}

void JointForm::setJointLimits() {
    if(config.override_vel_limit){
        jointLimits.min.speed = -fabs(config.override_vel_limit);
        jointLimits.max.speed = +fabs(config.override_vel_limit);
    }
    if(config.positive_vel_only){
        jointLimits.min.speed = 0;
    }

    this->ui->dsbPos->setMinimum(jointLimits.min.position);
    this->ui->dsbPos->setMaximum(jointLimits.max.position);
    this->ui->dsbPos->setDecimals(
        calcSpinBoxDecimals(jointLimits.min.position, jointLimits.max.position)
    );
    this->ui->dsbPos->setSingleStep(
        calcSpinBoxStep(jointLimits.min.position, jointLimits.max.position)
    );
    
    this->ui->dsbVel->setMinimum(jointLimits.min.speed);
    this->ui->dsbVel->setMaximum(jointLimits.max.speed);
    this->ui->dsbVel->setDecimals(
        calcSpinBoxDecimals(jointLimits.min.speed, jointLimits.max.speed)
    );
    this->ui->dsbVel->setSingleStep(
        calcSpinBoxStep(jointLimits.min.speed, jointLimits.max.speed)
    );

    this->ui->dsbEff->setMinimum(jointLimits.min.effort);
    this->ui->dsbEff->setMaximum(jointLimits.max.effort);    
    this->ui->dsbEff->setDecimals(
        calcSpinBoxDecimals(jointLimits.min.effort, jointLimits.max.effort)
    );
    this->ui->dsbEff->setSingleStep(
        calcSpinBoxStep(jointLimits.min.effort, jointLimits.max.effort)
    );
}

void JointForm::initFromJointRange(const base::JointLimitRange& rng, std::string name){
    setName(name);

    jointLimits = rng;
    
    setJointLimits();
}

void JointForm::initFromJointLimits(const urdf::JointLimits& limits, std::string name){
    setName(name);
    
    jointLimits.min.position = limits.lower;
    jointLimits.max.position = limits.upper;    
    
    jointLimits.min.speed = -limits.velocity;
    jointLimits.max.speed = +limits.velocity;
    
    jointLimits.min.effort = -limits.effort;
    jointLimits.max.effort = +limits.effort;
    
    setJointLimits();
}


void JointForm::setJointState(const base::JointState& state)
{
    if(state.hasPosition()) {
        this->ui->dsbPos->blockSignals(true);
        this->ui->dsbPos->setValue(state.position);
        this->ui->dsbPos->blockSignals(false);
        this->ui->slPos->blockSignals(true);
        this->ui->slPos->setValue(realPosToSliderValue(state.position));
        this->ui->slPos->blockSignals(false);
    }
    if(state.hasSpeed()) {
        this->ui->dsbVel->blockSignals(true);
        this->ui->dsbVel->setValue(state.speed);
        this->ui->dsbVel->blockSignals(false);
        this->ui->slVel->blockSignals(true);
        this->ui->slVel->setValue(realVelToSliderValue(state.speed));
        this->ui->slVel->blockSignals(false);
    }
    if(state.hasEffort()) {
        this->ui->dsbEff->blockSignals(true);
        this->ui->dsbEff->setValue(state.effort);
        this->ui->dsbEff->blockSignals(false);
        this->ui->slEff->blockSignals(true);
        this->ui->slEff->setValue(realEffToSliderValue(state.effort));
        this->ui->slEff->blockSignals(false);
    }
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
