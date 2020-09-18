#ifndef JOINTFORM_H
#define JOINTFORM_H

#include <QWidget>
#include <base/JointLimitRange.hpp>
#include <urdf_model/model.h>

namespace Ui {
class JointForm;
}

class JointForm : public QWidget
{
    Q_OBJECT
    
public:
    struct Config{
        inline Config():
            override_vel_limit(0), positive_vel_only(false),
            no_effort(false), no_velocity(false), command_noise_std_dev(0.0){}

        double override_vel_limit;
        bool positive_vel_only;
        bool no_effort;
        bool no_velocity;
        double command_noise_std_dev;
    };

    explicit JointForm(QWidget *parent = 0, Config=Config());
    void initFromJointRange(const base::JointLimitRange& rng, std::string name);
    void initFromJointLimits(const urdf::JointLimits& limits, std::string name);
    base::JointState getJointState();
    ~JointForm();
    
    const double SLIDER_INCREMENTS = 256.0;
    const double SPINBOX_STEPS = 100.0;

    double sliderValueToRealValue(const double& min, const double& max, const int& sliderValue);
    double sliderValueToRealPos(const int& sliderValue);
    double sliderValueToRealVel(const int& sliderValue);
    double sliderValueToRealEff(const int& sliderValue);

    int realValueTosliderValue(const double& min, const double& max, const double& realValue);
    int realPosToSliderValue(const double& realValue);
    int realVelToSliderValue(const double& realValue);
    int realEffToSliderValue(const double& realValue);
    
    double calcSpinBoxStep(const double& min, const double& max);
    int calcSpinBoxDecimals(const double& min, const double& max);
    
public slots:
    void setJointState(const base::JointState& state);
    void activate(bool active);
    void getJointState(base::JointState& state);
    void setName(std::string name);
    void handleValueChange();

protected slots:
    void handlePosSliderChange(int);
    void handleVelSliderChange(int val);
    void handleEffSliderChange(int val);

    void handlePosBoxChange(double val);
    void handleVelBoxChange(double val);
    void handleEffBoxChange(double val);
    void setJointLimits();

signals:
    void valueChanged(std::string name, base::JointState state);
    
private:
    Config config;
    Ui::JointForm *ui;
    base::JointLimitRange jointLimits;
    
};

#endif // JOINTFORM_H
