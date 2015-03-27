#ifndef JOINTFORM_H
#define JOINTFORM_H

#include <QWidget>
#include <base/JointLimitRange.hpp>
#include <urdf_model/model.h>

#define SLIDER_POS_SCALE_FACTOR 100.
#define SLIDER_VEL_SCALE_FACTOR 100.
#define SLIDER_EFF_SCALE_FACTOR 100.

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
    void setJointLimit(double min, double max);

signals:
    void valueChanged(std::string name, base::JointState state);
    
private:
    Ui::JointForm *ui;
    Config config;
};

#endif // JOINTFORM_H
