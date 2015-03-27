#ifndef CONTROLBOT_H
#define CONTROLBOT_H

#include <base/JointLimits.hpp>
#include <QtGui>
#include <base/commands/Joints.hpp>
#include "jointform.h"
#include <QTimer>

inline double deg(double rad){return rad*57.2957795;}
inline double rad(double deg){return deg*0.0174532925;}

class ControlUi : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(bool generateJointStateUi READ getGenerateJointStateUi WRITE setGenerateJointStateUi)
public:
    ControlUi(QWidget *parent = 0);
    virtual ~ControlUi();

public:
    inline bool getGenerateJointStateUi(){return generateJointStateUi;}
    inline bool setGenerateJointStateUi(bool generate){generateJointStateUi=generate; return  true;}

    Q_INVOKABLE void setJointState(base::samples::Joints const &sample);
    Q_INVOKABLE base::commands::Joints getJoints();
    void setReference(const base::samples::Joints &sample);

public slots:

     void checkKeepSendingCB(bool checked);
     void checkUpdateCB(bool checked);
     void enableUpdateCB(bool enable);
     void enableSendCBs(bool enable);
     void layoutJointForms(int columns);
     void configureUi(double override_vel_limit, bool positive_vel_only,
                   bool no_effort, bool no_velocity, double command_noise_std_dev = 0);
     void initFromYaml(QString filepath);
     void initFromURDF(QString filepath);
     void initModel(const base::JointLimits &limits);

protected slots:
    void handleUserInput(std::string, base::JointState);
    void triggerSend();
    void handleUpdateCheckbox(bool update);
    void handleKeepSendingCheckbox(bool doSend);
signals:
    void newVal(base::samples::Joints val);
    void sendSignal();

protected:

    double whiteNoise(const double std_dev);

    base::commands::Joints currentJointCommand;
    base::samples::Joints currentJointsState;
    bool generateJointStateUi;
    std::vector<JointForm*> joint_forms;
    QTimer* sendTimer;
    QPushButton* send_button;
    bool doSend;
    bool doUpdate;
    QCheckBox* send_pos;
    QCheckBox* send_vel;
    QCheckBox* send_eff;
    JointForm::Config config;
    QCheckBox *cb_update;
    QCheckBox *cb_keep_sending;
    QSpinBox *sb_columns;
    QGridLayout* joints_layout;
    QWidget* joints_widget;
    QScrollArea* joints_scroll;
};

#endif /* CONTROLBOT_H */
