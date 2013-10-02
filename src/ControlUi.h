#ifndef CONTROLBOT_H
#define CONTROLBOT_H

#include <QtGui>
#include <base/commands/Joints.hpp>
#include <jointform.h>
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
    inline bool setGenerateJointStateUi(bool generate){generateJointStateUi=generate;}

    Q_INVOKABLE void setJointState(base::samples::Joints const &sample);
    Q_INVOKABLE base::commands::Joints getJoints();

public slots:
    void initModel(QString filepath);

protected slots:
    void handleUserInput(std::string, base::JointState);
    void triggerSend();
    void handleUpdateCheckbox(bool update);
    void handleKeepSendingCheckbox(bool doSend);

signals:
    void newVal(base::commands::Joints val);
    void sendSignal();

protected:
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
};

#endif /* CONTROLBOT_H */
