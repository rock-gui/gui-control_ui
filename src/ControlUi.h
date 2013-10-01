#ifndef CONTROLBOT_H
#define CONTROLBOT_H

#include <QtGui>
#include <base/commands/Joints.hpp>

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
    const base::commands::Joints& getJoints();
    inline bool getGenerateJointStateUi(){return generateJointStateUi;}
    inline bool setGenerateJointStateUi(bool generate){generateJointStateUi=generate;}

    Q_INVOKABLE void setJointState(base::samples::Joints const &sample);

public slots:
    void initModel(QString filepath);

protected slots:
    void handleNewVal(double val);

signals:
    void newVal(QString name, double val);
    void newVal(base::commands::Joints val);

protected:
    base::commands::Joints currentJointCommand;
    base::samples::Joints currentJointsState;
    bool generateJointStateUi;
};

#endif /* CONTROLBOT_H */
