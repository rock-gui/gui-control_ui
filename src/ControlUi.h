#ifndef CONTROLBOT_H
#define CONTROLBOT_H

#include <QtGui>
#include <base/commands/Joints.hpp>

inline double deg(double rad){return rad*57.2957795;}
inline double rad(double deg){return deg*0.0174532925;}

class ControlUi : public QWidget
{
    Q_OBJECT
public:
    ControlUi(QWidget *parent = 0);
    virtual ~ControlUi();

public:
    const base::commands::Joints& getJoints();

public slots:
    void initModel(QString filepath);
    void handleNewVal(double val);

signals:
    void newVal(QString name, double val);
    void newVal(base::commands::Joints val);

protected:
    base::commands::Joints currentJoints_;
};

#endif /* CONTROLBOT_H */
