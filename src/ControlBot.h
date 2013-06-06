#ifndef CONTROLBOT_H
#define CONTROLBOT_H

#include <QtGui>
#include <urdf/model.h>
#include <base/commands/Joints.hpp>

inline double deg(double rad){return rad*57.2957795;}
inline double rad(double deg){return deg*0.0174532925;}

class ControlBot : public QWidget
{
    Q_OBJECT
public:
    ControlBot(QWidget *parent = 0);
    virtual ~ControlBot();

public slots:
    void initModel(QString filepath);
    void handleNewVal(double val);

signals:
    void newVal(QString name, double val);
    void newVal(base::commands::Joints val);

public:
    //QMap<QString, QDoubleSpinBox*> spinBoxes;
};

#endif /* CONTROLBOT_H */
