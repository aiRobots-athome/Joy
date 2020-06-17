#ifndef FORM_CONTROLPANEL_H
#define FORM_CONTROLPANEL_H
#include <QtWidgets/QMainWindow>
#include "Gui/ui_Form_ControlPanel.h"
#include "Robot/Form_Robot.h"
#include "Scara/Form_Scara.h"
#include "Form_Network.h"
#include "Form_Strategy.h"

class Form_ControlPanel : public QMainWindow
{
    Q_OBJECT

public:
    explicit Form_ControlPanel(QWidget *parent = nullptr);
    virtual ~Form_ControlPanel();

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void on_pB_Form_Robot_clicked();
    void on_pB_Form_Scara_clicked();
    void on_pB_Form_Strategy_clicked();
    void on_pB_Form_Network_clicked();

private:
    Ui::Form_ControlPanel *ui;
    Form_Robot *form_robot;
    Form_Scara *form_scara;
    Form_Strategy *form_strategy;
    Form_Network *form_network;
};

#endif // FORM_CONTROLPANEL_H
