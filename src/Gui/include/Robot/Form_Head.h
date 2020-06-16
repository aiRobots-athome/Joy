#ifndef FORM_HEAD_H
#define FORM_HEAD_H
#include <QtCore/QObject>
#include "Gui/ui_Form_Robot.h"
#include "Robot/HeadandLifting/HeadandLifting.h"

class Form_Head : public QObject
{
    Q_OBJECT

public:
    Form_Head(Ui::Form_Robot *_ui, QObject *parent = nullptr) : QObject(parent) { ui = _ui; };
    ~Form_Head(){};
    void Display();
    void SetHeadandLifting(HeadandLifting *_HeadandLifting)
    {
        CHeadandLifting = _HeadandLifting;
    }

public slots:
    void HeadReset();
    void HeadVerticalMove();
    void HeadHorizontalMove();

private:
    Ui::Form_Robot *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);

    HeadandLifting *CHeadandLifting;
};

#endif