#ifndef FORM_HEAD_H
#define FORM_HEAD_H
#include <QtCore/QObject>
#include "Gui/ui_Form_Body.h"
#include "Robot/Robot.h"

class Form_Head : public QObject
{
    Q_OBJECT

public:
    Form_Head(Ui::Form_Body *_ui, QObject *parent = nullptr) : QObject(nullptr) { ui = _ui; };
    ~Form_Head(){};
    void Display();

public slots:
    void HeadReset();
    void HeadVerticalMove();
    void HeadHorizontalMove();

private:
    Ui::Form_Body *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);
};

#endif