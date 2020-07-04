#ifndef FORM_XYPLATFORM_H
#define FORM_XYPLATFORM_H
#include "Gui/ui_Form_Scara.h"
#include "Scara/XYPlatform/XYPlatform.h"

class Form_XYPlatform : public QObject
{
    Q_OBJECT

public:
    Form_XYPlatform(Ui::Form_Scara *_ui, QObject *parent = nullptr) : QObject(nullptr) { ui = _ui; };
    ~Form_XYPlatform(){};
    void Initial();
    void Display();
    void SetXYPlatform(XYPlatform *_XYPlatform)
    {
        CXYPlatform = _XYPlatform;
    }

public slots:
    void XYPlatformPosGo();
    void XYPlatformReset();

private:
    Ui::Form_Scara *ui;
    XYPlatform *CXYPlatform;
};
#endif // FORM_XYPLATFORM_H