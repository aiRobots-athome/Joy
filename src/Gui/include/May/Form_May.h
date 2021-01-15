#ifndef FORM_MAY_H
#define FORM_MAY_H
#include "Gui/ui_Form_May.h"
#include "May/May.h"
#include "Form_MayArm.h"
#include <QtWidgets/QDialog>
#include <QtCore/QThread>
#include <thread>

class Form_May : public QDialog
{
    Q_OBJECT

public:
    explicit Form_May(QWidget *parent = nullptr);
    virtual ~Form_May();

protected:
    void showEvent(QShowEvent *event);
	void closeEvent(QCloseEvent *event);

private slots:
    void on_May_btn_Start_clicked();
    void on_May_btn_Stop_clicked();

private:
    Ui::Form_May *ui;
    Form_MayArm *form_mayarm;
    QThread *thread_mayarm;

    // Display
	std::thread *thread_display;
	bool _is_deleted_thread_display;
	void Display();

    May *CMay;
};

#endif
