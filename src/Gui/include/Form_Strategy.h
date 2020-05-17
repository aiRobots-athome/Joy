#ifndef FORM_STRATEGY_H
#define FORM_STRATEGY_H
#include "Gui/ui_Form_Strategy.h"
#include "Strategy/Strategy.h"
#include <fstream>
#include <iostream>
#include <thread>

class Form_Strategy : public QDialog
{
	Q_OBJECT

public:
	explicit Form_Strategy(QWidget *parent = nullptr);
	virtual ~Form_Strategy();

private slots:
	void on_button_RunStrategy_clicked();
	void on_button_EndStrategy_clicked();
	void on_button_ClearLog_clicked();

private:
	Ui::Form_Strategy *ui;
	std::fstream file_strategy_name;
	std::thread *thread_strategy;
	Strategy *strategy;
};

#endif // FORM_STRATEGY_H
