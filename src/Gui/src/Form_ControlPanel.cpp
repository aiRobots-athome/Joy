#include "../include/Form_ControlPanel.h"

Form_ControlPanel::Form_ControlPanel(QWidget *parent) : QMainWindow(parent),
														ui(new Ui::Form_ControlPanel)
{
	ui->setupUi(this);
	form_body = new Form_Body(this);
	form_network = new Form_Network();
	form_strategy = new Form_Strategy(this);
	// form_vision = new Form_Vision();
}

Form_ControlPanel::~Form_ControlPanel()
{
	form_body->deleteLater();
	form_network->deleteLater();
	form_strategy->deleteLater();
	// form_vision->deleteLater();
}

void Form_ControlPanel::closeEvent(QCloseEvent *event)
{
	// if (form_vision->isVisible())
	// 	form_vision->close();
	if (form_body->isVisible())
		form_body->close();
	if (form_strategy->isVisible())
		form_strategy->close();
	if (form_network->isVisible())
		form_network->close();
}

void Form_ControlPanel::on_pB_Form_Vision_clicked()
{
	// if (form_vision->isHidden())
	// 	form_vision->show();
	// else
	// 	form_vision->hide();
}

void Form_ControlPanel::on_pB_Form_Body_clicked()
{
	if (form_body->isHidden())
		form_body->show();
	else
		form_body->hide();
}

void Form_ControlPanel::on_pB_Form_Strategy_clicked()
{
	if (form_strategy->isHidden())
		form_strategy->show();
	else
		form_strategy->hide();
}

void Form_ControlPanel::on_pB_Form_Network_clicked()
{
	if (form_network->isHidden())
		form_network->show();
	else
		form_network->hide();
}