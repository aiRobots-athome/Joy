#include "../include/Form_ControlPanel.h"

Form_ControlPanel::Form_ControlPanel(QWidget *parent) : QMainWindow(parent),
														ui(new Ui::Form_ControlPanel)
{
	ui->setupUi(this);
	form_robot = new Form_Robot(this);
	form_scara = new Form_Scara(this);
	form_network = new Form_Network(this);
	form_strategy = new Form_Strategy(this);

}

Form_ControlPanel::~Form_ControlPanel()
{
	form_robot->deleteLater();
	form_scara->deleteLater();
	form_network->deleteLater();
	form_strategy->deleteLater();
	delete ui;
}

void Form_ControlPanel::closeEvent(QCloseEvent *event)
{
	if (form_robot->isVisible())
		form_robot->close();
	if(form_scara->isVisible())
		form_scara->close();
	if (form_strategy->isVisible())
		form_strategy->close();
	if (form_network->isVisible())
		form_network->close();
}

void Form_ControlPanel::on_pB_Form_Robot_clicked()
{
	if (form_robot->isHidden())
		form_robot->show();
	else
		form_robot->hide();
}

void Form_ControlPanel::on_pB_Form_Scara_clicked()
{
	if (form_scara->isHidden())
		form_scara->show();
	else
		form_scara->hide();
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