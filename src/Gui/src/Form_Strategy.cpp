#include "../include/Form_Strategy.h"

Form_Strategy::Form_Strategy(QWidget *parent) : QDialog(parent),
												ui(new Ui::Form_Strategy)
{
	ui->setupUi(this);
	std::string str;
	file_strategy_name.open(std::string(getenv("PWD")) + "/src/Strategy/name.txt");
	do
	{
		getline(file_strategy_name, str);
		ui->comboBox_SelectStrategy->addItem(QString::fromStdString(str));
	} while (!file_strategy_name.eof());
	file_strategy_name.close();
	strategy = new Strategy();
}

Form_Strategy::~Form_Strategy()
{
	delete strategy;
	this->deleteLater();		
	delete ui;
}

void Form_Strategy::on_button_RunStrategy_clicked()
{
	ui->button_RunStrategy->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 0, 0);\n"));
	ui->button_RunStrategy->setEnabled(false);
	ui->comboBox_SelectStrategy->setEnabled(false);

	thread_strategy = new std::thread(&Strategy::select_strategy, strategy, ui->comboBox_SelectStrategy->currentText().toStdString());
}

void Form_Strategy::on_button_EndStrategy_clicked()
{
	strategy->terminate_strategy();
	thread_strategy->join();
	
	ui->button_RunStrategy->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 255, 0);\n"));
	ui->button_RunStrategy->setEnabled(true);
	ui->comboBox_SelectStrategy->setEnabled(true);
}

void Form_Strategy::on_button_ClearLog_clicked()
{
	ui->listWidget_StrategyStageLog->clear();
}