#include "../include/Form_Network.h"

Form_Network::Form_Network(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Form_Network)
{
    ui->setupUi(this);

	ui-> comboBox_choose-> addItem("Server");
	ui-> comboBox_choose-> addItem("Client");
}

Form_Network::~Form_Network()
{
	// CNetwork.socketclose();
    delete ui;
}


// void Form_Network::on_Button_network_connect_clicked()
// {
// 	wchar_t* ip_use_array;
// 	ip_use_array = new wchar_t[(ui->textEdit_network_ip->toPlainText()).size() + 1];
// 	(ui->textEdit_network_ip->toPlainText()).toWCharArray(ip_use_array);
	
// 	// if (CNetwork.get_who_use() == server_use)
// 	// {
// 	// 	CNetwork.socketopen_server(ip_use_array, (ui->textEdit_network_port->toPlainText()).toInt());
// 	// 	ui->label_network_state->setText(" Connect to Client Successfully. ");
// 	// 	ui->label_network_state->setStyleSheet(QString::fromUtf8("background-color: rgb(170, 255, 127);\n"));
// 	// }
// 	// else if (CNetwork.get_who_use() == client_use)
// 	// {
// 	// 	CNetwork.socketopen_client(ip_use_array, (ui->textEdit_network_port->toPlainText()).toInt());
// 	// 	if (CNetwork.get_check_connect() == 0)
// 	// 	{
// 	// 		ui->label_network_state->setText(" Connect to Server Successfully. ");
// 	// 		ui->label_network_state->setStyleSheet(QString::fromUtf8("background-color: rgb(170, 255, 127);\n"));
// 	// 	}
// 	// 	else if (CNetwork.get_check_connect() == -1)
// 	// 	{
// 	// 		ui->label_network_state->setText(" Connect to Client Failed. ");
// 	// 		this->ui->label_network_state->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 192, 203);\n"));
// 	// 	}
// 	// }
// }

// void Form_Network::on_Button_stop_to_connect_clicked()
// {
// 	// CNetwork.socketclose();
// 	ui->label_network_state->setText(" state. ");
// 	ui->label_network_state->setStyleSheet(QString::fromUtf8("background-color: rgb(240, 240, 240);\n"));
// }

// void Form_Network::on_Button_send_start_clicked()
// {
// 	// CNetwork.sendmessage((ui->textEdit_network_messagetosend->toPlainText()).toStdString(), 1);
// }

// void Form_Network::on_Button_send_stop_clicked()
// {
// }

// void Form_Network::on_Button_rceive_start_clicked()
// {
// 	// ui->listWidget_network_message_receive->addItem(CNetwork.recvmessage().c_str());
// }

// void Form_Network::on_Button_rceive_stop_clicked()
// {
// 	bThreadNetwork = false;
// }

// void Form_Network::on_Button_rceive_clear_clicked()
// {
// 	ui->listWidget_network_message_receive->clear();
// }

// void Form_Network::on_comboBox_choose_currentTextChanged()
// {
// 	if ((ui->comboBox_choose->currentText()).toStdString() == "Server")
// 	{
// 		// CNetwork.set_who_use(server_use);
// 	}
// 	else if ((ui->comboBox_choose->currentText()).toStdString() == "Client")
// 	{
// 		// CNetwork.set_who_use(client_use);
// 	}
// }
// void Form_Network::thread_Network_Color()
// {	
// 	while (bThreadNetwork)
// 	{
// 		// ui->listWidget_network_message_receive->addItem(CNetwork.recvmessage().c_str());
// 	}
// }


