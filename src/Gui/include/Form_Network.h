#ifndef FORM_NETWORK_H
#define FORM_NETWORK_H
#include "Gui/ui_Form_Network.h"
#include <QtWidgets/QDialog>
#include <thread>

class Form_Network : public QDialog
{
    Q_OBJECT

public:
    explicit Form_Network(QWidget *parent = nullptr);
    virtual ~Form_Network();

private slots:
	// void on_Button_network_connect_clicked();
	// void on_Button_stop_to_connect_clicked();
	// void on_Button_send_start_clicked();
	// void on_Button_send_stop_clicked();
	// void on_Button_rceive_start_clicked();
	// void on_Button_rceive_stop_clicked();
	// void on_Button_rceive_clear_clicked();
	// void on_comboBox_choose_currentTextChanged();

private:
    Ui::Form_Network *ui;
	//tcpip_network CNetwork;
	
	// bool bThreadNetwork = true;
	// void thread_Network_Color();
	// std::thread backgroundWorker_Network;
};

#endif // FORM_NETWORK_H
