#include "../include/Form_Vision.h"

#define IMG_WIDTH 1280
#define IMG_HEIGHT 720

Form_Vision::Form_Vision(QWidget *parent) : QDialog(parent),
											ui(new Ui::Form_Vision)
{
	ui->setupUi(this);
	InitialParameters();
}

Form_Vision::~Form_Vision()
{
	bThreadVisionRun = false;
	delete ui;
}
//--------------------QT Form Event--------------------------------
void Form_Vision::showEvent(QShowEvent *ev)
{
	//cout << "The Form_Vision show " << endl;
}
//---------------------CheckBox-------------------------------------------
// void Form_Vision::on_checkBox_EnableColorImage_stateChanged(int arg1)
// {
// 	CMay->CMayVision->bEnableColorImage = this->ui->checkBox_EnableColorImage->isChecked();
// }
// void Form_Vision::on_checkBox_EnableDepthImage_stateChanged(int arg1)
// {
// 	CMay->CMayVision->bEnableDepthImage = this->ui->checkBox_EnableDepthImage->isChecked();
// }
void Form_Vision::on_checkBox_BlurImage_stateChanged(int arg1)
{
	if (!this->ui->checkBox_BlurImage->isChecked())
		CMay->CMayVision->bEnableBlurImage = false;
}
void Form_Vision::on_checkBox_BinaryImage_stateChanged(int arg1)
{
	if (!this->ui->checkBox_BinaryImage->isChecked())
		CMay->CMayVision->bEnableBinaryImage = false;
}
void Form_Vision::on_checkBox_FindCircles_stateChanged(int arg1)
{
	this->ui->textBox_FindCirclesThreshold->setEnabled(true);

	if (!this->ui->checkBox_FindCircles->isChecked())
		CMay->CMayVision->bEnableFindCircles = false;
}
void Form_Vision::on_checkBox_EnableColorImageLeft_Zed_stateChanged(int arg1)
{
	CMay->CMayVision->bEnableColorImageLeft_Zed = this->ui->checkBox_EnableColorImageLeft_Zed->isChecked();
}
void Form_Vision::on_checkBox_EnableDepthImageLeft_Zed_stateChanged(int arg1)
{
	CMay->CMayVision->bEnableDepthImageLeft_Zed = this->ui->checkBox_EnableDepthImageLeft_Zed->isChecked();
}
void Form_Vision::on_checkBox_EnableColorImageRight_Zed_stateChanged(int arg1)
{
	CMay->CMayVision->bEnableColorImageRight_Zed = this->ui->checkBox_EnableColorImageRight_Zed->isChecked();
}
void Form_Vision::on_checkBox_EnableDepthImageRight_Zed_stateChanged(int arg1)
{
	CMay->CMayVision->bEnableDepthImageRight_Zed = this->ui->checkBox_EnableDepthImageRight_Zed->isChecked();
}
void Form_Vision::on_CB_EnableBoundingBox_stateChanged()
{
	CMay->CMayVision->bEnableShowBoundingBox_Zed = this->ui->CB_EnableBoundingBox->isChecked();
}
void Form_Vision::on_PB_OrientationRecognition_clicked()
{
	
	//Loading Image -------------------------------------------
    //CMay->CMayVision->StartSubFromCamera();
    // ros::AsyncSpinner spinner(2); // Use 2 threads
    // spinner.start();
    while(!CMay->CMayVision->CImageConverter->image_sub_flag || !CMay->CMayVision->CImageConverter->depth_sub_flag)
    {
	    this_thread::sleep_for(chrono::milliseconds(10));
    }
    CMay->CMayVision->CImageConverter->GetImgForProcessing(0);
    CMay->CMayVision->CImageConverter->GetImgForProcessing(1);
    CMay->CMayStrategyRunner->COrientation_Rec->RGB_img = CMay->CMayVision->CImageConverter->img_for_processing.clone();
    CMay->CMayStrategyRunner->COrientation_Rec->Depth_img = CMay->CMayVision->CImageConverter->depth_for_processing.clone();
	cv::Mat RGB_img = CMay->CMayStrategyRunner->COrientation_Rec->RGB_img;

    // Loading ROI ----------------------------------------------
    vector<string> ROI_Paramiter;
    fstream fin;
    string line;
    fin.open("/home/bob/catkin_ws/src/may/src/Database/ROI/ROI.txt", ios::in);
    while (getline(fin, line, '\n')) {
        ROI_Paramiter.push_back(line);
    }
    fin.close();
    CMay->CMayStrategyRunner->COrientation_Rec->D_min=stoi(ROI_Paramiter[0]);
    CMay->CMayStrategyRunner->COrientation_Rec->D_max=stoi(ROI_Paramiter[1]);
    CMay->CMayStrategyRunner->COrientation_Rec->X_min=stoi(ROI_Paramiter[2]);
    CMay->CMayStrategyRunner->COrientation_Rec->X_max=stoi(ROI_Paramiter[3]);
    CMay->CMayStrategyRunner->COrientation_Rec->Y_min=stoi(ROI_Paramiter[4]);
    CMay->CMayStrategyRunner->COrientation_Rec->Y_max=stoi(ROI_Paramiter[5]);
	// Find ROI Center -> Make 2 RGBD Image -> Publish Img -> Subscribe Orientation -> Return 8 points
	CMay->CMayStrategyRunner->COrientation_Rec->Draw_Bounding_Box(0);

	// Drawing Bounding Box
	//3D Point to 2D Pix ------------------------------------------------------
	int ptu[9];
    int ptv[9];
    cv::Mat pt8_Zed_true = CMay->CMayStrategyRunner->COrientation_Rec->pt8_for_Bounding_Box;
	this->ui->LE_Point1_X->setText(QString::number(pt8_Zed_true.at<float>(0,0)));
	this->ui->LE_Point1_Y->setText(QString::number(pt8_Zed_true.at<float>(1,0)));
	this->ui->LE_Point1_Z->setText(QString::number(pt8_Zed_true.at<float>(2,0)));

	this->ui->LE_Point2_X->setText(QString::number(pt8_Zed_true.at<float>(0,1)));
	this->ui->LE_Point2_Y->setText(QString::number(pt8_Zed_true.at<float>(1,1)));
	this->ui->LE_Point2_Z->setText(QString::number(pt8_Zed_true.at<float>(2,1)));
	
	this->ui->LE_Point3_X->setText(QString::number(pt8_Zed_true.at<float>(0,2)));
	this->ui->LE_Point3_Y->setText(QString::number(pt8_Zed_true.at<float>(1,2)));
	this->ui->LE_Point3_Z->setText(QString::number(pt8_Zed_true.at<float>(2,2)));

	this->ui->LE_Point4_X->setText(QString::number(pt8_Zed_true.at<float>(0,3)));
	this->ui->LE_Point4_Y->setText(QString::number(pt8_Zed_true.at<float>(1,3)));
	this->ui->LE_Point4_Z->setText(QString::number(pt8_Zed_true.at<float>(2,3)));

	this->ui->LE_Point5_X->setText(QString::number(pt8_Zed_true.at<float>(0,4)));
	this->ui->LE_Point5_Y->setText(QString::number(pt8_Zed_true.at<float>(1,4)));
	this->ui->LE_Point5_Z->setText(QString::number(pt8_Zed_true.at<float>(2,4)));

	this->ui->LE_Point6_X->setText(QString::number(pt8_Zed_true.at<float>(0,5)));
	this->ui->LE_Point6_Y->setText(QString::number(pt8_Zed_true.at<float>(1,5)));
	this->ui->LE_Point6_Z->setText(QString::number(pt8_Zed_true.at<float>(2,5)));

	this->ui->LE_Point7_X->setText(QString::number(pt8_Zed_true.at<float>(0,6)));
	this->ui->LE_Point7_Y->setText(QString::number(pt8_Zed_true.at<float>(1,6)));
	this->ui->LE_Point7_Z->setText(QString::number(pt8_Zed_true.at<float>(2,6)));

	this->ui->LE_Point8_X->setText(QString::number(pt8_Zed_true.at<float>(0,7)));
	this->ui->LE_Point8_Y->setText(QString::number(pt8_Zed_true.at<float>(1,7)));
	this->ui->LE_Point8_Z->setText(QString::number(pt8_Zed_true.at<float>(2,7)));

	this->ui->LE_Point9_X->setText(QString::number(pt8_Zed_true.at<float>(0,8)));
	this->ui->LE_Point9_Y->setText(QString::number(pt8_Zed_true.at<float>(1,8)));
	this->ui->LE_Point9_Z->setText(QString::number(pt8_Zed_true.at<float>(2,8)));

    for (int ptr_num = 0; ptr_num < 9; ptr_num++)
			{
				float fx = CMay->CMayVision->CImageConverter->fx;// 1401.46;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.fx;
				float fy = CMay->CMayVision->CImageConverter->fy;//1401.46;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.fy;
				float cx = CMay->CMayVision->CImageConverter->cx;//1046.08;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.cx;
				float cy = CMay->CMayVision->CImageConverter->cy;//636.451;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.cy;
				float k1 = CMay->CMayVision->CImageConverter->k1;//-0.174071;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[0];
				float k2 = CMay->CMayVision->CImageConverter->k2;//0.027556;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[1];
				float k3 = CMay->CMayVision->CImageConverter->k3;//0;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[2];
				float p1 = CMay->CMayVision->CImageConverter->p1;//0;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[3];
				float p2 = CMay->CMayVision->CImageConverter->p2;//0;//CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[4];
				// float x_ = pt8_Zed_true->at<float>(0,ptr_num) / pt8_Zed_true->at<float>(2,ptr_num);
				// float y_ = pt8_Zed_true->at<float>(1,ptr_num) / pt8_Zed_true->at<float>(2,ptr_num);;
                float x_ = -pt8_Zed_true.at<float>(1,ptr_num) / pt8_Zed_true.at<float>(0,ptr_num);
				float y_ = -pt8_Zed_true.at<float>(2,ptr_num) / pt8_Zed_true.at<float>(0,ptr_num);
				float r2 = x_*x_ + y_*y_;
				float x__ = x_*(1 + k1*r2 + k2*r2*r2) + 2 * p1*x_*y_ + p2*(r2 + 2 * x_*x_);
				float y__ = y_*(1 + k1*r2 + k2*r2*r2) + p1*(r2 + 2 * y_*y_) + 2 * p2*x_*y_;
				float u = fx*x__ + cx;
				float v = fy*y__ + cy;
				cout << "u " << u << " v " << v << endl;
				//cv::circle(RGB_img, Point(u, v), 1, Scalar(0, 255, 0), 20);
				ptu[ptr_num]=u-30;
				ptv[ptr_num] = v;
			}
    // Show
    cv::line(RGB_img, Point(ptu[0], ptv[0]), Point(ptu[1], ptv[1]), Scalar(0, 255, 0), 2);
    cv::line(RGB_img, Point(ptu[2], ptv[2]), Point(ptu[3], ptv[3]), Scalar(0, 255, 0), 2);
    cv::line(RGB_img, Point(ptu[4], ptv[4]), Point(ptu[5], ptv[5]), Scalar(0, 255, 0), 2);
    cv::line(RGB_img, Point(ptu[6], ptv[6]), Point(ptu[7], ptv[7]), Scalar(0, 255, 0), 2);
    
    cv::line(RGB_img, Point(ptu[0], ptv[0]), Point(ptu[3], ptv[3]), Scalar(0, 0, 255), 2);
    cv::line(RGB_img, Point(ptu[1], ptv[1]), Point(ptu[2], ptv[2]), Scalar(0, 0, 255), 2);
    cv::line(RGB_img, Point(ptu[4], ptv[4]), Point(ptu[7], ptv[7]), Scalar(0, 0, 255), 2);
    cv::line(RGB_img, Point(ptu[5], ptv[5]), Point(ptu[6], ptv[6]), Scalar(0, 0, 255), 2);
    
    cv::line(RGB_img, Point(ptu[0], ptv[0]), Point(ptu[4], ptv[4]), Scalar(255, 0, 0), 2);
    cv::line(RGB_img, Point(ptu[1], ptv[1]), Point(ptu[5], ptv[5]), Scalar(255, 0, 0), 2);
    cv::line(RGB_img, Point(ptu[2], ptv[2]), Point(ptu[6], ptv[6]), Scalar(255, 0, 0), 2);
    cv::line(RGB_img, Point(ptu[3], ptv[3]), Point(ptu[7], ptv[7]), Scalar(255, 0, 0), 2);

    cv::circle(RGB_img, Point(ptu[8],ptv[8]), 2, Scalar(0, 0, 0),20);


    cv::namedWindow("Bounding Box", CV_WINDOW_FREERATIO);
    cv::resizeWindow("Bounding Box", 640, 360);
    imshow("Bounding Box",RGB_img);
    
    cv::waitKey(0);

	
}
//------Button--------------------------------------
void Form_Vision::on_button_WbCm_Str_clicked()
{
	CMay->CMayVision->WebCam_Start();
}
void Form_Vision::on_button_WbCm_Stop_clicked()
{
	CMay->CMayVision->WebCam_Stop();
}
void Form_Vision::on_button_Save_ColorFilter_clicked()
{
	//ofstream outfile;
	//outfile.open(".\\Database\\ColorFilter\\yourfile.txt");
	//outfile << "your data 1"<<endl;
	//outfile << "your data 2" << endl;

	//ui->button_Save_ColorFilter->setToolTip(tr("Save current color filter settings"));

	QString homepath = getenv("HOME");
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save current color filter settings"),
	 homepath + "/catkin_ws/src/may/src/Database/ColorFilter/",
	  "Text files (*.txt)");

	QFileInfo FileNameandPath(fileName);
	string File_Path = FileNameandPath.path().toStdString();
	string File_Name_Path = FileNameandPath.filePath().toStdString();
	string File_Name = FileNameandPath.fileName().toStdString();

	if (fileName.isEmpty())
		return;
	else
	{
		QFile file(fileName);
		if (!file.open(QIODevice::WriteOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"),
									 file.errorString());
			return;
		}

		ofstream outfile;
		outfile.open(File_Name_Path);
		outfile << ui->hScrollBar_Hmin->value() << endl;
		outfile << ui->hScrollBar_Hmax->value() << endl;
		outfile << ui->hScrollBar_Smin->value() << endl;
		outfile << ui->hScrollBar_Smax->value() << endl;
		outfile << ui->hScrollBar_Vmin->value() << endl;
		outfile << ui->hScrollBar_Vmax->value() << endl;
	}
}
void Form_Vision::on_button_Load_ColorFilter_clicked()
{
	QLineEdit *nameLine;
	//QTextEdit *addressText;
	QString homepath = getenv("HOME");
	QString fileName = QFileDialog::getOpenFileName(this,
													tr("Load color filter settings"), homepath + "/catkin_ws/src/may/src/Database/ColorFilter",
													"Text file (*.txt)");
	// tr("txt files (*.txt)|*.txt*.*")
	QFileInfo FileNameandPath(fileName);
	string File_Path = FileNameandPath.path().toStdString();
	string File_Name_Path = FileNameandPath.filePath().toStdString();
	string File_Name = FileNameandPath.fileName().toStdString();

	if (fileName.isEmpty())
		return;
	else
	{

		QFile file(fileName);

		if (!file.open(QIODevice::ReadOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"),
									 file.errorString());
			return;
		}
		fstream fin;
		string line;
		vector<string> HSV_number;
		fin.open(File_Name_Path, ios::in);
		while (getline(fin, line, '\n'))
		{
			//cout << line << endl;
			HSV_number.push_back(line);
		}
		fin.close();
		ui->hScrollBar_Hmin->setValue(stoi(HSV_number[0]));
		ui->hScrollBar_Hmax->setValue(stoi(HSV_number[1]));
		ui->hScrollBar_Smin->setValue(stoi(HSV_number[2]));
		ui->hScrollBar_Smax->setValue(stoi(HSV_number[3]));
		ui->hScrollBar_Vmin->setValue(stoi(HSV_number[4]));
		ui->hScrollBar_Vmax->setValue(stoi(HSV_number[5]));
	}
}

void Form_Vision::on_PB_Save_ROI_clicked()
{
	QString homepath = getenv("HOME");
	QString fileName = QFileDialog::getSaveFileName(this,
													tr("Save current color filter settings"), homepath + "/catkin_ws/src/may/src/Database/ROI",
													"Text files (*.txt)");

	QFileInfo FileNameandPath(fileName);
	string File_Path = FileNameandPath.path().toStdString();
	string File_Name_Path = FileNameandPath.filePath().toStdString();
	string File_Name = FileNameandPath.fileName().toStdString();

	if (fileName.isEmpty())
		return;
	else
	{
		QFile file(fileName);
		if (!file.open(QIODevice::WriteOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"),
									 file.errorString());
			return;
		}

		ofstream outfile;
		outfile.open(File_Name_Path);
		outfile << ui->Slider_Dmin->value() << endl;
		outfile << ui->Slider_Dmax->value() << endl;
		outfile << ui->Slider_Xmin->value() << endl;
		outfile << ui->Slider_Xmax->value() << endl;
		outfile << ui->Slider_Ymin->value() << endl;
		outfile << ui->Slider_Ymax->value() << endl;
	}
}

void Form_Vision::on_PB_Publish_clicked()
{
	CMay->CMayVision->CImageConverter->PubImg();
	CMay->CMayVision->CPclConverter->SubInfo();
	CMay->CMayVision->CPclConverter->GetOrientation();

}

void Form_Vision::on_PB_BoundingBoxTest_clicked()
{
	// Create Testing Picture
	//Loading Image -------------------------------------------
    //CMay->CMayVision->StartSubFromCamera();
    // ros::AsyncSpinner spinner(2); // Use 2 threads
    // spinner.start();
    while(!CMay->CMayVision->CImageConverter->image_sub_flag || !CMay->CMayVision->CImageConverter->depth_sub_flag)
    {
	    this_thread::sleep_for(chrono::milliseconds(10));
    }
    CMay->CMayVision->CImageConverter->GetImgForProcessing(0);
    CMay->CMayVision->CImageConverter->GetImgForProcessing(1);
    CMay->CMayStrategyRunner->COrientation_Rec->RGB_img = CMay->CMayVision->CImageConverter->img_for_processing.clone();
    CMay->CMayStrategyRunner->COrientation_Rec->Depth_img = CMay->CMayVision->CImageConverter->depth_for_processing.clone();
	cv::Mat RGB_img = CMay->CMayStrategyRunner->COrientation_Rec->RGB_img;

    // Loading ROI ----------------------------------------------
    vector<string> ROI_Paramiter;
    fstream fin;
    string line;
    fin.open("/home/bob/catkin_ws/src/may/src/Database/ROI/ROI.txt", ios::in);
    while (getline(fin, line, '\n')) {
        ROI_Paramiter.push_back(line);
    }
    fin.close();
    CMay->CMayStrategyRunner->COrientation_Rec->D_min=stoi(ROI_Paramiter[0]);
    CMay->CMayStrategyRunner->COrientation_Rec->D_max=stoi(ROI_Paramiter[1]);
    CMay->CMayStrategyRunner->COrientation_Rec->X_min=stoi(ROI_Paramiter[2]);
    CMay->CMayStrategyRunner->COrientation_Rec->X_max=stoi(ROI_Paramiter[3]);
    CMay->CMayStrategyRunner->COrientation_Rec->Y_min=stoi(ROI_Paramiter[4]);
    CMay->CMayStrategyRunner->COrientation_Rec->Y_max=stoi(ROI_Paramiter[5]);

	CMay->CMayStrategyRunner->COrientation_Rec->DataNum = this->ui->LE_BoundingBoxPicReal_DataNum->text().toDouble();
	CMay->CMayStrategyRunner->COrientation_Rec->view_port = this->ui->LE_BoundingBoxPicReal_Ans->text().toDouble();

	// Take Picture
	CMay->CMayStrategyRunner->COrientation_Rec->Save_Training_Pic();

}
void Form_Vision::on_PB_Load_ROI_clicked()
{
	QLineEdit *nameLine;
	//QTextEdit *addressText;
	QString homepath = getenv("HOME");
	QString fileName = QFileDialog::getOpenFileName(this,
													tr("Load color filter settings"), homepath + "/catkin_ws/src/may/src/Database/ROI",
													"Text file (*.txt)");
	// tr("txt files (*.txt)|*.txt*.*")
	QFileInfo FileNameandPath(fileName);
	string File_Path = FileNameandPath.path().toStdString();
	string File_Name_Path = FileNameandPath.filePath().toStdString();
	string File_Name = FileNameandPath.fileName().toStdString();

	if (fileName.isEmpty())
		return;
	else
	{

		QFile file(fileName);

		if (!file.open(QIODevice::ReadOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"),
									 file.errorString());
			return;
		}
		fstream fin;
		string line;
		vector<string> ROI_Paramiter;
		fin.open(File_Name_Path, ios::in);
		while (getline(fin, line, '\n'))
		{
			//cout << line << endl;
			ROI_Paramiter.push_back(line);
		}
		fin.close();
		ui->Slider_Dmin->setValue(stoi(ROI_Paramiter[0]));
		ui->Slider_Dmax->setValue(stoi(ROI_Paramiter[1]));
		ui->Slider_Xmin->setValue(stoi(ROI_Paramiter[2]));
		ui->Slider_Xmax->setValue(stoi(ROI_Paramiter[3]));
		ui->Slider_Ymin->setValue(stoi(ROI_Paramiter[4]));
		ui->Slider_Ymax->setValue(stoi(ROI_Paramiter[5]));
	}
}
//------ScrollBar---------------------------------------
void Form_Vision::on_hScrollBar_Hmin_valueChanged()
{
	this->ui->label_Hmin_Value->setText(QString::number(ui->hScrollBar_Hmin->value())); //->Text = System::Convert::ToString(cvRound((this->hScrollBar_Hmin->Value / 91.0) * 180.0));
}
void Form_Vision::on_hScrollBar_Hmax_valueChanged()
{
	this->ui->label_Hmax_Value->setText(QString::number(ui->hScrollBar_Hmax->value())); //->Text = System::Convert::ToString(cvRound((this->hScrollBar_Hmin->Value / 91.0) * 180.0));
}
void Form_Vision::on_hScrollBar_Smin_valueChanged()
{
	this->ui->label_Smin_Value->setText(QString::number(ui->hScrollBar_Smin->value())); //->Text = System::Convert::ToString(cvRound((this->hScrollBar_Hmin->Value / 91.0) * 180.0));
}
void Form_Vision::on_hScrollBar_Smax_valueChanged()
{
	this->ui->label_Smax_Value->setText(QString::number(ui->hScrollBar_Smax->value())); //->Text = System::Convert::ToString(cvRound((this->hScrollBar_Hmin->Value / 91.0) * 180.0));
}
void Form_Vision::on_hScrollBar_Vmin_valueChanged()
{
	this->ui->label_Vmin_Value->setText(QString::number(ui->hScrollBar_Vmin->value())); //->Text = System::Convert::ToString(cvRound((this->hScrollBar_Hmin->Value / 91.0) * 180.0));
}
void Form_Vision::on_hScrollBar_Vmax_valueChanged()
{
	this->ui->label_Vmax_Value->setText(QString::number(ui->hScrollBar_Vmax->value())); //->Text = System::Convert::ToString(cvRound((this->hScrollBar_Hmin->Value / 91.0) * 180.0));
}
void Form_Vision::on_Slider_Dmin_valueChanged()
{
	this->ui->LB_Dmin->setText(QString::number(ui->Slider_Dmin->value()));
}

void Form_Vision::on_Slider_Dmax_valueChanged()
{
	this->ui->LB_Dmax->setText(QString::number(ui->Slider_Dmax->value()));
}

void Form_Vision::on_Slider_Xmin_valueChanged()
{
	this->ui->LB_Xmin->setText(QString::number(ui->Slider_Xmin->value()));
}

void Form_Vision::on_Slider_Xmax_valueChanged()
{
	this->ui->LB_Xmax->setText(QString::number(ui->Slider_Xmax->value()));
}

void Form_Vision::on_Slider_Ymin_valueChanged()
{
	this->ui->LB_Ymin->setText(QString::number(ui->Slider_Ymin->value()));
}

void Form_Vision::on_Slider_Ymax_valueChanged()
{
	this->ui->LB_Ymax->setText(QString::number(ui->Slider_Ymax->value()));
}
//---------------------trackBar------------------------------------
void Form_Vision::on_trackBar_BlurMaskSize_valueChanged()
{
	CMay->CMayVision->CImageProcessing->nBlurMaskSize = this->ui->trackBar_BlurMaskSize->value() * 2 - 1;
	this->ui->label_BinaryThreshold->setText(QString::number(this->ui->hScrollBar_BinaryImage->value()));
}
// ---------------------Radio-------------------------------
void Form_Vision::on_radioButton_Morphology_No_toggled()
{
	this->ui->comboBox_MorphologyMaskSize->setEnabled(false);
	this->ui->comboBox_MorphologyTimes->setEnabled(false);

	if (this->ui->radioButton_Morphology_No->isChecked())
		CMay->CMayVision->bEnableMorphology = false;
}
void Form_Vision::on_radioButton_Dilate_toggled()
{
	this->ui->comboBox_MorphologyMaskSize->setEnabled(true);
	this->ui->comboBox_MorphologyTimes->setEnabled(true);
}
void Form_Vision::on_radioButton_Erode_toggled()
{
	this->ui->comboBox_MorphologyMaskSize->setEnabled(true);
	this->ui->comboBox_MorphologyTimes->setEnabled(true);
}
void Form_Vision::on_radioButton_OpeningOperation_toggled()
{
	this->ui->comboBox_MorphologyMaskSize->setEnabled(true);
	this->ui->comboBox_MorphologyTimes->setEnabled(true);
}
void Form_Vision::on_radioButton_ClosingOperation_toggled()
{
	this->ui->comboBox_MorphologyMaskSize->setEnabled(true);
	this->ui->comboBox_MorphologyTimes->setEnabled(true);
}

void Form_Vision::on_radioButton_AllContours_toggled()
{
	this->ui->textBox_MinContourArea->setEnabled(true);
	this->ui->textBox_RefineValue->setEnabled(true);
}
void Form_Vision::on_radioButton_MaxContour_toggled()
{
	this->ui->textBox_MinContourArea->setEnabled(false);
	this->ui->textBox_RefineValue->setEnabled(true);
}
void Form_Vision::on_radioButton_NoContour_toggled()
{
	this->ui->textBox_MinContourArea->setEnabled(false);
	this->ui->textBox_RefineValue->setEnabled(false);
	CMay->CMayVision->bEnableFindContours = false;
	CMay->CMayVision->CImageProcessing->CContourTool->vec_ptCenters.clear();
	CMay->CMayVision->CImageProcessing->CContourTool->vec2_ptContours.clear();
}
//----------------------comboBox--------------------------------------
void Form_Vision::on_comboBox_MorphologyMaskSize_activated(int arg)
{
	QString a = this->ui->comboBox_MorphologyMaskSize->currentText();
	bool ok = true;
	CMay->CMayVision->nMorphologyMaskSize = a.toInt(&ok, 10);
}
void Form_Vision::on_comboBox_MorphologyTimes_activated(int arg)
{
	QString a = this->ui->comboBox_MorphologyTimes->currentText();
	bool ok = true;
	CMay->CMayVision->nMorphologyTimes = a.toInt(&ok, 10);
}

//------------------textLine
void Form_Vision::on_textBox_MinContourArea_textChanged(const QString &arg1)
{
	if (this->ui->textBox_MinContourArea->text() == "")
		return;
	//for each (Char ch in this->textBox_MinContourArea->Text)
	//{
	//	if (!Char::IsDigit(ch))
	//		return;
	//}

	int nMinArea = this->ui->textBox_MinContourArea->text().toDouble();

	if (nMinArea > 0 && (nMinArea < Width_Color_Frame * Height_Color_Frame))
		CMay->CMayVision->CImageProcessing->CContourTool->nMinContourArea = nMinArea;
}
void Form_Vision::on_textBox_RefineValue_textChanged(const QString &arg1)
{
	if (this->ui->textBox_MinContourArea->text() == "")
		return;
	//for each (Char ch in this->ui->textBox_MinContourArea->text)
	//{
	//	if (!Char::IsDigit(ch))
	//		return;
	//}

	int nMinArea = this->ui->textBox_MinContourArea->text().toDouble();

	if (nMinArea > 0 && (nMinArea < Width_Color_Frame * Height_Color_Frame))
		CMay->CMayVision->CImageProcessing->CContourTool->nMinContourArea = nMinArea;
}
void Form_Vision::on_textBox_FindCirclesThreshold_textChanged(const QString &arg1)
{
	// 檢查有無字元
	if (this->ui->textBox_FindCirclesThreshold->text() == "")
		return;
	// 檢查是否皆為數字
	//for each (Char ch in this->textBox_FindCirclesThreshold->Text)
	//{
	//	if (!Char::IsDigit(ch))
	//		return;
	//}

	int nThreshold = this->ui->textBox_FindCirclesThreshold->text().toDouble();

	CMay->CMayVision->CImageProcessing->CCircleTool->nCenterDetectionThreshold = nThreshold;
}

//
//
//-----------------------------------------------------------------
//-----------------Original .h file
void Form_Vision::InitialParameters(void)
{
	this->ui->label_Hmin_Value->setText(QString::number(this->ui->hScrollBar_Hmin->value()));
	//QString s = "123";
	//this->ui->label_Hmin_Value->setText(s);
	this->ui->label_Hmax_Value->setText(QString::number(this->ui->hScrollBar_Hmax->value()));
	this->ui->label_Smin_Value->setText(QString::number(this->ui->hScrollBar_Smin->value()));
	this->ui->label_Smax_Value->setText(QString::number(this->ui->hScrollBar_Smax->value()));
	this->ui->label_Vmin_Value->setText(QString::number(this->ui->hScrollBar_Vmin->value()));
	this->ui->label_Vmax_Value->setText(QString::number(this->ui->hScrollBar_Vmax->value()));

	this->ui->label_BinaryThreshold->setText(QString::number(this->ui->hScrollBar_BinaryImage->value()));

	//this->form_AddNewObjectToDatabase = gcnew Form_AddNewObjectToDatabase;
	//this->form_AddNewObjectToDatabase->Show();
	//this->form_AddNewObjectToDatabase->Hide();

	CMay->CMayVision->CImageProcessing->CContourTool->nMinContourArea = this->ui->textBox_MinContourArea->text().toDouble();
	CMay->CMayVision->CImageProcessing->CContourTool->ApproximatecontoursValue = this->ui->textBox_RefineValue->text().toDouble();
}

//-----------------Original .cpp file
bool bMouseOn_ColorImage = false;
int x_ColorImageMouseDown = -1;
int y_ColorImageMouseDown = -1;
int x_ColorImageMouseNow = -1;
int y_ColorImageMouseNow = -1;
int count_save_mask = 0;

enum MorphologyState
{
	Dilating,
	Eroding,
	Closing,
	Opening
};

MorphologyState enumMorphologyState;

enum ContourCase
{
	MaxContour,
	AllContours
};

ContourCase enumContourCase;

void onMouse_ColorImage(int event, int x, int y, int flag, void *param)
{
	// 按下滑鼠左鍵?
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		// 使用追蹤
		if (CMay->CMayVision->bEnableCompressiveTracking || CMay->CMayVision->bEnableCMT)
		{
			CMay->CMayVision->rectTrackingTarget.x = x;
			CMay->CMayVision->rectTrackingTarget.y = y;
			x_ColorImageMouseDown = x;
			y_ColorImageMouseDown = y;
		}
		bMouseOn_ColorImage = true;

		if (CMay->CMayVision->bEnableDatabaseObjectMaking)
		{
			CMay->CMayVision->vec_ptObjContour.push_back(cv::Point(x, y));
		}

		if (CMay->CMayVision->bsave_mask)
		{

			// float xWorld;
			// float yWorld;
			// float zWorld;

			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X))
			// 	xWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X;
			// else
			// 	xWorld = -1;
			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y))
			// 	yWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y;
			// else
			// 	yWorld = -1;
			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z))
			// 	zWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z;
			// else
			// 	zWorld = -1;

			// //To Be Fixed
			// //System::IO::StreamWriter^ sw = gcnew System::IO::StreamWriter(".//Database//Calibration//mask_point.txt", true);

			// //if (count_save_mask < 4) {  //save four edge point
			// //	System::String^ str = x_ColorImageMouseNow + "," + y_ColorImageMouseNow;
			// //	sw->WriteLine(str);
			// //}

			// //sw->Close();
			// //delete sw;
		}
		if (CMay->CMayVision->bsave_putArea)
		{

			// float xWorld;
			// float yWorld;
			// float zWorld;

			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X))
			// 	xWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X;
			// else
			// 	xWorld = -1;
			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y))
			// 	yWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y;
			// else
			// 	yWorld = -1;
			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z))
			// 	zWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z;
			// else
			// 	zWorld = -1;

			// //To Be Fixed
			// //System::IO::StreamWriter^ sw = gcnew System::IO::StreamWriter(".//Database//Calibration//put_area.txt", true);

			// //if (count_save_mask < 4) {  //save four edge point
			// //	System::String^ str = x_ColorImageMouseNow + "," + y_ColorImageMouseNow;
			// //	sw->WriteLine(str);
			// //}

			// //sw->Close();
			// //delete sw;
		}

		if (CMay->CMayVision->bsave_vision_points)
		{

			// float xWorld;
			// float yWorld;
			// float zWorld;

			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X))
			// 	xWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X;
			// else
			// 	xWorld = -1;
			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y))
			// 	yWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y;
			// else
			// 	yWorld = -1;
			// if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z))
			// 	zWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z;
			// else
			// 	zWorld = -1;

			// //To Be Fixed
			// //System::IO::StreamWriter^ sw = gcnew System::IO::StreamWriter(".//Database//Calibration//KinectPoints.txt", true);
			// //System::String^ str = xWorld + "," + yWorld + "," + zWorld;

			// //sw->WriteLine(str);
			// //sw->Close();
			// //delete sw;
		}
	}
	// 放開滑鼠左鍵
	if (event == CV_EVENT_LBUTTONUP)
	{
		// 結束追蹤選取物品
		if (CMay->CMayVision->bEnableCompressiveTracking || CMay->CMayVision->bEnableCMT)
		{
			CMay->CMayVision->bTargetSelected = true;
			x_ColorImageMouseDown = -1;
			y_ColorImageMouseDown = -1;
		}

		bMouseOn_ColorImage = false;
	}
	// 放開滑鼠右鍵
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		if (CMay->CMayVision->bEnableDatabaseObjectMaking)
			CMay->CMayVision->bNewObjectSelected = true;
	}
	// 在視窗移動
	if (event == CV_EVENT_MOUSEMOVE)
	{
		x_ColorImageMouseNow = x;
		y_ColorImageMouseNow = y;
		//cout << "x:" << x_ColorImageMouseNow<<endl;
		//cout << "y:" << y_ColorImageMouseNow<< endl;

		// 追蹤選取物品
		if ((CMay->CMayVision->bEnableCompressiveTracking || CMay->CMayVision->bEnableCMT) && bMouseOn_ColorImage)
		{
			int nWidth = x - x_ColorImageMouseDown;
			int nHeight = y - y_ColorImageMouseDown;

			if (nWidth < 0)
			{
				CMay->CMayVision->rectTrackingTarget.x = x;
			}
			else if (nWidth > 0)
			{
				CMay->CMayVision->rectTrackingTarget.x = x_ColorImageMouseDown;
			}

			if (nHeight < 0)
			{
				CMay->CMayVision->rectTrackingTarget.y = y;
			}
			else if (nHeight > 0)
			{
				CMay->CMayVision->rectTrackingTarget.y = y_ColorImageMouseDown;
			}

			CMay->CMayVision->rectTrackingTarget.width = abs(nWidth);
			CMay->CMayVision->rectTrackingTarget.height = abs(nHeight);
		}
	}
	else
	{
		x_ColorImageMouseNow = -1;
		y_ColorImageMouseNow = -1;
	}
}

void onMouse_Zed(int event, int x, int y, int flag, void *param)
{
	if (event == CV_EVENT_MOUSEMOVE)
	{
		x_ColorImageMouseNow = x;
		y_ColorImageMouseNow = y;
	}
	else
	{
		x_ColorImageMouseNow = -1;
		y_ColorImageMouseNow = -1;
	}
}

void Form_Vision::create_color_image_window(void)
{
	cv::namedWindow("Color_Image", CV_WINDOW_FREERATIO);
	cv::resizeWindow("Color_Image", 640, 360);
	cv::moveWindow("Color_Image", 0, 0);
	cv::setMouseCallback("Color_Image", onMouse_ColorImage, NULL);

	this->bColorImageWindowCreated = true;
}

void Form_Vision::create_depth_image_window(void)
{
	cv::namedWindow("Depth Image", CV_WINDOW_FREERATIO);
	cv::resizeWindow("Depth Image", 512, 424);
	cv::moveWindow("Depth Image", 0, 400);

	this->bDepthImageWindowCreated = true;
}

void Form_Vision::create_zed_color_image_window(void)
{
	cv::namedWindow("Zed Color Image", CV_WINDOW_FREERATIO);
	//cv::resizeWindow("Zed Color Image", 640, 360);
	cv::resizeWindow("Zed Color Image", 672, 376);

	cv::moveWindow("Zed Color Image", 0, 0);
	cv::setMouseCallback("Zed Color Image", onMouse_Zed, NULL);

	this->bZedColorImageWindowCreated = true;
}

void Form_Vision::create_zed_depth_image_window(void)
{
	cv::namedWindow("Zed Depth Image", CV_WINDOW_FREERATIO);
	cv::resizeWindow("Zed Depth Image", 672, 376);
	cv::moveWindow("Zed Depth Image", 0, 400);

	this->bZedDepthImageWindowCreated = true;
}

void Form_Vision::run_vision_thread(void)
{
	// 讀入彩色影像
	// if (CMay->CMayVision->bEnableColorImage == true)
	// 	CMay->CMayVision->InputColorImageFromKinect();
	// // 讀入深度影像
	// if (CMay->CMayVision->bEnableDepthImage == true)
	// 	CMay->CMayVision->InputDepthImageFromKinect();

	// 讀入彩色影像 Zed Left
	if (CMay->CMayVision->bEnableColorImageLeft_Zed == true)
		CMay->CMayVision->InputColorImageFromZedLeft();
	// 讀入深度影像 Zed Left
	if (CMay->CMayVision->bEnableDepthImageLeft_Zed == true)
		CMay->CMayVision->InputDepthImageFromZedLeft();
	// 讀入彩色影像 Zed Right
	if (CMay->CMayVision->bEnableColorImageRight_Zed == true)
		CMay->CMayVision->InputColorImageFromZedRight();
	// 讀入深度影像 Zed Right
	if (CMay->CMayVision->bEnableDepthImageRight_Zed == true)
		CMay->CMayVision->InputDepthImageFromZedRight();

	// BLS TEST(already have data)
	if (CMay->CMayVision->bEnableBLSTEST)
	{
		// CMay->CMayVision->CBLS->BLS_TEST();
		// CMay->CMayVision->bEnableBLSTEST = false;
	}
	// gesture pic transfer to matrix
	if (CMay->CMayVision->bEnableTrainPicMatrix)
	{
		// CMay->CMayVision->CBLS->trans_pic_matrix();
		// CMay->CMayVision->bEnableTrainPicMatrix = false;
	}

	// 如果有彩色影像的話
	if (CMay->CMayVision->bEnableColorImage == true)
	{

#pragma region Image Processing
		// Color Filter
		if (CMay->CMayVision->bEnableColorFilter)
		{
			CMay->CMayVision->CImageProcessing->ColorFilter(CMay->CMayVision->matColorImageProcessed, this->nHmin, this->nHmax, this->nSmin, this->nSmax, this->nVmin, this->nVmax);
		}

		//To Be Fixed
		// ROI
		//if (this->checkBox_CB_EnableROI->Checked)
		//	CMay->CMayVision->CImageProcessing->RoI(CMay->CMayVision->matColorImageProcessed, this->nXmin, this->nXmax, this->nYmin, this->nYmax);

		//To Be Fixed
		// 高度設定
		//if (this->checkBox_CB_EnableHeightSet->Checked)
		//	CMay->CMayVision->HeightThreshold(CMay->CMayVision->matColorImageProcessed, this->nLine_BallandOther, this->nLine_ObjandHole, this->nHeight_Threshold, this->nHeight_Threshold_mask);

		// Binary
		if (CMay->CMayVision->bEnableBinaryImage)
		{
			CMay->CMayVision->CImageProcessing->BinaryImage(CMay->CMayVision->matColorImageProcessed, this->nBinaryThreshold);
		}
		// Blur
		if (CMay->CMayVision->bEnableBlurImage)
		{
			CMay->CMayVision->CImageProcessing->BlurImage(CMay->CMayVision->matColorImageProcessed);
		}
		// Morphology
		if (CMay->CMayVision->bEnableMorphology)
		{
			switch (enumMorphologyState)
			{
			case MorphologyState::Eroding:
				CMay->CMayVision->CImageProcessing->CMorphologyTool->ErodeImage(CMay->CMayVision->matColorImageProcessed, CMay->CMayVision->nMorphologyMaskSize, CMay->CMayVision->nMorphologyTimes);
				break;
			case MorphologyState::Dilating:
				CMay->CMayVision->CImageProcessing->CMorphologyTool->DilateImage(CMay->CMayVision->matColorImageProcessed, CMay->CMayVision->nMorphologyMaskSize, CMay->CMayVision->nMorphologyTimes);
				break;
			case MorphologyState::Opening:
				CMay->CMayVision->CImageProcessing->CMorphologyTool->OpeningOperation(CMay->CMayVision->matColorImageProcessed, CMay->CMayVision->nMorphologyMaskSize, CMay->CMayVision->nMorphologyTimes);
				break;
			case MorphologyState::Closing:
				CMay->CMayVision->CImageProcessing->CMorphologyTool->ClosingOperation(CMay->CMayVision->matColorImageProcessed, CMay->CMayVision->nMorphologyMaskSize, CMay->CMayVision->nMorphologyTimes);
				break;
			default:
				break;
			}
		}
		// Find Contours
		if (CMay->CMayVision->bEnableFindContours)
		{
			switch (enumContourCase)
			{
			case ContourCase::AllContours:
				CMay->CMayVision->CImageProcessing->CContourTool->FindContours(CMay->CMayVision->matColorImageProcessed, CMay->CMayVision->CImageProcessing->CContourTool->nMinContourArea);
				break;
			case ContourCase::MaxContour:
				CMay->CMayVision->CImageProcessing->CContourTool->FindMaxContours(CMay->CMayVision->matColorImageProcessed);
				break;
			default:
				break;
			}

			CMay->CMayVision->CImageProcessing->CContourTool->RefineToApproximateContours();
		}
		// Find Circles
		if (CMay->CMayVision->bEnableFindCircles)
			CMay->CMayVision->CImageProcessing->CCircleTool->FindCircles(CMay->CMayVision->matColorImageProcessed);

#pragma endregion Image Processing

		if (CMay->CMayVision->rectTrackingTarget.x != -1 && CMay->CMayVision->rectTrackingTarget.y != -1 && CMay->CMayVision->rectTrackingTarget.width != 0 && CMay->CMayVision->rectTrackingTarget.height != 0)
		{
			// Compressive Tracking
			if ((CMay->CMayVision->bEnableCompressiveTracking == true) && (CMay->CMayVision->bTargetSelected == true))
			{
				CMay->CMayVision->DoCompressiveTracking();
			}
			// CMT
			if ((CMay->CMayVision->bEnableCMT == true) && (CMay->CMayVision->bTargetSelected == true))
			{
				CMay->CMayVision->DoCMT();
			}
			if (CMay->CMayVision->bTargetSelected == true)
			{
				cout << "The tracking result is at " << CMay->CMayVision->rectTrackingTarget.x + CMay->CMayVision->rectTrackingTarget.width / 2.0 << ", " << CMay->CMayVision->rectTrackingTarget.y + CMay->CMayVision->rectTrackingTarget.height / 2.0 << endl;
				cout << "\t\t\t\t\twith Width: " << CMay->CMayVision->rectTrackingTarget.width << " Height: " << CMay->CMayVision->rectTrackingTarget.height << endl;
			}
		}

		// // Object Recognition
		// if (CMay->CMayVision->bEnableObjectRecognizing && (CMay->CMayVision->nObjectNumber <= OBJECTS_NUMBER && CMay->CMayVision->nObjectNumber >= 0))
		// {
		// 	CMay->CMayVision->CObjectRecognizer->SetImage(CMay->CMayVision->matColorImageProcessed);
		// 	CMay->CMayVision->CObjectRecognizer->DoDatabaseMatching(CMay->CMayVision->nObjectNumber);
		// }

		// if (CMay->CMayVision->bEnableFaceDetecting)
		// {
		// 	CMay->CMayVision->CFaceRecognizer->detectAndDisplay(CMay->CMayVision->matColorImageProcessed, CMay->CMayVision->bSaveFace, CMay->CMayVision->bLoadFace, CMay->CMayVision->bTrainFace, CMay->CMayVision->nFaceLabel);
		// 	CMay->CMayVision->bSaveFace = false;
		// 	CMay->CMayVision->bLoadFace = false;
		// 	//CMay->CMayVision->bTrainFace = false;
		// }

		// //Initial EyesTracker (Calibration)
		// if (CMay->CMayVision->bEnableInitialEye)
		// {

		// 	if (CMay->CMayVision->CFaceRecognizer->last_time < 10.0) {
		// 		CMay->CMayVision->CFaceRecognizer->initialEyes(CMay->CMayVision->matColorImageProcessed);
		// 	}
		// 	else {
		// 		CMay->CMayVision->bEnableInitialEye = false;
		// 		cvDestroyWindow("initial");               //delete the window
		// 		CMay->CMayVision->CFaceRecognizer->initial_picture.release();
		// 		cout << "initial_Eye End" << endl;
		// 	}
		// }

		// //Tracking eyes
		// if (CMay->CMayVision->bEnableEyeTracker)
		// {
		// 	CMay->CMayVision->CFaceRecognizer->trackingEyes(CMay->CMayVision->matColorImageProcessed);
		// 	CMay->CMayVision->bEnableEyeTracker = false;
		// }

		// //Train Eyes
		// if (CMay->CMayVision->bEnableTrainEye)
		// {
		// 	if (!CMay->CMayVision->CFaceRecognizer->close_file) {
		// 		CMay->CMayVision->CFaceRecognizer->trainEyes(CMay->CMayVision->matColorImageProcessed);
		// 	}
		// 	else {
		// 		CMay->CMayVision->bEnableTrainEye = false;
		// 	}
		// }
	}
#pragma region DRAWING
	if (CMay->CMayVision->bEnableColorImage)
	{
		// 畫出追蹤目標
		if ((CMay->CMayVision->bEnableCompressiveTracking || CMay->CMayVision->bEnableCMT))
			CMay->CMayVision->DrawTrackedTarget();

		// 畫出資料庫物品
		if (CMay->CMayVision->bEnableDatabaseObjectMaking)
			CMay->CMayVision->DrawDatabaseObjectCreatingRegion(CMay->CMayVision->bNewObjectSelected);

		// 畫辨識物品
		if (CMay->CMayVision->bEnableObjectRecognizing)
			CMay->CMayVision->DrawRecognizedObject();

		// 畫輪廓
		if (CMay->CMayVision->bEnableFindContours)
			CMay->CMayVision->DrawContoursFound();

		if (CMay->CMayVision->bEnableFindContours && CMay->CMayVision->bEnableFindContoursPoint)
			CMay->CMayVision->DrawContoursPointFound();

		// 畫圓
		if (CMay->CMayVision->bEnableFindCircles)
			CMay->CMayVision->DrawCirclesFound();

		//To Be Fixed
		// 畫線
		//if (this->checkBox_CB_EnableLine->Checked)
		//	CMay->CMayVision->DrawLineObjandHole(this->nLine_BallandOther, this->nLine_ObjandHole);
	}

	if (CMay->CMayVision->bEnableDepthImage)
	{
		// 畫骨架
		if (CMay->CMayVision->bEnableSkeleton)
			CMay->CMayVision->DrawSkeletonsOnDepthImage();
	}

#pragma endregion DRAWING

	// 將影像存入uchar
	if (CMay->CMayVision->bUpdateColorImage && CMay->CMayVision->bEnableColorImage)
		CMay->CMayVision->colorImageData2Show = CMay->CMayVision->matColorImageProcessed.data;

	if (CMay->CMayVision->bEnableFrontSight)
	{
		line(CMay->CMayVision->matColorImageProcessed, cv::Point(CMay->CMayVision->matColorImageProcessed.cols / 2 - 10, CMay->CMayVision->matColorImageProcessed.rows / 2), cv::Point(CMay->CMayVision->matColorImageProcessed.cols / 2 + 10, CMay->CMayVision->matColorImageProcessed.rows / 2), Scalar(0, 255, 0), 3);
		line(CMay->CMayVision->matColorImageProcessed, cv::Point(CMay->CMayVision->matColorImageProcessed.cols / 2, CMay->CMayVision->matColorImageProcessed.rows / 2 - 10), cv::Point(CMay->CMayVision->matColorImageProcessed.cols / 2, CMay->CMayVision->matColorImageProcessed.rows / 2 + 10), Scalar(0, 255, 0), 3);
	}

	// 顯示彩色影像
	if (CMay->CMayVision->bUpdateColorImage || CMay->CMayVision->bShowColorImage)
	{
		if (!this->bColorImageWindowCreated)
			this->create_color_image_window();

		if ((CMay->CMayVision->bUpdateColorImage && CMay->CMayVision->bEnableColorImage) || CMay->CMayVision->bShowColorImage)
		{
			cv::Mat mat = cv::Mat(CMay->CMayVision->matColorImageProcessed.rows, CMay->CMayVision->matColorImageProcessed.cols, CMay->CMayVision->matColorImageProcessed.type(), CMay->CMayVision->colorImageData2Show);

			if (CMay->CMayVision->bEnableEyeRegion == true)
			{
				cv::Mat mat_eye_region = mat;
				cv::rectangle(mat_eye_region, cv::Point(1920 * 4.45 / 10, 1080 * 2.0 / 10), cv::Point(1920 * 5.55 / 10, 1080 * 2.7 / 10), Scalar(0, 0, 255), 5);
				cv::imshow("Color_Image", mat_eye_region);
			}
			else
			{
				cv::imshow("Color_Image", mat);
			}

			cv::waitKey(1);
			mat.release();

			CMay->CMayVision->bUpdateColorImage = false;
			CMay->CMayVision->bShowColorImage = false;
		}
	}

	// if (CMay->CMayVision->bEnableColorImageLeft_Zed)
	// {
	// 	if (CMay->CMayVision->CZed->grab() == sl::SUCCESS) {
	// 		// Retrieve left image
	// 		sl::Mat zed_image;
	// 		CMay->CMayVision->CZed->retrieveImage(zed_image, sl::VIEW_LEFT);
	// 		cv::imshow("VIEW1", cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
	// 		cv::waitKey(5);
	// 	}
	// }
	// if (CMay->CMayVision->bEnableColorImageRight_Zed)
	// {
	// 	if (CMay->CMayVision->CZed->grab() == sl::SUCCESS) {
	// 		// Retrieve left image
	// 		sl::Mat zed_image;
	// 		CMay->CMayVision->CZed->retrieveImage(zed_image, sl::VIEW_RIGHT);
	// 		cv::imshow("VIEW2", cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
	// 		cv::waitKey(5);
	// 	}
	// }
	// if (CMay->CMayVision->bEnableDepthImageLeft_Zed)
	// {
	// 	if (CMay->CMayVision->CZed->grab() == sl::SUCCESS) {
	// 		// Retrieve left image
	// 		sl::Mat zed_image;
	// 		sl::Mat depth_map;
	// 		CMay->CMayVision->CZed->retrieveImage(zed_image, sl::VIEW_DEPTH);
	// 		CMay->CMayVision->CZed->retrieveMeasure(depth_map, sl::MEASURE_DEPTH); // Retrieve depth
	// 		cv::imshow("VIEW3", cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
	// 		cv::setMouseCallback("VIEW3", onMouse_ColorImage, NULL);
	// 		cv::waitKey(5);
	// 	}
	// }
	// if (CMay->CMayVision->bEnableDepthImageRight_Zed)
	// {
	// 	if (CMay->CMayVision->CZed->grab() == sl::SUCCESS) {
	// 		// Retrieve left image
	// 		sl::Mat zed_image;
	// 		sl::Mat depth_map;
	// 		CMay->CMayVision->CZed->retrieveImage(zed_image, sl::VIEW_DEPTH);
	// 		CMay->CMayVision->CZed->retrieveMeasure(depth_map, sl::MEASURE_DEPTH); // Retrieve depth
	// 		cv::imshow("VIEW4", cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
	// 		cv::waitKey(5);
	// 	}
	// }

	if (bColorImageWindowCreated && !CMay->CMayVision->bEnableColorImage) //�????�?destroyWindow
	{
		bColorImageWindowCreated = false;
		destroyWindow("Color_Image");
		cv::waitKey(1);
	}

	if (bDepthImageWindowCreated && !CMay->CMayVision->bEnableDepthImage)
	{
		bDepthImageWindowCreated = false;
		cv::destroyWindow("Depth Image");
		cv::waitKey(1);
	}

	if (CMay->CMayVision->bUpdateDepthImage && CMay->CMayVision->matDepthImageProcessed.empty() == false)
	{
		if (!this->bDepthImageWindowCreated)
			this->create_depth_image_window();

		cv::imshow("Depth Image", CMay->CMayVision->matDepthImageProcessed);
		cv::waitKey(1);

		CMay->CMayVision->bUpdateDepthImage = false;
	}
	//else
	//{
	//    cv::destroyWindow("Depth Image");
	//    this->bDepthImageWindowCreated = false;
	//}
	// Zed Image Input ==============================================================================================
	if (CMay->CMayVision->bEnableColorImageLeft_Zed || CMay->CMayVision->bEnableDepthImageLeft_Zed)
	{
		// if(this->bInitialColorImageLeft_Zed && CMay->CMayVision->bEnableColorImageLeft_Zed)
		// {
		// 	this->bInitialColorImageLeft_Zed = false;
		// 	CMay->CMayVision->StartSubFromCamera();
		// 	ros::AsyncSpinner spinner(2); // Use 2 threads
		// 	spinner.start();
		// 	this_thread::sleep_for(chrono::milliseconds(500));
		// }
		// // Image Processing-------------------------------------------------------------------------------------------
		while (!CMay->CMayVision->CImageConverter->image_sub_flag)
		{
			this_thread::sleep_for(chrono::milliseconds(100));
		}
		CMay->CMayVision->CImageConverter->GetImgForProcessing(0);
		CMay->CMayVision->CImageConverter->GetImgForProcessing(1);
		cv::Mat ZedColorImage = CMay->CMayVision->CImageConverter->img_for_processing.clone();
		cv::Mat ZedDepthImage = CMay->CMayVision->CImageConverter->depth_for_processing.clone();

		// HSV ---------------------
		// Color Filter
		if (CMay->CMayVision->bEnableColorImageLeft_Zed)
		{
			CMay->CMayVision->CImageProcessing->ColorFilter(ZedColorImage, this->nHmin, this->nHmax, this->nSmin, this->nSmax, this->nVmin, this->nVmax);
		}

		//ROI(Depth Included) ----------------------
		//sl::Mat point_cloud;
		//CMay->CMayVision->CZed->retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA);
		/*sl::float4 point3D;*/
		// Get the 3D point cloud values for pixel (i,j)
		//Sleep(100);
		//point_cloud.getValue(x_ColorImageMouseNow, y_ColorImageMouseNow, &point3D); //�@�ɮy��
		//Sleep(100);

		if (this->ui->CB_ROI_Enable->isChecked())
		{
			this->nXmin = cvRound(this->ui->Slider_Xmin->value());
			this->nXmax = cvRound(this->ui->Slider_Xmax->value());
			this->nYmin = cvRound(this->ui->Slider_Ymin->value());
			this->nYmax = cvRound(this->ui->Slider_Ymax->value());
			this->nDmin = cvRound(this->ui->Slider_Dmin->value());
			this->nDmax = cvRound(this->ui->Slider_Dmax->value());
			CMay->CMayVision->CImageProcessing->RoI(ZedColorImage, this->nXmin, this->nXmax, this->nYmin, this->nYmax);
		}
		if (CMay->CMayVision->bEnableShowBoundingBox_Zed)
		{
			float ptx[8], pty[8], ptz[8];
			ptx[0] = this->ui->LE_Point1_X->text().toDouble();
			pty[0] = this->ui->LE_Point1_Y->text().toDouble();
			ptz[0] = this->ui->LE_Point1_Z->text().toDouble();

			ptx[1] = this->ui->LE_Point2_X->text().toDouble();
			pty[1] = this->ui->LE_Point2_Y->text().toDouble();
			ptz[1] = this->ui->LE_Point2_Z->text().toDouble();

			ptx[2] = this->ui->LE_Point3_X->text().toDouble();
			pty[2] = this->ui->LE_Point3_Y->text().toDouble();
			ptz[2] = this->ui->LE_Point3_Z->text().toDouble();

			ptx[3] = this->ui->LE_Point4_X->text().toDouble();
			pty[3] = this->ui->LE_Point4_Y->text().toDouble();
			ptz[3] = this->ui->LE_Point4_Z->text().toDouble();

			ptx[4] = this->ui->LE_Point5_X->text().toDouble();
			pty[4] = this->ui->LE_Point5_Y->text().toDouble();
			ptz[4] = this->ui->LE_Point5_Z->text().toDouble();

			ptx[5] = this->ui->LE_Point6_X->text().toDouble();
			pty[5] = this->ui->LE_Point6_Y->text().toDouble();
			ptz[5] = this->ui->LE_Point6_Z->text().toDouble();

			ptx[6] = this->ui->LE_Point7_X->text().toDouble();
			pty[6] = this->ui->LE_Point7_Y->text().toDouble();
			ptz[6] = this->ui->LE_Point7_Z->text().toDouble();

			ptx[7] = this->ui->LE_Point8_X->text().toDouble();
			pty[7] = this->ui->LE_Point8_Y->text().toDouble();
			ptz[7] = this->ui->LE_Point8_Z->text().toDouble();

			float ptu[8];
			float ptv[8];
			for (int ptr_num = 0; ptr_num < 8; ptr_num++)
			{
				float fx = 1401.46;   //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.fx;
				float fy = 1401.46;   //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.fy;
				float cx = 1046.08;   //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.cx;
				float cy = 636.451;   //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.cy;
				float k1 = -0.174071; //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[0];
				float k2 = 0.027556;  //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[1];
				float p1 = 0;		  //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[2];
				float p2 = 0;		  //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[3];
				float p3 = 0;		  //CMay->CMayVision->CZed->getCameraInformation().calibration_parameters.left_cam.disto[4];
				float x_ = -pty[ptr_num] / ptx[ptr_num];
				float y_ = -ptz[ptr_num] / ptx[ptr_num];
				float r2 = x_ * x_ + y_ * y_;
				float x__ = x_ * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p1 * x_ * y_ + p2 * (r2 + 2 * x_ * x_);
				float y__ = y_ * (1 + k1 * r2 + k2 * r2 * r2) + p1 * (r2 + 2 * y_ * y_) + 2 * p2 * x_ * y_;
				float u = fx * x__ + cx;
				float v = fy * y__ + cy;
				cout << "u " << u << " v " << v << endl;
				cv::circle(ZedColorImage, Point(u, v), 1, Scalar(0, 255, 0), -1);
				ptu[ptr_num] = u;
				ptv[ptr_num] = v;
				//cv::line(ZedColorImage, );
			}
			cv::line(ZedColorImage, Point(ptu[0], ptv[0]), Point(ptu[1], ptv[1]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[2], ptv[2]), Point(ptu[3], ptv[3]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[4], ptv[4]), Point(ptu[5], ptv[5]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[6], ptv[6]), Point(ptu[7], ptv[7]), Scalar(255, 0, 0), 3);

			cv::line(ZedColorImage, Point(ptu[0], ptv[0]), Point(ptu[3], ptv[3]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[1], ptv[1]), Point(ptu[2], ptv[2]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[4], ptv[4]), Point(ptu[7], ptv[7]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[5], ptv[5]), Point(ptu[6], ptv[6]), Scalar(255, 0, 0), 3);

			cv::line(ZedColorImage, Point(ptu[0], ptv[0]), Point(ptu[4], ptv[4]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[1], ptv[1]), Point(ptu[5], ptv[5]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[2], ptv[2]), Point(ptu[6], ptv[6]), Scalar(255, 0, 0), 3);
			cv::line(ZedColorImage, Point(ptu[3], ptv[3]), Point(ptu[7], ptv[7]), Scalar(255, 0, 0), 3);
		}
		// Show Picture------------------------------------------------------------------------------------------------

		if (CMay->CMayVision->bEnableColorImageLeft_Zed)
		{
			if (!this->bZedColorImageWindowCreated)
				this->create_zed_color_image_window();

			cv::imshow("Zed Color Image", ZedColorImage);
			cv::waitKey(1);
			//cv::imshow("Zed Color Img", cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
		}
		if (CMay->CMayVision->bEnableDepthImageLeft_Zed)
		{
			if (!this->bZedDepthImageWindowCreated)
				this->create_zed_depth_image_window();

			cv::imshow("Zed Depth Image", ZedDepthImage);
			cv::waitKey(1);
		}
	}
	CMay->CMayVision->EndSomething();
}

void Form_Vision::check_form(void)
{
	// Color_image Kinect
	// if (ui->checkBox_EnableColorImage->isChecked())
	// 	CMay->CMayVision->bEnableColorImage = true;

	// // Depth image Kinect
	// if (ui->checkBox_EnableDepthImage->isChecked())
	// 	CMay->CMayVision->bEnableDepthImage = true;

	// Color_image Zed Left
	if (ui->checkBox_EnableColorImageLeft_Zed->isChecked())
		CMay->CMayVision->bEnableColorImageLeft_Zed = true;

	//Depth image Zed
	if (ui->checkBox_EnableDepthImageLeft_Zed->isChecked())
		CMay->CMayVision->bEnableDepthImageLeft_Zed = true;

	//Color_image Zed Left
	if (ui->checkBox_EnableColorImageRight_Zed->isChecked())
		CMay->CMayVision->bEnableColorImageRight_Zed = true;

	//Depth image Zed
	if (ui->checkBox_EnableDepthImageRight_Zed->isChecked())
		CMay->CMayVision->bEnableDepthImageRight_Zed = true;

	// To Be Fixed
	//// Compressive tracking
	//if (this->checkBox_CompressiveTracking->Checked)
	//	CMay->CMayVision->bEnableCompressiveTracking = true;

	//// CMT
	//if (this->checkBox_CMT->Checked)
	//	CMay->CMayVision->bEnableCMT = true;

	//// Object recognition
	//if (this->checkBox_ObjectRecognition->Checked)
	//	CMay->CMayVision->bEnableObjectRecognizing = true;

	//// Database make
	//if (this->checkBox_NewObj->Checked)
	//	CMay->CMayVision->bEnableDatabaseObjectMaking = true;

	// Color Filter
	if (ui->checkBox_ColorFilter->isChecked())
		CMay->CMayVision->bEnableColorFilter = true;

	////Face detection
	//if (this->checkBox_FaceDetection->Checked)
	//	CMay->CMayVision->bEnableFaceDetecting = true;

	this->nHmin = cvRound(this->ui->hScrollBar_Hmin->value());
	this->nHmax = cvRound(this->ui->hScrollBar_Hmax->value());
	this->nSmin = cvRound(this->ui->hScrollBar_Smin->value());
	this->nSmax = cvRound(this->ui->hScrollBar_Smax->value());
	this->nVmin = cvRound(this->ui->hScrollBar_Vmin->value());
	this->nVmax = cvRound(this->ui->hScrollBar_Vmax->value());

	//// Blur Image
	if (this->ui->checkBox_BlurImage->isChecked())
		CMay->CMayVision->bEnableBlurImage = true;

	this->nBinaryThreshold = this->ui->hScrollBar_BinaryImage->value();

	// Binary Image
	if (this->ui->checkBox_BinaryImage->isChecked())
		CMay->CMayVision->bEnableBinaryImage = true;

	//// Find Contour
	//if (this->radioButton_AllContours->Checked || this->radioButton_MaxContour->Checked)
	//	CMay->CMayVision->bEnableFindContours = true;

	//if (this->radioButton_AllContours->Checked || this->radioButton_MaxContour->Checked)
	//	CMay->CMayVision->bEnableFindContoursPoint = true;

	//if (this->radioButton_AllContours->Checked)
	//	enumContourCase = ContourCase::AllContours;
	//else if (this->radioButton_MaxContour->Checked)
	//	enumContourCase = ContourCase::MaxContour;

	// Morphology
	if (this->ui->radioButton_Erode->isChecked() || this->ui->radioButton_Dilate->isChecked() ||
		this->ui->radioButton_OpeningOperation->isChecked() || this->ui->radioButton_ClosingOperation->isChecked())
		CMay->CMayVision->bEnableMorphology = true;

	if (this->ui->radioButton_Erode->isChecked())
		enumMorphologyState = MorphologyState::Eroding;
	else if (this->ui->radioButton_Dilate->isChecked())
		enumMorphologyState = MorphologyState::Dilating;
	else if (this->ui->radioButton_OpeningOperation->isChecked())
		enumMorphologyState = MorphologyState::Opening;
	else if (this->ui->radioButton_ClosingOperation->isChecked())
		enumMorphologyState = MorphologyState::Closing;

	// Find Circles
	if (this->ui->checkBox_FindCircles->isChecked())
		CMay->CMayVision->bEnableFindCircles = true;

	//// Skeleton
	//if (this->checkBox_Enable_Skeleton->Checked)
	//	CMay->CMayVision->bEnableSkeleton = true;

	//// ROI
	//if (this->checkBox_CB_EnableROI->Checked)
	//{
	//	this->nXmin = System::Convert::ToInt32(textBox_CB_Xmin->Text);
	//	this->nXmax = System::Convert::ToInt32(textBox_CB_Xmax->Text);
	//	this->nYmin = System::Convert::ToInt32(textBox_CB_Ymin->Text);
	//	this->nYmax = System::Convert::ToInt32(textBox_CB_Ymax->Text);
	//}
	////�e�u
	//if (this->checkBox_CB_EnableLine->Checked)
	//{
	//	this->nLine_BallandOther = System::Convert::ToInt32(textBox_CB_LineBallandOther->Text);
	//	this->nLine_ObjandHole = System::Convert::ToInt32(textBox_CB_LineObjandHole->Text);

	//}
	////Height_Set
	//if (this->checkBox_CB_EnableROI->Checked)
	//{
	//	this->nHeight_Threshold = System::Convert::ToInt32(textBox_CB_Height_Threshold->Text);
	//	this->nHeight_Threshold_mask = System::Convert::ToInt32(textBox_CB_Height_Threshold_mask->Text);

	//}
	//--------------------------------------------------------------------------------------------------------------------
	//if (CMay->CMayVision->bUpdateDepthImage)
	//{
	//    // 如果不刪除bitmap會造成記憶體不足，刪除不慎也會造成程式錯誤
	//    delete this->bmpDepthImage;
	//    Mat mat = CMay->CMayVision->matDepthImageProcessed.clone();

	//    System::Drawing::Bitmap^ bmpTmp = gcnew Bitmap(
	//        CMay->CMayVision->matDepthImageProcessed.cols,
	//        CMay->CMayVision->matDepthImageProcessed.rows,
	//        CMay->CMayVision->matDepthImageProcessed.step,
	//        Imaging::PixelFormat::Format24bppRgb,
	//        (IntPtr)mat.ptr());

	//    this->bmpDepthImage = gcnew Bitmap(bmpTmp);

	//    this->pictureBox_DepthImage->Image = this->bmpDepthImage;

	//    delete bmpTmp;

	//    CMay->CMayVision->bUpdateDepthImage = false;
	//}
}

void Form_Vision::show_mouse_information(void)
{
	if (x_ColorImageMouseNow > 0 && y_ColorImageMouseNow > 0)
	{
		//sl::Mat zed_image;
		//CMay->CMayVision->CZed->retrieveImage(zed_image, sl::VIEW_DEPTH);
		////CMay->CMayVision->CZed->retrieveMeasure(zed_image, sl::MEASURE_DEPTH); // Retrieve depth
		// float depth_value = 0;
		// zed_image.getValue(x_ColorImageMouseNow, y_ColorImageMouseNow, &depth_value);
		// cout << depth_value << endl;
		// To Be Fixed
		// 顯示游標資訊
		//System::String^ strImageXY; // image x y
		//System::String^ strKinectXYZ; // kinect x y z
		//System::String^ strHeadXYZ; // Head x y z
		//System::String^ strLeftArmXYZ; // LeftArm x y z
		//System::String^ strLeftShoulderXYZ; // LeftArm x y z
		//System::String^ strLeftArmXYZR; // LeftArm x y z
		//System::String^ strLeftShoulderXYZR; // LeftArm x y z

		// float xWorld;
		// float yWorld;
		// float zWorld;

		//float fDepth = 0;

		//CameraSpacePoint* CSP = new CameraSpacePoint;
		////0.02及0.01為shift量
		//if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X))
		//	xWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].X;
		//else
		//	xWorld = -1;
		//if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y))
		//	yWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Y;
		//else
		//	yWorld = -1;
		//if (_finite(CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z))
		//	zWorld = CMay->CMayVision->CMayKinect->CSP_From_Color[x_ColorImageMouseNow + Width_Color_Frame * y_ColorImageMouseNow].Z;
		//else
		//	zWorld = -1;

		//CSP->X = xWorld;
		//CSP->Y = yWorld;
		//CSP->Z = zWorld;

		//strImageXY = gcnew System::String("(" + x_ColorImageMouseNow + ", " + y_ColorImageMouseNow + ")");
		//strKinectXYZ = gcnew System::String("KINECT\n" + xWorld + "\n" + yWorld + "\n" + zWorld);

		// float hx, hy, hz;

		// CMay->CMayBody->CMayHeadandLiftingPlatform->HeadCoordinate(xWorld, yWorld, zWorld, hx, hy, hz);

		// //float* arm_solR_ShoulderToArm = CMay->CMayBody->CMayRightArm->CenterToShoulder(hx, hy, hz);    //???may???測�??覺�????????
		// float *arm_solL_ShoulderToArm = CMay->CMayBody->CMayLeftArm->CenterToShoulder(hx, hy, hz); //???may???測�??覺�????????
		// //float* arm_solR = CMay->CMayBody->CMayRightArm->ShoulderToArm(arm_solR_ShoulderToArm[0], arm_solR_ShoulderToArm[1], arm_solR_ShoulderToArm[2]);    //???may???測�??覺�????????
		// float *arm_sol = CMay->CMayBody->CMayLeftArm->ShoulderToArm(arm_solL_ShoulderToArm[0], arm_solL_ShoulderToArm[1], arm_solL_ShoulderToArm[2]); //???may???測�??覺�????????

		// //																																				 //Point3d ptKinect(xWorld, yWorld, zWorld);
		//																																				 //Point3d arm_sol = CMay->CMayVision->get_arm_point_from_Kinect(ptKinect);

		//strHeadXYZ = gcnew System::String("HEAD\n" + hx + "\n" + hy + "\n" + hz);
		//strLeftShoulderXYZ = gcnew System::String("Left Shoulder\n" + arm_solL_ShoulderToArm[0] + "\n" + arm_solL_ShoulderToArm[1] + "\n" + arm_solL_ShoulderToArm[2]);
		//strLeftShoulderXYZR = gcnew System::String("Right Shoulder\n" + arm_solR_ShoulderToArm[0] + "\n" + arm_solR_ShoulderToArm[1] + "\n" + arm_solR_ShoulderToArm[2]);
		//strLeftArmXYZ = gcnew System::String("LEFT ARM\n" + arm_sol[0] + "\n" + arm_sol[1] + "\n" + arm_sol[2]);
		//strLeftArmXYZR = gcnew System::String("RIGHT ARM\n" + arm_solR[0] + "\n" + arm_solR[1] + "\n" + arm_solR[2]);
		////strLeftArmXYZ = gcnew System::String("LEFT ARM\n" + arm_sol.x + "\n" + arm_sol.y + "\n" + arm_sol.z);
		//this->label_Coordinate_Image_Value->Text = strImageXY;
		//this->label_Coordinate_World_Value->Text = strKinectXYZ;
		//this->label_headCoordinate->Text = strHeadXYZ;
		//this->label_Left_Shoulder_pose->Text = strLeftShoulderXYZ;
		//this->label_Right_Shoulder_pose->Text = strLeftShoulderXYZR;
		//this->label_ArmPos->Text = strLeftArmXYZ;
		//this->label_ArmPosR->Text = strLeftArmXYZR;

		//delete strImageXY;
		//delete strKinectXYZ;
		//delete strHeadXYZ;
		//delete strLeftArmXYZ;
		//delete strLeftShoulderXYZ;
		//delete strLeftShoulderXYZR;
		//delete[] arm_sol;
		//delete CSP;

		CMay->CMayVision->CPclConverter->GetCloudForProcessing();
		CMay->CMayVision->CImageConverter->GetImgForProcessing(1);
		


		cv::Point tmp_pt;

		tmp_pt.x = x_ColorImageMouseNow;
		tmp_pt.y = y_ColorImageMouseNow;

		float dep=CMay->CMayVision->CImageConverter->depth_for_processing.at<float>(tmp_pt);
		cout<<"depth: "<<dep<<endl;




		int r;
		int g;
		int b;
		r = CMay->CMayVision->CImageConverter->img_for_processing.at<cv::Vec3b>(tmp_pt)[2];
		g = CMay->CMayVision->CImageConverter->img_for_processing.at<cv::Vec3b>(tmp_pt)[1];
		b = CMay->CMayVision->CImageConverter->img_for_processing.at<cv::Vec3b>(tmp_pt)[0];

		int point_cloud_index;
		// point_cloud_index = tmp.y * IMG_WIDTH + tmp.x;
		point_cloud_index = tmp_pt.y * CMay->CMayVision->CImageConverter->img_for_processing.size().width + tmp_pt.x;
		cv::Point3f pc_coord(0, 0, 0);
		cv::Point3i pc_color(0, 0, 0);
		pc_coord.x = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].x;
		pc_coord.y = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].y;
		pc_coord.z = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].z;
		pc_color.x = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].r;
		pc_color.y = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].g;
		pc_color.z = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].b;

		cout << "pc_coord: " << pc_coord.x << "," << pc_coord.y << "," << pc_coord.z << endl;
		cout << "pc_color: " << pc_color.x << "," << pc_color.y << "," << pc_color.z << endl;

		float hx;
		float hy;
		float hz;

		CMay->CMayBody->CMayHeadandLiftingPlatform->HeadCoordinate(pc_coord.y, pc_coord.z, pc_coord.x, hx, hy, hz);
		float *arm_solL_ShoulderToArm = CMay->CMayBody->CMayLeftArm->CenterToShoulder(hx, hy, hz);
		float *arm_sol = CMay->CMayBody->CMayLeftArm->ShoulderToArm(arm_solL_ShoulderToArm[0], arm_solL_ShoulderToArm[1], arm_solL_ShoulderToArm[2]);

		float *arm_solR_ShoulderToArm = CMay->CMayBody->CMayRightArm->CenterToShoulder(hx, hy, hz);
		float *arm_solR = CMay->CMayBody->CMayRightArm->ShoulderToArm(arm_solR_ShoulderToArm[0], arm_solR_ShoulderToArm[1], arm_solR_ShoulderToArm[2]);
		// cout << "center_coord: " << hx << "," << hy << "," << hz << endl;
		// cout << endl;
		// cout << "leftshoulder_coord: " << arm_solL_ShoulderToArm[0] << "," << arm_solL_ShoulderToArm[1] << "," << arm_solL_ShoulderToArm[2];
		// cout << endl;
		// cout << "leftatm_coord:" << arm_sol[0] << "," << arm_sol[1] << "," << arm_sol[2] << endl;

		this->ui->label_coord_head->setText(QString::number(hx) + "\n" + QString::number(hy) + "\n" + QString::number(hz));
		this->ui->label_coord_left_s->setText(QString::number(arm_solL_ShoulderToArm[0] - 23) + "\n" + QString::number(arm_solL_ShoulderToArm[1] - 37) + "\n" + QString::number(arm_solL_ShoulderToArm[2] - 80));
		this->ui->label_coord_left_h->setText(QString::number(arm_sol[0] - 23) + "\n" + QString::number(arm_sol[1] - 37) + "\n" + QString::number(arm_sol[2] - 80));
		this->ui->label_coord_right_s->setText(QString::number(arm_solR_ShoulderToArm[0]) + "\n" + QString::number(arm_solR_ShoulderToArm[1]) + "\n" + QString::number(arm_solR_ShoulderToArm[2]));
		this->ui->label_coord_right_h->setText(QString::number(arm_solR[0]) + "\n" + QString::number(arm_solR[1]) + "\n" + QString::number(arm_solR[2]));
		this->ui->label_image_posit->setText(QString::number(tmp_pt.x) + "\n" + QString::number(tmp_pt.y));

		cout << " ColorImageMouseNow " << x_ColorImageMouseNow << "," << y_ColorImageMouseNow << endl;

		delete[] arm_sol;
		delete[] arm_solL_ShoulderToArm;
		delete[] arm_solR;
		delete[] arm_solR_ShoulderToArm;
	}
}

//---------------以下為之前的BackGroundWorker ---用C++11 的Threadru library 解決

void Form_Vision::thread_Vision()
{
	cout << "Entering thread vision" << endl;
	while (bThreadVisionRun)
	{
		this->check_form();
		this->show_mouse_information();
		this->run_vision_thread();
		this_thread::sleep_for(chrono::milliseconds(1000));
	}
	cout << "thread_Vision Out " << endl;
}

void Form_Vision::on_QRcode_decoder_clicked()
{
	// fstream data;
	// std::string line;

	CMay->CMayVision->StartQRcode();

	// data.open("/home/bob/catkin_ws/src/may/src/Vision/QRcode/QRcode.txt");

	// while(data.peek()!=EOF)
	// {
	// 	getline(data,line);
    //    QRdata = std::stof(line);
	// }
}
