#ifndef FORM_VISION_H
#define FORM_VISION_H
#include "Gui/ui_Form_Vision.h"
#include <QtWidgets/QDialog>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtCore/QMap>

struct surface_info
{
	float x = 0;
	float y = 0;
	float z = 0;
	float nx = 0;
	float ny = 0;
	float nz = 0;
	//bbox_processing pic_info;
};
struct img2world
{
	int pix_x;
	int pix_y;
	float x;
	float y;
	float z;
	int r;
	int g;
	int b;
};
class Form_Vision : public QDialog
{
    Q_OBJECT

public:
    explicit Form_Vision(QWidget *parent = nullptr);
    ~Form_Vision();

	bool bThreadVisionRun = false;
	void thread_Vision();
	std::thread backgroundWorker_Vision;

protected:
	void showEvent(QShowEvent *ev);

private slots:
	void on_checkBox_BlurImage_stateChanged(int arg1);
	void on_checkBox_BinaryImage_stateChanged(int arg1);
	void on_checkBox_FindCircles_stateChanged(int arg1);
	void on_trackBar_BlurMaskSize_valueChanged();
	void on_radioButton_Morphology_No_toggled();
	void on_button_WbCm_Str_clicked();
	void on_button_WbCm_Stop_clicked();
	void on_button_Save_ColorFilter_clicked();
	void on_button_Load_ColorFilter_clicked();
	void on_hScrollBar_Hmin_valueChanged();
	void on_hScrollBar_Hmax_valueChanged();
	void on_hScrollBar_Smin_valueChanged();
	void on_hScrollBar_Smax_valueChanged();
	void on_hScrollBar_Vmin_valueChanged();
	void on_hScrollBar_Vmax_valueChanged();
	void on_radioButton_Dilate_toggled();
	void on_radioButton_Erode_toggled();
	void on_radioButton_OpeningOperation_toggled();
	void on_radioButton_ClosingOperation_toggled();
	void on_radioButton_NoContour_toggled();
	void on_radioButton_MaxContour_toggled();
	void on_radioButton_AllContours_toggled();
	void on_comboBox_MorphologyMaskSize_activated(int);
	void on_comboBox_MorphologyTimes_activated(int arg);
	void on_textBox_MinContourArea_textChanged(const QString &arg1);
	void on_textBox_RefineValue_textChanged(const QString &arg1);
	void on_textBox_FindCirclesThreshold_textChanged(const QString &arg1);
	void on_checkBox_EnableColorImageLeft_Zed_stateChanged(int arg1);
	void on_checkBox_EnableDepthImageLeft_Zed_stateChanged(int arg1);
	void on_checkBox_EnableColorImageRight_Zed_stateChanged(int arg1);
	void on_checkBox_EnableDepthImageRight_Zed_stateChanged(int arg1);
	void on_Slider_Dmin_valueChanged();
	void on_Slider_Dmax_valueChanged();
	void on_Slider_Xmin_valueChanged();
	void on_Slider_Xmax_valueChanged();
	void on_Slider_Ymin_valueChanged();
	void on_Slider_Ymax_valueChanged();
	void on_PB_Save_ROI_clicked();
	void on_PB_Load_ROI_clicked();

	void on_PB_Publish_clicked();
	void on_PB_BoundingBoxTest_clicked();
	void on_CB_EnableBoundingBox_stateChanged();	
	void on_PB_OrientationRecognition_clicked();
	void on_QRcode_decoder_clicked();
	// void on_PB_GetPointCloudFromZed_clicked();
	// void on_PB_LoadPointCloud_clicked();
	// void on_PB_ShowPointCloud_clicked();
	// void on_CB_ShowPointCloud_stateChanged();
	// void on_PB_ClearPointCloud_clicked();
	// void on_PB_SavePointCloud_clicked();
	// void on_PB_LoadYolo_clicked();
	// void on_PB_YoloClean_clicked();
	// void on_PB_YoloDetect_clicked();
	// void on_CB_YoloFaceNormalVector_stateChanged(int arg1);
	// void on_PB_LoadModel_clicked();
	// void on_PB_ClearModel_clicked();
	// void on_PB_ShowModelPC_clicked();
	// void on_PB_ShowModelVoxel_clicked();
	// void on_PB_MoveModel_clicked();
	// void on_PB_ShowModelDownSampling_clicked();
	// void on_PB_LoadModelPCD_clicked();
	// void on_LE_ModelDownSamplingSize_textChanged();
	// void on_VS_ModelRotate_a_valueChanged();
	// void on_VS_ModelRotate_b_valueChanged();
	// void on_VS_ModelRotate_c_valueChanged();
	// void on_PB_SaveModelTrainData_clicked();
	// void on_PB_ShowSegmentation_clicked();
	// void on_PB_3DCNNDetect_clicked();
	// void on_PB_SaveModel_clicked();
	// void on_PB_MidLengthFilter_clicked();
	// void on_PB_Trans2_TestData_clicked();
	// void on_PB_ShowTestTxt_clicked();



private:
    Ui::Form_Vision *ui;
	void run_vision_thread(void); // �D�n�b�I�����檺�\��
//	bool check_if_point_exists(cv::Point ptNewPoint);
	void check_form(void); // �ˬdform������
	void create_color_image_window(void); // �s�رm��v������
	void create_depth_image_window(void); // �s�ز`�׼v������
	void create_zed_color_image_window(void); // �s�رm��v������
	void create_zed_depth_image_window(void); // �s�ز`�׼v������
	void show_mouse_information(void); // �btimer����s��и�T
#pragma region MY_PARAMETERS
	bool bColorImageWindowCreated = false;
	bool bDepthImageWindowCreated = false;
	bool bZedColorImageWindowCreated = false;
	bool bZedDepthImageWindowCreated = false;
	int nXmax, nXmin, nYmax, nYmin;  // ROI
	float nDmax, nDmin;  
	int nLine_BallandOther, nLine_ObjandHole;  //Line
	int nHeight_Threshold, nHeight_Threshold_mask;  //Height
	int nHmin, nHmax, nSmin, nSmax, nVmin, nVmax; // Color Filter HSV
	int nBlurMask, nBinaryThreshold;
	void InitialParameters(void);
	bool EnableCalculateNormalVector;
	bool ShowPictureBoxCenterNormal=0;
	// <x> <y> <z> <nx> <ny> <nz> <bbox_processing>
	surface_info *Obj_Coordinate_Info = new surface_info [7];
	vector<img2world> *Rec2Dto3D = new vector<img2world>;
	bool Surface_Detected[7] = {};
	float ctr_len = 0;
	bool bInitialColorImageLeft_Zed = true;

};

#endif // FORM_VISION_H
