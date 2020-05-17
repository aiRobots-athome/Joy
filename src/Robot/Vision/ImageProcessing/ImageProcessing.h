#pragma once
#include <opencv2/opencv.hpp>

#define Width_Color_Frame 1920
#define Height_Color_Frame 1080
#define Width_Processing 640
#define Height_Processing 360

typedef unsigned char uchar;

using namespace std;

class ImageProcessing
{
public:
    class ContourTool
    {
    public:
        int nMinContourArea;
		float ApproximatecontoursValue = 8;   //0.03
        vector<vector<cv::Point>> vec2_ptContours;
        vector<cv::Point> vec_ptCenters;

        void FindContours(const cv::Mat &src, int nMinArea = 10);
        void FindMaxContours(const cv::Mat &src);
        void FindContoursCenters(void);
        void RefineToApproximateContours(void);
    };

    class Morphology
    {
    public:
        void DilateImage(cv::Mat &src, int nMaskSize, int nTimes); // ����
        void ErodeImage(cv::Mat &src, int nMaskSize, int nTimes); // �I�k
        void OpeningOperation(cv::Mat &src, int nMaskSize, int nTimes); // �}�B��
        void ClosingOperation(cv::Mat &src, int nMaskSize, int nTimes); // ���B��
    };

    class Circle
    {
    public:
        vector<cv::Vec3f> vec_vec3fCircles; // vec_vec3fCircles[i][0] = x, vec_vec3fCircles[i][1] = y, vec_vec3fCircles[i][2] = radius 
        int nCenterDetectionThreshold; // How perfect the circle is, default value is 100
        void FindCircles(cv::Mat &src); 
    };
    
    ImageProcessing();
    ~ImageProcessing();
    
    ContourTool *CContourTool;
    Morphology *CMorphologyTool;
    Circle *CCircleTool;

    int nBlurMaskSize;
	

    void ColorFilter(cv::Mat &src, int H_min, int H_max, int S_min, int S_max, int V_min, int V_max); //�L�o�C��
    void ColorFilter(cv::Mat &src, std::string strFileName); //�L�o�C��AŪ���ɮ�
    void BlurImage(cv::Mat &src); // �ҽk��
    void BinaryImage(cv::Mat &src, int nThreshold); // �G�Ȥ�

	void RoI(cv::Mat &src, int X_min, int X_max, int Y_min, int Y_max ); //�]�wROI
	

    //vector<cv::Point> FindMaxContour(void); // ��̤j����
};

