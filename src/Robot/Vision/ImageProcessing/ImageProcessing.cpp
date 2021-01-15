#include "ImageProcessing.h"

ImageProcessing::ImageProcessing()
{
    this->CContourTool = new ContourTool();
    this->CMorphologyTool = new Morphology();
    this->CCircleTool = new Circle();

    this->nBlurMaskSize = 1;
    this->CContourTool->nMinContourArea = 10;
    this->CCircleTool->nCenterDetectionThreshold = 100;
}


ImageProcessing::~ImageProcessing()
{
    delete this->CContourTool;
    delete this->CMorphologyTool;
    delete this->CCircleTool;
}

void ImageProcessing::ColorFilter(cv::Mat &src, int H_min, int H_max, int S_min, int S_max, int V_min, int V_max)
{
    /*
    In Opencv: H: 0~180, S: 0~255, V: 0~255
    */
    cv::Mat matHSV, matMaskedImage;
    cv::Mat matMask = cv::Mat::zeros(src.rows, src.cols, CV_8U);
    cv::Mat matMask1 = cv::Mat::zeros(src.rows, src.cols, CV_8U);;
    cv::Mat matMask2 = cv::Mat::zeros(src.rows, src.cols, CV_8U);;

    cv::cvtColor(src, matHSV, cv::COLOR_BGR2HSV);

    // Ensure the HSV input values are in correct range
    H_min = H_min > 180 ? 180 : H_min;
    H_min = H_min < 0 ? 0 : H_min;
    H_max = H_max > 180 ? 180 : H_max;
    H_max = H_max < 0 ? 0 : H_max;
    S_min = S_min > 255 ? 255 : S_min;
    S_min = S_min < 0 ? 0 : S_min;
    S_max = S_max > 255 ? 255 : S_max;
    S_max = S_max < 0 ? 0 : S_max;
    V_min = V_min > 255 ? 255 : V_min;
    V_min = V_min < 0 ? 0 : V_min;
    V_max = V_max > 255 ? 255 : V_max;
    V_max = V_max < 0 ? 0 : V_max;

    // Red is between about 160~20
    if (H_min < H_max)
    {
        cv::inRange(matHSV, cv::Scalar(H_min, V_min, S_min), cv::Scalar(H_max, V_max, S_max), matMask);
    }
    else if (H_min > H_max)
    {
        cv::inRange(matHSV, cv::Scalar(H_min, V_min, S_min), cv::Scalar(180, V_max, S_max), matMask1);
        cv::inRange(matHSV, cv::Scalar(0, V_min, S_min), cv::Scalar(H_max, V_max, S_max), matMask2);
        matMask = matMask1 + matMask2;
    }

    src.copyTo(matMaskedImage, matMask);
    src = matMaskedImage;

    matMask.release();
    matMask1.release();
    matMask2.release();

}

void ImageProcessing::ColorFilter(cv::Mat &src, std::string strFileName)
{
	//Fix requirement 20181003
 /*   System::String^ strFile = gcnew System::String(strFileName.c_str());
    System::String^ strFileFull = ".\\Database\\ColorFilter\\" + strFile + "\.txt";

    if (!System::IO::File::Exists(strFileFull))
        System::Console::WriteLine("File doesn't exist");
    else
    {
        System::IO::StreamReader^ sr = gcnew System::IO::StreamReader(strFileFull);

        System::String^ strLine;
        int Hmin, Hmax, Smin, Smax, Vmin, Vmax;

        Hmax = System::Convert::ToInt32(sr->ReadLine());
        Hmin = System::Convert::ToInt32(sr->ReadLine());
        Smax = System::Convert::ToInt32(sr->ReadLine());
        Smin = System::Convert::ToInt32(sr->ReadLine());
        Vmax = System::Convert::ToInt32(sr->ReadLine());
        Vmin = System::Convert::ToInt32(sr->ReadLine());

        this->ColorFilter(src, Hmin, Hmax, Smin, Smax, Vmin, Vmax);

        sr->Close();

        delete sr, strLine;
    }
    delete strFile;*/
}

void ImageProcessing::BlurImage(cv::Mat &src)
{
    cv::Mat dst;

    cv::blur(src, dst, cv::Size(this->nBlurMaskSize, this->nBlurMaskSize));
    src = dst;

    dst.release();
}

void ImageProcessing::BinaryImage(cv::Mat &src, int nThreshold)
{
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, cv::COLOR_RGB2GRAY);

    cv::Mat dst = cv::Mat::zeros(src_gray.rows, src_gray.cols, src_gray.type());

    for (int x = 0; x < src_gray.cols; x++) for (int y = 0; y < src_gray.rows; y++)
    {
        if ((int)src_gray.at<uchar>(y, x) > nThreshold)
            dst.at<uchar>(y, x) = (uchar)255;
    }
    cv::cvtColor(dst, src, cv::COLOR_GRAY2BGR);

    src_gray.release();
    dst.release();
}

void ImageProcessing::RoI(cv::Mat &src, int X_min, int X_max, int Y_min, int Y_max )
{
	
	cv::Mat mask( src.rows, src.cols, CV_8UC3 );
	cv::Mat matMaskedImage;
	src.copyTo( mask );
	//�]�wROI
	for (int i = 0; i < src.size().height; i++)
	{
		for (int j = 0; j < src.size().width; j++)
		{
			if (j < X_min || j > X_max || Y_max < i || Y_min > i)
				mask.at<cv::Vec3b>( i, j ) = cv::Vec3b( 0, 0, 0 );    //��l�H�~���ϰ�
			
		}
	}

	src.copyTo( matMaskedImage, mask );
	src = matMaskedImage;
	
	mask.release( );
}

#pragma region ContourTool
void ImageProcessing::ContourTool::FindContours(const cv::Mat &src, int nMinArea)
{
    this->vec2_ptContours.clear();
    
    vector<vector<cv::Point>> vec2_ptContoursFound;
    vector<vector<cv::Point>> vec2_ptContoursRefined;
    
    cv::Mat src_gray;

    cvtColor(src, src_gray, cv::COLOR_RGB2GRAY);
	findContours(src_gray, vec2_ptContoursFound, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    

    for (int i = 0; i < vec2_ptContoursFound.size(); i++)
    {
        int nContourArea = contourArea(vec2_ptContoursFound[i]);

        if (nContourArea > nMinArea)
        {
            vec2_ptContoursRefined.push_back(vec2_ptContoursFound[i]);
        }
    }

    this->vec2_ptContours = vec2_ptContoursRefined;


	
    vec2_ptContoursFound.clear();
    vec2_ptContoursRefined.clear();

    this->FindContoursCenters();
	src_gray.release();
}

void ImageProcessing::ContourTool::FindMaxContours(const cv::Mat &src)
{
    this->vec2_ptContours.clear();

    vector<vector<cv::Point>> vec2_ptContoursFound;
    vector<vector<cv::Point>> vec2_ptContourMax;

    cv::Mat src_gray;

    cvtColor(src, src_gray, cv::COLOR_RGB2GRAY);
    findContours(src_gray, vec2_ptContoursFound, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    

    int nMaxContourArea = 0;

    for (int i = 0; i < vec2_ptContoursFound.size(); i++)
    {
        int nContourArea = contourArea(vec2_ptContoursFound[i]);

        if (nContourArea > nMaxContourArea)
        {
            nMaxContourArea = nContourArea;
            vec2_ptContourMax.clear();
            vec2_ptContourMax.push_back(vec2_ptContoursFound[i]);
        }
    }

    this->vec2_ptContours = vec2_ptContourMax;

    vec2_ptContoursFound.clear();
    vec2_ptContourMax.clear();

    this->FindContoursCenters();
	
	src_gray.release();
}

void ImageProcessing::ContourTool::FindContoursCenters(void)
{
    this->vec_ptCenters.clear();
    this->vec_ptCenters.resize(this->vec2_ptContours.size());

    for (int i = 0; i < this->vec_ptCenters.size(); i++)
    {
        cv::Moments moments = cv::moments(vec2_ptContours[i]);
        double dObjectArea = cv::contourArea(vec2_ptContours[i]);

        this->vec_ptCenters[i].x = moments.m10 / dObjectArea;
        this->vec_ptCenters[i].y = moments.m01 / dObjectArea;
    }
}

void ImageProcessing::ContourTool::RefineToApproximateContours(void)
{
    vector<cv::Point> vec_ptApprox;
    vector<vector<cv::Point>> vec2_ptApproximateContours;

    for (int i = 0; i < this->vec2_ptContours.size(); i++)
    {
        vec_ptApprox.clear();
		//cv::approxPolyDP(cv::Mat(this->vec2_ptContours[i]), vec_ptApprox, cv::arcLength(cv::Mat(this->vec2_ptContours[0]), true) * this->ApproximatecontoursValue, true);// �쥻0.032
		cv::approxPolyDP(cv::Mat(this->vec2_ptContours[i]), vec_ptApprox, this->ApproximatecontoursValue, true); //�]���W�Ȥ��ɦ]���s�W�o��A�쥻�O�W������
        vec2_ptApproximateContours.push_back(vec_ptApprox);
    }
    this->vec2_ptContours = vec2_ptApproximateContours;

    vec_ptApprox.clear();
    vec2_ptApproximateContours.clear();

    this->FindContoursCenters();
}
#pragma endregion ContourTool

#pragma region Morphology
void ImageProcessing::Morphology::DilateImage(cv::Mat &src, int nMaskSize, int nTimes)
{
    cv::Mat dst;
	cv::Mat matStructuringElement = getStructuringElement(cv::MORPH_RECT, cv::Size(nMaskSize, nMaskSize));
    
    for (int i = 0; i < nTimes; i++)
    {
        cv::dilate(src, dst, matStructuringElement);
        src = dst;
    }

    dst.release();
    matStructuringElement.release();
}

void ImageProcessing::Morphology::ErodeImage(cv::Mat &src, int nMaskSize, int nTimes)
{
    cv::Mat dst;
    cv::Mat matStructuringElement = getStructuringElement(cv::MORPH_CROSS, cv::Size(nMaskSize, nMaskSize));

    for (int i = 0; i < nTimes; i++)
    {
        cv::erode(src, dst, matStructuringElement);
        src = dst;
    }

    dst.release();
    matStructuringElement.release();
}

void ImageProcessing::Morphology::OpeningOperation(cv::Mat &src, int nMaskSize, int nTimes)
{
    // erode -> dilate
    cv::Mat dst;
    cv::Mat matStructuringElement = getStructuringElement(cv::MORPH_CROSS, cv::Size(nMaskSize, nMaskSize));

    for (int i = 0; i < nTimes; i++)
    {
        cv::erode(src, dst, matStructuringElement);
        src = dst;
        cv::dilate(src, dst, matStructuringElement);
        src = dst;
    }

    dst.release();
    matStructuringElement.release();
}

void ImageProcessing::Morphology::ClosingOperation(cv::Mat &src, int nMaskSize, int nTimes)
{
    // dilate -> erode
    cv::Mat dst;
    cv::Mat matStructuringElement = getStructuringElement(cv::MORPH_CROSS, cv::Size(nMaskSize, nMaskSize));

    for (int i = 0; i < nTimes; i++)
    {
        cv::dilate(src, dst, matStructuringElement);
        src = dst;
        cv::erode(src, dst, matStructuringElement);
        src = dst;
    }

    dst.release();
    matStructuringElement.release();
}
#pragma endregion Morphology

#pragma region Circle
void ImageProcessing::Circle::FindCircles(cv::Mat &src)
{
    cv::Mat src_gray;
  
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    cv::HoughCircles(src_gray, this->vec_vec3fCircles, cv::HOUGH_GRADIENT, 1, src_gray.rows / 8, 100, this->nCenterDetectionThreshold);

    src_gray.release();
}
#pragma endregion Circle