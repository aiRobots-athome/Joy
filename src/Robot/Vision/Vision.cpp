#include "Vision.h"

Vision::Vision(void)
{
	this->CImageProcessing = new ImageProcessing();
	this->CImageConverter = new ImageConverter();
	this->CMouseEvent = new MouseEvent();
	this->CPclConverter = new PclConverter();
    cout<<"\tClass constructed: Vision"<<endl;
}

Vision::~Vision(void)
{
	delete CImageProcessing;
	delete CImageConverter;
	delete CMouseEvent;
	delete CPclConverter;
}