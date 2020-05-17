#pragma once
#include "../../Strategy.h"

class Demo_catch_YOLO
{
public:
	Demo_catch_YOLO();
    ~Demo_catch_YOLO();

    void MainStrategy(int);
    void StartStrategy(int);
    void Walk(void);
    void Look_and_Catch(void);
    void Walk_Back(void);
	void FinalStrategy(void);

    void ALL_Combined(int);

    int good_index; // 商品的 YOLO 編號
    
    int nTaskStep;
private:
};