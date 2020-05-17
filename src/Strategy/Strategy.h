#pragma once
#include <string>

class Strategy
{
public:
    Strategy();
    ~Strategy();

    void select_strategy(std::string strategy_name);
    void terminate_strategy();

private:
    bool isStrategyRunning;

    // add strategy below !!!!!!!!
    // Demo_catch_YOLO *cDemoCatchYolo;
};