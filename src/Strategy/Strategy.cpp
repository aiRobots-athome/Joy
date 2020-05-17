#include "Strategy.h"

Strategy::Strategy()
{
    isStrategyRunning = false;
}

Strategy::~Strategy()
{
}

void Strategy::select_strategy(std::string strategy_name)
{
    isStrategyRunning = true;
    while (isStrategyRunning)
    {
        // add strategy below !!!!!!!!
        if (strategy_name == "Demo_catch_YOLO")
        {
            // Demo_catch_YOLO cDemo_catch_YOLO;
            // cDemo_catch_YOLO.MainStrategy(0);
        }
        else
            ;
    }
}

void Strategy::terminate_strategy()
{
    isStrategyRunning = false;
}