#include "Strategy.h"

void Strategy::select_strategy(std::string strategy_name)
{
    isStrategyRunning = true;
    while (isStrategyRunning)
    {
        // add strategy below !!!!!!!!
        if (strategy_name == "Demo")
        {
            // Demo cDemo;
            // cDemo.MainStrategy(0);
        }
        else
            ;
    }
}