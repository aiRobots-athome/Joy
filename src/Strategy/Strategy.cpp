#include "Strategy.h"
#include "./Scaratest/Scaratest.h"

void Strategy::select_strategy(std::string strategy_name)
{
    isStrategyRunning = true;
    while (isStrategyRunning)
    {
        // add strategy below !!!!!!!!
        if (strategy_name == "Scaratest")
        {
            Scaratest cScaratest;
            cScaratest.test1();
            // Demo cDemo;
            // cDemo.MainStrategy(0);
        }
        else
            ;
    }
}