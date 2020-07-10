#include "Strategy.h"
#include "./Scaratest/Scaratest.h"

// Constructor
Strategy::Strategy() {
    isStrategyRunning = false;
}

// Destructor
Strategy::~Strategy() {}

/**
 * Run choosen strategy
 * 
 * @param strategy_name - the strategy name choosen by user
 */
void Strategy::select_strategy(std::string strategy_name) {
    isStrategyRunning = true;
    while (isStrategyRunning) {
        // add strategy below !!!!!!!!
        // If the strategy name match the cases, execute the strategy
        if (strategy_name == "Scaratest") {
            Scaratest cScaratest;
            cScaratest.test1();
            // Demo cDemo;
            // cDemo.MainStrategy(0);
        }
        else
            ;
    }
}