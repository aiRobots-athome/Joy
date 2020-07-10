#pragma once
#include <string>

class Strategy {
public:
    Strategy();
    ~Strategy();

    void select_strategy(std::string strategy_name);
    void terminate_strategy() { isStrategyRunning = false; }

private:
    bool isStrategyRunning;

    // add strategy below !!!!!!!!
    // Demo *cDemo;
};