#pragma once

#include <string>

#include "gurobi_c++.h"

class Test {
public:
  Test(std::string filename, int startHour, int timeHorizonHours,
       std::string logFilename);

  void run(GRBEnv &env, bool pUpperBound = false);

private:
  std::string filename;
  int startHour;
  int timeHorizonHours;
  std::string logFilename;
};
