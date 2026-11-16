#pragma once

#include "Logger.h"
#include "Master.h"
#include "Slave.h"
#include "Viewer.h"
#include "gurobi_c++.h"

struct SolverRunResult {
  int iterations;
  double finalTheta;
  double finalSlaveExcess;
  bool converged;
  bool optimal;
};

class Solver {
public:
  explicit Solver(Logger &logger);
  SolverRunResult run(DACData &data, GRBEnv &env);

private:
  Logger &logger;
};
