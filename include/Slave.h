#pragma once
#include "DACData.h"
#include "Logger.h"
#include "Matrix.h"
#include "gurobi_c++.h"
#include <iostream>
#include <vector>

using namespace std;
class MasterSolution;
class SlaveSolution;

class Slave {
  int excess;
  int T;
  GRBLinExpr obj;
  Matrix<GRBVar> p;
  Matrix<GRBVar> q;
  Matrix<GRBVar> Q;
  Matrix<GRBVar> e;
  Matrix<GRBVar> E;
  Matrix<GRBVar> w;

  bool pUpperBoundEnabled;
  GRBEnv env;
  GRBModel slave_model;
  // MasterSolution &solution;
  Matrix<double> solutionX;
  double solutionTheta;
  DACData data;
  Logger &logger;

public:
  Slave(MasterSolution &s, GRBEnv &env, DACData &data, Logger &logger,
        bool pUpperBoundEnabled = false);
  void constructObjective();
  SlaveSolution solve();
  void update(MasterSolution &s);
};

class SlaveSolution {
public:
  double excess;
  Matrix<double> p;
  Matrix<double> q;
  Matrix<double> Q;
  Matrix<double> e;
  Matrix<double> E;
  Matrix<double> solutionX;

  SlaveSolution(double obj, Matrix<GRBVar> &p_vars, Matrix<GRBVar> &q_vars,
                Matrix<GRBVar> &Q_vars, Matrix<GRBVar> &e_vars,
                Matrix<GRBVar> &E_vars, Matrix<double> &solutionX);
};