#pragma once
#include "Matrix.h"
#include "generate_dac_data.h"
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

  GRBEnv env;
  GRBModel slave_model;
  MasterSolution &solution;
  DACData data;

public:
  Slave(MasterSolution &s, GRBEnv &env, DACData &data);
  void constructObjective();
  SlaveSolution solve();
  void update(MasterSolution &s);
};

class SlaveSolution {
public:
  double excess;
  Matrix<double> e;
  SlaveSolution(double obj, Matrix<GRBVar> &e);
};