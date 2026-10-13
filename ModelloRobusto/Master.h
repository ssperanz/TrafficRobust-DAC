#pragma once
#include "Matrix.h"
#include "Slave.h"
#include "generate_dac_data.h"
#include "gurobi_c++.h"
#include <vector>

using namespace std;

class MasterSolution {
public:
  int status;
  Matrix<double> x;
  Matrix<double> s;
  double theta;
  MasterSolution(Matrix<GRBVar> &X, Matrix<GRBVar> &S, GRBLinExpr &theta,
                 GRBModel &model)
      : theta(model.get(GRB_DoubleAttr_ObjVal)), x(X.getnrows(), X.getncols()),
        s(X.getnrows(), X.getncols()), status(model.get(GRB_IntAttr_Status)) {
    double t = model.get(GRB_DoubleAttr_ObjVal);
    int T = X.getnrows();
    int C = X.getncols();
    for (int t = 0; t < T; t++) {
      for (int c = 0; c < C; c++) {
        x.at(t, c) = X.at(t, c).get(GRB_DoubleAttr_X);
      }
    }

    for (int t = 0; t < T; t++) {
      for (int c = 0; c < C; c++) {
        s.at(t, c) = S.at(t, c).get(GRB_DoubleAttr_X);
      }
    }
  }
};

class Master {
  GRBEnv env;
  GRBModel master_model;
  DACData data;

  std::vector<SlaveSolution> realizations;
  int T;
  int C;
  Matrix<GRBVar> x;
  Matrix<GRBVar> s;
  GRBLinExpr theta;

public:
  int iter;
  Master(DACData, GRBEnv &);
  MasterSolution solve();
  // MasterSolution getActualSolution();
  void addCut(SlaveSolution &slaveSol);
};