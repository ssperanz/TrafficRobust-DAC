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
  int C;
  GRBLinExpr obj;
  Matrix<GRBVar> e;
  Matrix<GRBVar> y;
  Matrix<GRBVar> z;

  GRBEnv env;
  GRBModel model;
  MasterSolution &solution;
  DACData data;

public:
  Slave(MasterSolution &s, GRBEnv &env, DACData data);
  void constructObjective();
  SlaveSolution solve();
  GRBLinExpr f(int i, int t,
               Matrix<GRBVar> &y); // indice i è riferito al settore, e t al
                                   // tempo. y(k,t) invece dice se si verifica
                                   // maltempo nell'airblock k tempo t
  void update(MasterSolution &s);
};

class SlaveSolution {
public:
  double excess;
  Matrix<double> e;
  SlaveSolution(double obj, Matrix<GRBVar> &e);
};