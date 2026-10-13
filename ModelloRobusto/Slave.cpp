#include "Slave.h"
#include "Master.h"

Slave::Slave(MasterSolution &sol, GRBEnv &env, DACData data)
    : T(data.grid.getNTimeslots()), env(env), solution(sol), model(env),
      data(data), e(data.sectors.size(), T),
      y(data.grid.getNCols() * data.grid.getNRows(), T),
      z(data.sectors.size(), T) {
  // creo modello slave

  // variabile y: variabile binaria che indica se si verifica effettivamente
  // maltempo nell’airblock k nell’intervallo t
  for (int a = 0; a < data.grid.getNCols() * data.grid.getNRows(); a++) {
    for (int t = 0; t < T; t++) {
      string name = "y_" + to_string(a) + "_" + to_string(t);
      y.at(a, t) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
    }
  }

  // variabile z: zi,t = 1 se e solo se ce eccesso nel settore i nell’intervallo
  // t

  for (int s = 0; s < data.sectors.size(); s++) {
    for (int t = 0; t < T; t++) {
      string name = "z_" + to_string(s) + "_" + to_string(t);
      z.at(s, t) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
    }
  }

  // variabile e: eccesso settore i tempo t

  for (int s = 0; s < data.sectors.size(); s++) {
    for (int t = 0; t < T; t++) {
      string name = "e" + to_string(s) + "_" + to_string(t);
      e.at(s, t) = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
    }
  }

  // vincoli che modellano e

  double wSum = data.grid.w[data.grid.w.size() - 1]; // somma dei pesi w
  for (int s = 0; s < data.sectors.size(); s++) {
    for (int t = 0; t < T; t++) {
      // double
      // M=data.sectors[s].getTraffic(t)+(data.sectors[s].getCapacity(t))+wSum;
      // //big M per il vincolo 5.37
      double M =
          data.sectors[s].getTraffic(t) + data.sectors[s].getCapacity(t) * 2.0;
      model.addConstr(e.at(s, t) >= f(s, t, y)); // vincolo 5.35
      model.addConstr(e.at(s, t) >= 0);          // vincolo 5.36
      model.addConstr(e.at(s, t) <=
                      f(s, t, y) + M * (1 - z.at(s, t))); // vincolo 5.37
      model.addConstr(e.at(s, t) <= M * z.at(s, t));      // vincolo 5.38
    }
  }

  const int NUM_P_LEVELS = 3;

  int totalAirblocks = data.grid.getNCols() * data.grid.getNRows();

  for (int t = 0; t < T; t++) {
    for (int p = 1; p <= NUM_P_LEVELS; p++) {

      GRBLinExpr sum_y = 0.0;
      int count_p = 0; // Conta quanti airblock hanno pressione p al tempo t

      for (int a = 0; a < totalAirblocks; a++) {
        if (data.grid.airBlock(a).getplevel(t) == p) {
          sum_y += y.at(a, t);
          count_p++;
        }
      }

      if (count_p > 0) {

        double rhs = data.grid.pi[p - 1] * count_p;

        model.addConstr(sum_y <= rhs);
      }
    }
  }

  // vincolo 5.40
  for (int a = 0; a < data.grid.getNCols() * data.grid.getNRows(); a++) {
    for (int t = 0; t < T; t++) {
      if (data.grid.airBlock(a).getglevel(t) == 1 &&
          data.grid.airBlock(a).getplevel(t) == 1) {
        model.addConstr(y.at(a, t) == 0.0); // vincolo 5.40
      }
    }
  }

  constructObjective();
}

GRBLinExpr Slave::f(int i, int t,
                    Matrix<GRBVar> &y) { // indice i = settore, t = tempo
  double traffic = data.sectors[i].getTraffic(t);
  double capacity = data.sectors[i].getCapacity(t);

  // frazione di capacità per singolo airblock
  double fraction =
      data.sectors[i].getCapacity(t) / data.sectors[i].getNAirblocks();
  // somma pesata del meteo sugli airblock del settore
  GRBLinExpr media = 0.0;
  for (int a = 0; a < data.sectors[i].getNAirblocks(); a++) {
    int glevel = data.sectors[i].airblocks[a]->getglevel(t);
    media += y.at(data.sectors[i].airblocks[a]->getIndex(), t) *
             data.grid.w[glevel - 1];
  }

  media = fraction * media;

  return (traffic - capacity) + media;
}

void Slave::constructObjective() {
  GRBLinExpr sum = 0.0;
  for (int t = 0; t < solution.x.getnrows(); t++) {
    for (int c = 0; c < solution.x.getncols(); c++) {
      GRBLinExpr excess = 0.0;
      for (int i = 0; i < data.configurations[c].sectors.size(); i++) {
        excess += e.at(data.configurations[c].sectors[i]->getIndex(), t);
      }

      sum += solution.x.at(t, c) * excess;
    }
  }

  model.setObjective(sum, GRB_MAXIMIZE);
  obj = sum;
}

void Slave::update(MasterSolution &s) {
  solution = s;
  this->constructObjective();
}

SlaveSolution Slave::solve() {
  model.optimize();
  SlaveSolution slaveSol(model.get(GRB_DoubleAttr_ObjVal), e);
  return slaveSol;
}

SlaveSolution::SlaveSolution(double objVal, Matrix<GRBVar> &e_vars)
    : excess(objVal), e(e_vars.getnrows(), e_vars.getncols()) {

  int nSectors = e_vars.getnrows();
  int T = e_vars.getncols();

  for (int s = 0; s < nSectors; s++) {
    for (int t = 0; t < T; t++) {
      this->e.at(s, t) = e_vars.at(s, t).get(GRB_DoubleAttr_X);
    }
  }
}