#include "Master.h"
#include "Slave.h"

#include <iostream>

MasterSolution Master::solve() {
  model.optimize();
  MasterSolution solution(x, s, theta, model);
  iter++;
  return solution;
}

Master::Master(DACData data, GRBEnv &env)
    : T(data.grid.getNTimeslots()), C(data.getNConfigurations()), x(T, C),
      s(T, C), env(env), model(env), data(data), iter(0) {
  int T = data.grid.getNTimeslots();
  int C = data.getNConfigurations();

  for (int t = 0; t < T; t++) {
    for (int c = 0; c < C; c++) {
      string name = "x_" + to_string(t) + "_" + to_string(c);
      x.at(t, c) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
    }
  }

  for (int t = 0; t < T; t++) {
    for (int c = 0; c < C; c++) {
      string name = "s_" + to_string(t) + "_" + to_string(c);
      s.at(t, c) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
    }
  }

  // Vincolo 5.22 uguale al nominale
  for (int t = 0; t < T; t++) {
    GRBLinExpr sum = 0;
    for (int c = 0; c < C; c++)
      sum += x.at(t, c);

    model.addConstr(sum == 1);
  }

  // Vincolo 5.23 uguale al nominale
  for (int t = 0; t < T - 1; t++) {
    int t_succ = t + 1;
    for (int c = 0; c < C; c++) {
      vector<int> c_similar =
          data.getSimilarConfigurationsIndex(data.configurations[c]);
      // vector<int> c_similar;
      // c_similar.push_back(0); //DA RIMUOVERE
      // c_similar.push_back(1); //DA RIMUOVERE
      GRBLinExpr sum = 0;
      for (int c_similar_idx = 0; c_similar_idx < c_similar.size();
           c_similar_idx++) {
        sum += x.at(t_succ, c_similar[c_similar_idx]);
      }

      model.addConstr(x.at(t, c) - sum <= 0);
    }
  }

  // Vincolo 5.24 uguale al nominale
  for (int t = 0; t < T; t++) {
    int t_prec = t - 1;
    for (int c = 0; c < C; c++) {
      if (t == 0) {
        model.addConstr(x.at(t, c) <= s.at(t, c));
      } else {
        model.addConstr(x.at(t, c) - x.at(t_prec, c) <= s.at(t, c));
      }
    }
  }

  // Vincolo 5.25 uguale al nominale
  int h = data.grid.geth();
  for (int t = 0; t < T; t++) {
    GRBLinExpr sum = 0;
    int upper_limit = std::min(t + h - 1, T - 1);
    for (int y = t; y <= upper_limit; y++) {
      for (int c = 0; c < C; c++) {
        sum += s.at(y, c);
      }
    }
    model.addConstr(sum <= 1);
  }
  // Inizializza la variabile theta
  theta = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "theta");

  // Sincronizza il modello
  model.update();

  model.setObjective(theta, GRB_MINIMIZE);
}

void Master::addCut(SlaveSolution &slaveSol) {
  realizations.push_back(slaveSol);
  GRBLinExpr cut = 0;
  for (int t = 0; t < T; t++) {
    for (int c = 0; c < C; c++) {
      double ConfigurationCExcess = 0.0;
      for (int i = 0; i < data.configurations[c].sectors.size(); i++) {
        ConfigurationCExcess +=
            slaveSol.e.at(data.configurations[c].sectors[i]->getIndex(), t);
      }
      cut += x.at(t, c) * ConfigurationCExcess;
    }
  }
  model.addConstr(theta >= cut);
  model.update();
}