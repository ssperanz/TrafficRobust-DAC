#include "Master.h"
#include "Slave.h"

#include <iostream>

MasterSolution Master::solve() {
  master_model.optimize();
  MasterSolution solution(x, s, theta, master_model);
  iter++;
  return solution;
}

Master::Master(DACData data, GRBEnv &env)
    : T(data.grid.getNTimeslots()), C(data.getNConfigurations()), x(T, C),
      s(T, C), env(env), master_model(env), data(data), iter(0) {
  int t_p = data.grid.get_t_p();
  // std::cout << "Numero di timeslots: " << T << std::endl;
  // std::cout << "Costruzione modello master..." << std::endl;
  //  TODO?? 6.34 con valori nominali

  for (int c = 0; c < C; c++) {
    for (int t = 0; t < T; t++) {
      string x_name = "x_" + to_string(t) + "_" + to_string(c);
      string s_name = "s_" + to_string(t) + "_" + to_string(c);
      x.at(t, c) =
          master_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, x_name); // 6.40
      s.at(t, c) =
          master_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, s_name); // 6.41
    }
  }
  //
  //  GRBLinExpr TotalExcess = 0.0;
  //  for (int t = 0; t < T; t++) {
  //    for (int c = 0; c < C; c++) {
  //      double PartialConfigurationCExcess = 0.0;
  //      for (int i = 0; i < data.configurations[c].sectors.size(); i++) {
  //        PartialConfigurationCExcess +=
  //            data.configurations[c].sectors[i]->getExcess(t);
  //      }
  //      TotalExcess += PartialConfigurationCExcess * x.at(t, c);
  //    }
  //  }
  //  master_model.addConstr(TotalExcess <= theta); // 6.34

  for (int t = 0; t < T; t++) {
    GRBLinExpr sum = 0;
    for (int c = 0; c < C; c++)
      sum += x.at(t, c);
    master_model.addConstr(sum == 1); // 6.35
  }

  // TODO da rivedere
  for (int t = 0; t < T - 1; t++) {
    for (int c = 0; c < C; c++) {
      vector<int> c_similar =
          data.getSimilarConfigurationsIndex(data.configurations[c]);
      GRBLinExpr sum = 0;
      for (int c_similar_idx = 0; c_similar_idx < c_similar.size();
           c_similar_idx++) {
        sum += x.at(t + 1, c_similar[c_similar_idx]);
      }

      master_model.addConstr(x.at(t, c) - sum <= 0); // 6.36
    }
  }

  for (int c = 0; c < C; c++) {
    master_model.addConstr(x.at(0, c) <= s.at(0, c)); // 6.39
    for (int t = 1; t < T; t++) {
      master_model.addConstr(x.at(t, c) - x.at(t - 1, c) <= s.at(t, c)); // 6.38
    }
  }

  for (int t = 0; t < T - t_p; t++) {
    GRBLinExpr sum = 0;
    for (int tao = t; tao <= min(t + t_p - 1, T - 1); tao++) {
      for (int c = 0; c < C; c++) {
        sum += s.at(tao, c); // 6.37
      }
    }
    master_model.addConstr(sum <= 1);
  }

  // Inizializza la variabile theta
  theta = master_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "theta");

  // Sincronizza il modello
  master_model.update();

  master_model.setObjective(theta, GRB_MINIMIZE);

  master_model.write("master_model.lp");
}

void Master::addCut(SlaveSolution &slaveSol) {
  realizations.push_back(slaveSol);
  GRBLinExpr cut = 0;
  for (int t = 0; t < T; t++) {
    for (int c = 0; c < C; c++) {
      double ConfigurationCExcess = 0.0;
      for (int i = 0; i < data.configurations[c].sectors.size(); i++) {
        ConfigurationCExcess +=
            slaveSol.e.at(t, data.configurations[c].sectors[i]->getIndex());
      }
      cut += x.at(t, c) * ConfigurationCExcess;
    }
  }
  master_model.addConstr(theta >= cut);
  master_model.update();
}