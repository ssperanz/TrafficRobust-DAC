#include "Slave.h"
#include "Master.h"

Slave::Slave(MasterSolution &sol, GRBEnv &env, DACData data)
    : T(data.grid.getNTimeslots()), env(env), solution(sol), slave_model(env),
      data(data), p(T, data.flows.size()), q(T, data.airblocks.size()),
      Q(T, data.sectors.size()), e(T, data.sectors.size()),
      E(T, data.configurations.size()), w(T, data.sectors.size()) {
  // std::cout << "Costruzione modello slave..." << std::endl;
  //  Variabili
  for (int t = 0; t < T; t++) {
    for (int f = 0; f < data.flows.size(); f++) {
      string p_name = "p_" + to_string(t) + "_" + to_string(f);
      p.at(t, f) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                      p_name); // 6.50
    }
    for (int a = 0; a < data.airblocks.size(); a++) {
      string q_name = "q_" + to_string(t) + "_" + to_string(a);
      q.at(t, a) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                      q_name); // 6.51
    }
    for (int s = 0; s < data.sectors.size(); s++) {
      string Q_name = "Q_" + to_string(t) + "_" + to_string(s);
      string e_name = "e_" + to_string(t) + "_" + to_string(s);
      string w_name = "w_" + to_string(t) + "_" + to_string(s);
      Q.at(t, s) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                      Q_name); // 6.52
      e.at(t, s) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                      e_name); // 6.52
      w.at(t, s) =
          slave_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, w_name); // 6.54
    }
    for (int c = 0; c < data.configurations.size(); c++) {
      string E_name = "E_" + to_string(t) + "_" + to_string(c);
      E.at(t, c) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                      E_name); // 6.53
    }
  }

  // std::cout << "Aggiunti vincoli 6.50-6.54" << std::endl;

  for (int t = 0; t < T; t++) {
    for (int f = 0; f < data.flows.size(); f++) {
      GRBLinExpr sum_left = 0;
      GRBLinExpr sum_right = 0;
      for (int tao = 0; tao <= t; tao++) {

        sum_left += p.at(tao, f);
        sum_right += data.flows[f].getScheduledDepartures(tao);
      }
      slave_model.addConstr(sum_left <= sum_right); // vincolo 6.43
    }
  }

  // std::cout << "Aggiunti vincoli 6.43" << std::endl;

  for (int t = 0; t < T; t++) {
    for (int a = 0; a < data.airblocks.size(); a++) {
      GRBLinExpr sum = 0;
      for (int f = 0; f < data.airblocks[a].getFlows().size(); f++) {
        for (int tao = data.flows[f].tao_f_a(t, a);
             tao <= t - data.flows[f].getTimeToReachAirBlock(a); tao++) {
          sum += p.at(tao, f);
        }
      }
      slave_model.addConstr(q.at(t, a) == sum); // vincolo 6.44
    }
  }

  // std::cout << "Aggiunti vincoli 6.44" << std::endl;

  for (int t = 0; t < T; t++) {
    for (int s = 0; s < data.sectors.size(); s++) {
      GRBLinExpr sum = 0;
      for (int tao = t; tao < t + (60 / data.PERIOD_LENGTH) - 1; tao++) {
        for (int a = 0; a < data.sectors[s].getNAirblocks(); a++) {
          sum += q.at(tao, a);
        }
      }
      slave_model.addConstr(Q.at(t, s) == sum); // vincolo 6.45
    }
  }

  // std::cout << "Aggiunti vincoli 6.45" << std::endl;

  int M = 1000000; // TODO: può essere ottimizzato

  for (int t = 0; t < T; t++) {
    for (int s = 0; s < data.sectors.size(); s++) {
      slave_model.addConstr(e.at(t, s) <= M * w.at(t, s)); // vincolo 6.46
      slave_model.addConstr(e.at(t, s) <=
                            Q.at(t, s) - data.sectors[s].getCapacity(t) +
                                M * (1 - w.at(t, s))); // vincolo 6.47
      slave_model.addConstr(e.at(t, s) >=
                            Q.at(t, s) -
                                data.sectors[s].getCapacity(t)); // vincolo 6.48
    }
  }

  // std::cout << "Aggiunti vincoli 6.46-6.48" << std::endl;

  for (int t = 0; t < T; t++) {
    for (int c = 0; c < data.configurations.size(); c++) {
      GRBLinExpr sum = 0;
      for (int s = 0; s < data.configurations[c].sectors.size(); s++) {
        sum += e.at(t, s);
      }
      slave_model.addConstr(E.at(t, c) == sum); // vincolo 6.49
    }
  }

  // std::cout << "Aggiunti vincoli 6.49" << std::endl;

  constructObjective();
}

void Slave::constructObjective() {
  GRBLinExpr sum = 0.0;
  for (int t = 0; t < solution.x.getnrows(); t++) {
    for (int c = 0; c < solution.x.getncols(); c++) {
      GRBLinExpr excess = 0.0;
      for (int i = 0; i < data.configurations[c].sectors.size(); i++) {
        excess += e.at(t, data.configurations[c].sectors[i]->getIndex());
      }

      sum += solution.x.at(t, c) * excess;
    }
  }

  slave_model.setObjective(sum, GRB_MAXIMIZE);
  obj = sum;
}

void Slave::update(MasterSolution &s) {
  solution = s;
  this->constructObjective();
}

SlaveSolution Slave::solve() {
  slave_model.optimize();
  SlaveSolution slaveSol(slave_model.get(GRB_DoubleAttr_ObjVal), e);
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