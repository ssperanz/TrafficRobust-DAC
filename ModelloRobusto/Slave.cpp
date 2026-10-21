#include "Slave.h"
#include "Master.h"

Slave::Slave(MasterSolution &sol, GRBEnv &env, DACData &data)
    : T(data.grid.getNTimeslots()), env(env), solution(sol), slave_model(env),
      data(data), p(T, data.flows.size()), q(T, data.airblocks.size()),
      Q(T, data.sectors.size()), e(T, data.sectors.size()),
      E(T, data.configurations.size()), w(T, data.sectors.size()) {
  // std::cout << "Costruzione modello slave..." << std::endl;
  //  Variabili
  for (int t = 0; t < T; t++) {
    for (Flow &flow : data.flows) {
      int f = flow.getId();
      string p_name = "p_" + to_string(t) + "_" + to_string(f);
      p.at(t, f) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,
                                      p_name); // 6.50
    }
    for (AirBlock &ab : data.airblocks) {
      int a = ab.getIndex();
      string q_name = "q_" + to_string(t) + "_" + to_string(a);
      q.at(t, a) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,
                                      q_name); // 6.51
    }
    for (Sector &sector : data.sectors) {
      int s = sector.getIndex();
      string Q_name = "Q_" + to_string(t) + "_" + to_string(s);
      string e_name = "e_" + to_string(t) + "_" + to_string(s);
      string w_name = "w_" + to_string(t) + "_" + to_string(s);
      Q.at(t, s) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,
                                      Q_name); // 6.52
      e.at(t, s) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,
                                      e_name); // 6.52
      w.at(t, s) =
          slave_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, w_name); // 6.54
    }
    for (Configuration &config : data.configurations) {
      int c = config.id;
      string E_name = "E_" + to_string(t) + "_" + to_string(c);
      E.at(t, c) = slave_model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,
                                      E_name); // 6.53
    }
  }

  // std::cout << "Aggiunti vincoli 6.50-6.54" << std::endl;

  for (int t = 0; t < T; t++) {
    for (Flow &flow : data.flows) {
      GRBLinExpr sum_left = 0;
      GRBLinExpr sum_right = 0;
      for (int tau = 0; tau <= t; tau++) {
        sum_left += p.at(tau, flow.getId());
        sum_right += flow.getScheduledDepartures(tau);
      }
      slave_model.addConstr(sum_left <= sum_right); // vincolo 6.43
    }
  }

  // std::cout << "Aggiunti vincoli 6.43" << std::endl;

  for (int t = 0; t < T; t++) {
    for (AirBlock &ab : data.airblocks) {
      int ab_id = ab.getIndex();
      GRBLinExpr sum = 0;
      vector<int> flowsid =
          ab.getFlows(); // flussi che attraversano l’airblock a
      for (int f = 0; f < flowsid.size(); f++) {
        int flowid = flowsid.at(f); //
        Flow &flow = data.flows.at(flowid);
        for (int tau = flow.tau_f_a(t, ab_id);
             tau <= t - flow.getTimeToReachAirBlock(ab_id); tau++) {
          sum += p.at(tau, flowid);
        }
      }
      slave_model.addConstr(q.at(t, ab_id) == sum); // vincolo 6.44
    }
  }

  // std::cout << "Aggiunti vincoli 6.44" << std::endl;

  for (int t = 0; t < T; t++) {
    for (Sector &sector : data.sectors) {
      GRBLinExpr sum = 0;
      for (int tau = t; tau <= t + (60 / data.PERIOD_LENGTH) - 1; tau++) {
        for (AirBlock *ab : sector.airblocks) {
          sum += q.at(tau, ab->getIndex());
        }
      }
      slave_model.addConstr(Q.at(t, sector.getIndex()) == sum); // vincolo 6.45
    }
  }

  // std::cout << "Aggiunti vincoli 6.45" << std::endl;

  int M = 1000000; // TODO: può essere ottimizzato

  for (int t = 0; t < T; t++) {
    for (Sector &sector : data.sectors) {
      int s = sector.getIndex();
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
    for (Configuration &config : data.configurations) {
      GRBLinExpr sum = 0;
      for (Sector *s : config.sectors) {
        sum += e.at(t, s->getIndex());
      }
      slave_model.addConstr(E.at(t, config.id) == sum); // vincolo 6.49
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
  slave_model.write("slave_model.lp");
  this->constructObjective();
}

SlaveSolution Slave::solve() {
  slave_model.optimize();
  SlaveSolution slaveSol(slave_model.get(GRB_DoubleAttr_ObjVal), e);
  return slaveSol;
}

SlaveSolution::SlaveSolution(double objVal, Matrix<GRBVar> &e_vars)
    : excess(objVal), e(e_vars.getnrows(), e_vars.getncols()) {

  int T = e_vars.getnrows();
  int nSectors = e_vars.getncols();

  for (int t = 0; t < T; t++) {
    for (int s = 0; s < nSectors; s++) {
      this->e.at(t, s) = e_vars.at(t, s).get(GRB_DoubleAttr_X);
    }
  }
}