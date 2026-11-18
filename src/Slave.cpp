#include "Slave.h"
#include "Master.h"

Slave::Slave(MasterSolution &sol, GRBEnv &env, DACData &data, Logger &logger,
             bool pUpperBoundEnabled)
    : T(data.grid.getNTimeslots()), env(env), solutionX(sol.x),
      solutionTheta(sol.theta), slave_model(env), data(data), logger(logger),
      p(T, data.flows.size()), q(T, data.airblocks.size()),
      Q(T, data.sectors.size()), e(T, data.sectors.size()),
      E(T, data.configurations.size()), w(T, data.sectors.size()),
      pUpperBoundEnabled(pUpperBoundEnabled) {
  // std::cout << "Costruzione modello slave..." << std::endl;
  //  Variabili
  for (int t = 0; t < T; t++) {
    for (Flow &flow : data.flows) {
      int f = flow.getId();
      string p_name = "p_" + to_string(t) + "_" + to_string(f);
      int upper_bound = flow.getCumulativeScheduledDepartures(t);
      // int upper_bound = flow.getScheduledDepartures(t) + 1;

      p.at(t, f) = slave_model.addVar(
          0.0, pUpperBoundEnabled ? upper_bound : GRB_INFINITY, 0.0,
          GRB_INTEGER, p_name); // 6.50
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

  // for (Flow &flow : data.flows) {
  //   for (int t = 0; t < T; t++) {
  //
  //    int cumulative = 0;
  //
  //    for (int tau = 0; tau <= t; tau++) {
  //      cumulative += flow.getScheduledDepartures(tau);
  //    }
  //    std::ostream &out = logger.getStream();
  //    out << "f = " << flow.getId() << " t = " << t
  //        << " cumulative = " << cumulative
  //        << " upper_bound = " << flow.getCumulativeScheduledDepartures(t)
  //        << "\n";
  //  }
  //}

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
        for (int tau = data.aux.tau_f_a[t][flowid]
                                       [flow.findAbIdxPositionInFlow(ab_id)];
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
      for (int tau = t; tau <= std::min(t + data.parameters.h - 1, T - 1);
           tau++) {
        for (AirBlock *ab : sector.airblocks) {
          sum += q.at(tau, ab->getIndex());
        }
      }
      slave_model.addConstr(Q.at(t, sector.getIndex()) == sum); // vincolo 6.45
    }
  }

  // std::cout << "Aggiunti vincoli 6.45" << std::endl;

  for (int t = 0; t < T; t++) {
    for (Sector &sector : data.sectors) {
      int s = sector.getIndex();
      int capacity_s_t = sector.getCapacity(t);
      slave_model.addConstr(e.at(t, s) <=
                            data.aux.BigM[t] * w.at(t, s)); // vincolo 6.46
      slave_model.addConstr(
          e.at(t, s) <=
          Q.at(t, s) - capacity_s_t +
              capacity_s_t * (1 - w.at(t, s))); // vincolo 6.47, M = C_s_t
      slave_model.addConstr(e.at(t, s) >=
                            Q.at(t, s) - capacity_s_t); // vincolo 6.48
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
  for (int t = 0; t < solutionX.getnrows(); t++) {
    for (int c = 0; c < solutionX.getncols(); c++) {
      sum += E.at(t, c) * solutionX.at(t, c);
    }
  }
  // sum -= solutionTheta; // non necessario

  slave_model.setObjective(sum, GRB_MAXIMIZE);
  obj = sum;
}

void Slave::update(MasterSolution &s) {
  static int i = 1;
  // slave_model.write("slave_model_" + std::to_string(i) + ".lp");
  solutionX = s.x;
  solutionTheta = s.theta;
  i++;

  this->constructObjective();
}

SlaveSolution Slave::solve() {
  // slave_model.set(GRB_DoubleParam_Heuristics, 0.01);
  slave_model.optimize();
  SlaveSolution slaveSol(slave_model.get(GRB_DoubleAttr_ObjVal), p, q, Q, e, E,
                         solutionX);
  return slaveSol;
}

SlaveSolution::SlaveSolution(double objVal, Matrix<GRBVar> &p_vars,
                             Matrix<GRBVar> &q_vars, Matrix<GRBVar> &Q_vars,
                             Matrix<GRBVar> &e_vars, Matrix<GRBVar> &E_vars,
                             Matrix<double> &solutionX)
    : excess(objVal), p(p_vars.getnrows(), p_vars.getncols()),
      q(q_vars.getnrows(), q_vars.getncols()),
      Q(Q_vars.getnrows(), Q_vars.getncols()),
      e(e_vars.getnrows(), e_vars.getncols()),
      E(E_vars.getnrows(), E_vars.getncols()),
      solutionX(solutionX.getnrows(), solutionX.getncols()) {

  for (int t = 0; t < p_vars.getnrows(); t++) {
    for (int f = 0; f < p_vars.getncols(); f++) {
      p.at(t, f) = p_vars.at(t, f).get(GRB_DoubleAttr_X);
    }
  }

  for (int t = 0; t < q_vars.getnrows(); t++) {
    for (int a = 0; a < q_vars.getncols(); a++) {
      q.at(t, a) = q_vars.at(t, a).get(GRB_DoubleAttr_X);
    }
  }

  for (int t = 0; t < Q_vars.getnrows(); t++) {
    for (int s = 0; s < Q_vars.getncols(); s++) {
      Q.at(t, s) = Q_vars.at(t, s).get(GRB_DoubleAttr_X);
      e.at(t, s) = e_vars.at(t, s).get(GRB_DoubleAttr_X);
    }
  }

  for (int t = 0; t < E_vars.getnrows(); t++) {
    for (int c = 0; c < E_vars.getncols(); c++) {
      E.at(t, c) = E_vars.at(t, c).get(GRB_DoubleAttr_X);
    }
  }

  for (int t = 0; t < solutionX.getnrows(); t++) {
    for (int c = 0; c < solutionX.getncols(); c++) {
      this->solutionX.at(t, c) = solutionX.at(t, c);
    }
  }
}
