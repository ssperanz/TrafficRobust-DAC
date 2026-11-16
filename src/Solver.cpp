#include "Solver.h"

#include <iostream>

Solver::Solver(Logger &logger) : logger(logger) {}

SolverRunResult Solver::run(DACData &data, GRBEnv &env) {
  std::ostream &out = logger.getStream();
  SolverRunResult result{0, 0.0, 0.0, false, false};

  Viewer viewer(data);
  viewer.showConfigurations();

  out << "### RISOLUZIONE ###\n";
  Master master(data, env);
  MasterSolution sol = master.solve();
  Slave slave(sol, env, data);
  bool converged = false;
  while (!converged) {
    out << "Iterazione " << master.iter << "\n";
    out << "  Risoluzione Master\n";
    int now = logger.getElapsedTime();
    sol = master.solve();
    out << "  Valore obiettivo Master: " << sol.theta << "\n";
    out << "  Risolto Master in " << logger.getElapsedTime() - now
        << " secondi.\n";
    slave.update(sol);
    out << "  Aggiornamento Slave completato.\n\n";
    out << "  Risoluzione Slave" << std::endl;
    now = logger.getElapsedTime();
    SlaveSolution slaveSol = slave.solve();
    out << "  Valore obiettivo Slave: " << slaveSol.excess << "\n";
    double max_excess = slaveSol.excess;
    result.finalSlaveExcess = max_excess;
    out << "  Risolto Slave in " << logger.getElapsedTime() - now
        << " secondi.\n";
    if (max_excess - sol.theta <= data.parameters.epsilon) {
      out << "Aggiustamento completato! Soluzione ottima trovata." << std::endl;
      converged = true;
      result.converged = true;
      result.finalTheta = sol.theta;
      result.optimal = (sol.status == GRB_OPTIMAL);
      int T = data.grid.getNTimeslots();
      int C = data.configurations.size();
      if (sol.status == GRB_OPTIMAL) {
        out << "\n===== SOLUZIONE OTTIMA =====\n";
        out << "Valore obiettivo = " << sol.theta << "\n\n";
        out << "\nConfigurazioni attive:\n";
        for (int t = 0; t < T; t++) {
          for (int c = 0; c < C; c++) {
            if (sol.x.at(t, c) > 0.5) {
              out << "t = " << t << " -> configurazione " << c << '\n';
            }
          }
        }

        out << "\nSwitch di configurazione rilevati:\n";
        for (int t = 0; t < T; t++) {
          for (int c = 0; c < C; c++) {
            bool switch_rilevato = false;

            if (t == 0) {
              if (sol.x.at(t, c) > 0.5) {
                switch_rilevato = true;
              }
            } else {
              if (sol.x.at(t, c) > 0.5 && sol.x.at(t - 1, c) < 0.5) {
                switch_rilevato = true;
              }
            }

            if (switch_rilevato) {
              out << "t = " << t << " -> Attivata configurazione " << c;

              if (sol.s.at(t, c) < 0.5) {
                out << " [ATTENZIONE: s_c^t non si è attivata correttamente "
                       "nel solutore!]";
              }
              out << '\n';
            }
          }
        }

      } else {
        out << "Soluzione non ottima. Stato: " << sol.theta << std::endl;
      }
    } else {
      master.addCut(slaveSol);
    }
    if (master.iter > 100) {
      out << "Numero massimo di iterazioni raggiunto. Interruzione del "
             "processo."
          << std::endl;
      break;
    }
  }

  result.iterations = master.iter;
  if (!result.converged) {
    result.finalTheta = sol.theta;
    result.optimal = (sol.status == GRB_OPTIMAL);
  }

  return result;
}
