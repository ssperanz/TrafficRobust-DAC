#include "Solver.h"

#include <iomanip>
#include <iostream>

Solver::Solver(Logger &logger) : logger(logger) {}

SolverRunResult Solver::run(DACData &data, GRBEnv &env,
                            bool pUpperBoundEnabled) {
  std::ostream &out = logger.getStream();
  SolverRunResult result{0, 0.0, 0.0, false, false};

  Viewer viewer(data);
  viewer.showConfigurations();

  out << "### RISOLUZIONE ###\n";
  Master master(data, env, logger);
  MasterSolution sol = master.solve();
  Slave slave(sol, env, data, logger, pUpperBoundEnabled);
  bool converged = false;
  while (!converged) {
    out << "Iterazione " << master.iter << "\n";
    out << "  Risoluzione Master\n";
    double now = logger.getElapsedTime();
    sol = master.solve();
    out << "  Valore obiettivo Master: " << sol.theta << "\n";
    out << "  Risolto Master in " << std::fixed << std::setprecision(2)
        << (logger.getElapsedTime() - now) << std::defaultfloat
        << " secondi.\n";
    slave.update(sol);
    out << "  Aggiornamento Slave completato.\n\n";
    out << "  Risoluzione Slave" << std::endl;
    now = logger.getElapsedTime();
    SlaveSolution slaveSol = slave.solve();
    out << "  Valore obiettivo Slave: " << slaveSol.excess << "\n";

    out << "  Valori finali dello Slave - iterazione " << master.iter - 1
        << ":\n";
    out << std::fixed << std::setprecision(0);

    out << "    p(t,f):\n";
    for (int t = 0; t < slaveSol.p.getnrows(); t++) {
      for (int f = 0; f < slaveSol.p.getncols(); f++) {
        out << "      p(" << t << "," << f
            << ") = " << std::abs(slaveSol.p.at(t, f)) << "\n";
      }
    }

    out << "    q(t,a):\n";
    for (int t = 0; t < slaveSol.q.getnrows(); t++) {
      for (int a = 0; a < slaveSol.q.getncols(); a++) {
        out << "      q(" << t << "," << a
            << ") = " << std::abs(slaveSol.q.at(t, a)) << "\n";
      }
    }

    out << "    Q(t,s):\n";
    for (int t = 0; t < slaveSol.Q.getnrows(); t++) {
      for (int s = 0; s < slaveSol.Q.getncols(); s++) {
        out << "      Q(" << t << "," << s
            << ") = " << std::abs(slaveSol.Q.at(t, s)) << "\n";
      }
    }

    out << "    e(t,s):\n";
    for (int t = 0; t < slaveSol.e.getnrows(); t++) {
      for (int s = 0; s < slaveSol.e.getncols(); s++) {
        out << "      e(" << t << "," << s
            << ") = " << std::abs(slaveSol.e.at(t, s)) << "\n";
      }
    }

    out << "    E(t,c):\n";
    for (int t = 0; t < slaveSol.E.getnrows(); t++) {
      for (int c = 0; c < slaveSol.E.getncols(); c++) {
        out << "      E(" << t << "," << c
            << ") = " << std::abs(slaveSol.E.at(t, c)) << "\n";
      }
    }

    out << "    X(t,c):\n";
    for (int t = 0; t < slaveSol.solutionX.getnrows(); t++) {
      for (int c = 0; c < slaveSol.solutionX.getncols(); c++) {
        out << "      X(" << t << "," << c
            << ") = " << slaveSol.solutionX.at(t, c) << "\n";
      }
    }

    out << std::defaultfloat;
    double max_excess = slaveSol.excess;
    result.finalSlaveExcess = max_excess;
    out << "  Risolto Slave in " << std::fixed << std::setprecision(2)
        << (logger.getElapsedTime() - now) << std::defaultfloat
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
