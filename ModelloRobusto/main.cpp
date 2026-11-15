#include "DACCSVGenerator.h"
#include "Logger.h"
#include "Matrix.h"
#include "generate_dac_data.h"
#include "gurobi_c++.h"
// #include "utils.h"
// #include "Writer.h"
// #include "TestData.h"
#include "Master.h"
#include "Slave.h"
#include "Viewer.h"
#include <iostream>
#include <vector>

using namespace std;

int main() {
  int ROWS = 20;
  int COLS = 20;
  int NHOURS = 3;
  int STARTHOUR = 0;
  int MIN_SECTOR_LENGTH = 3;
  int NCONFIGS = 20;
  int NFLOWS = 40;
  int NFLIGHTS = 20;
  int H = 6;
  int T_P = 6;
  double EPSILON = 1e-4;
  double MU_DEPARTURES_DIST = 0.15;

  std::string logFileName = "log-" + std::to_string(NCONFIGS) + "_Configs-" +
                            std::to_string(NHOURS) + "_Hours-" +
                            std::to_string(NFLIGHTS) + "_Flights-" +
                            std::to_string(60 / H) + "_MinDiscretization.txt";

  Logger logger(logFileName);
  std::ostream &out = logger.getStream();

  try {

    out << "Generazione dati DAC...\n";
    DACCSVGenerator generator(ROWS, COLS, NHOURS, STARTHOUR, MIN_SECTOR_LENGTH,
                              NCONFIGS, NFLOWS, NFLIGHTS, H, T_P, EPSILON,
                              MU_DEPARTURES_DIST);
    generator.generateDACCSV("path.csv", 42);
    out << "Dati DAC generati e salvati in path.csv\n";

    GRBEnv env(true);
    env.set("LogFile", "Gurobi.log");
    out << "Inizializzazione ambiente Gurobi...\n";
    env.start();

    out << "Lettura dati...\n\n";
    DACData data("path.csv");
    out << "\n### DATI ###\n";
    logger.logData(data);

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
      out << "  Risoluzione Slave" << endl;
      now = logger.getElapsedTime();
      SlaveSolution slaveSol = slave.solve();
      out << "  Valore obiettivo Slave: " << slaveSol.excess << "\n";
      double max_excess = slaveSol.excess;
      out << "  Risolto Slave in " << logger.getElapsedTime() - now
          << " secondi.\n";
      if (max_excess - sol.theta <= data.parameters.epsilon) {
        out << "Aggiustamento completato! Soluzione ottima trovata." << endl;
        converged = true;
        // Stampa risultati
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
                // A t=0 c'è uno switch se la configurazione si attiva per la
                // prima volta
                if (sol.x.at(t, c) > 0.5) {
                  switch_rilevato = true;
                }
              } else {
                // Per t > 0, c'è uno switch se era spenta prima e accesa ora
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
          out << "Soluzione non ottima. Stato: " << sol.theta << endl;
        }
      } else {
        master.addCut(slaveSol);
      }
      if (master.iter > 100) {
        out << "Numero massimo di iterazioni raggiunto. Interruzione del "
               "processo."
            << endl;
        break;
      }
    }

  } catch (GRBException &e) {
    out << "Errore Gurobi " << e.getErrorCode() << ": " << e.getMessage()
        << endl;
  } catch (const std::exception &e) {
    out << "Eccezione: " << e.what() << endl;
  } catch (...) {
    out << "Eccezione non identificata." << endl;
  }

  out << "Tempo totale di esecuzione: " << logger.getElapsedTime()
      << " secondi." << endl;

  return 0;
}
