#include "DACCSVGenerator.h"
#include "Matrix.h"
#include "generate_dac_data.h"
#include "gurobi_c++.h"
// #include "utils.h"
// #include "Writer.h"
// #include "TestData.h"
#include "Master.h"
#include "Slave.h"
#include <iostream>
#include <vector>

using namespace std;

int main() {
  try {
    std::cout << "Generazione dati DAC..." << std::endl;
    DACCSVGenerator generator(
        20, 20, 15, 6, 3, 20, 1, 5,
        1e-4); // parametri: ROWS, COLS, NTIMES, START_HOUR, MIN_SECTOR_LENGTH,
               // NCONFIGS, H, T_P, EPSILON
    generator.generateDACCSV("path.csv", 42);
    std::cout << "Dati DAC generati e salvati in path.csv" << std::endl;

    GRBEnv env(true);
    env.set("LogFile", "TrafficRobust-DAC.log");
    std::cout << "Inizializzazione ambiente Gurobi..." << std::endl;
    env.start();

    std::cout << "inizio" << std::endl;
    DACData data("path.csv");
    std::cout << "lettura dati" << std::endl;
    for (int c = 0; c < data.configurations.size(); c++) {
      cout << "Configurazione " << c << endl;

      for (auto s : data.configurations[c].sectors) {
        cout << " settore " << s->getIndex() << " cap t0=" << s->getCapacity(0)
             << " traffic t0=" << s->getTraffic(0) << endl;
      }
    }
    Master master(data, env);
    MasterSolution sol = master.solve();
    Slave slave(sol, env, data);

    bool converged = false;
    while (!converged) {
      std::cout << "Iterazione " << master.iter << std::endl;
      sol = master.solve();
      slave.update(sol);

      cout << "Risoluzione slave" << endl;
      SlaveSolution slaveSol = slave.solve();
      double max_excess = slaveSol.excess;
      if (max_excess - sol.theta <= data.parameters.epsilon) {
        cout << "Aggiustamento completato! Soluzione ottima trovata." << endl;
        converged = true;
        // Stampa risultati
        int T = data.grid.getNTimeslots();
        int C = data.configurations.size();
        if (sol.status == GRB_OPTIMAL) {
          cout << "\n===== SOLUZIONE OTTIMA =====\n";
          cout << "Valore obiettivo = " << sol.theta << "\n\n";
          cout << "\nConfigurazioni attive:\n";
          for (int t = 0; t < T; t++) {
            for (int c = 0; c < C; c++) {
              if (sol.x.at(t, c) > 0.5) {
                cout << "t = " << t << " -> configurazione " << c << '\n';
              }
            }
          }

          cout << "\nSwitch di configurazione rilevati:\n";
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
                cout << "t = " << t << " -> Attivata configurazione " << c;

                if (sol.s.at(t, c) < 0.5) {
                  cout << " [ATTENZIONE: s_c^t non si è attivata correttamente "
                          "nel solutore!]";
                }
                cout << '\n';
              }
            }
          }

        } else {
          cout << "Soluzione non ottima. Stato: " << sol.theta << endl;
        }
      } else {
        master.addCut(slaveSol);
      }
      if (master.iter > 100) {
        cout << "Numero massimo di iterazioni raggiunto. Interruzione del "
                "processo."
             << endl;
        break;
      }
    }

  } catch (GRBException &e) {
    cout << "Errore Gurobi " << e.getErrorCode() << ": " << e.getMessage()
         << endl;
  } catch (const std::exception &e) {
    cout << "Eccezione: " << e.what() << endl;
  } catch (...) {
    cout << "Eccezione non identificata." << endl;
  }
  return 0;
}
