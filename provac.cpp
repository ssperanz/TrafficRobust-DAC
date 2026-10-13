#include "gurobi_c++.h"
#include <iostream>

using namespace std;

int main() {
  try {
    // Crea l'ambiente
    GRBEnv env(true);
    env.set("LogFile", "gurobi.log");
    env.start();

    // Crea il modello
    GRBModel model(env);

    // Variabili
    GRBVar x = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "x");
    GRBVar y = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "y");

    // Vincolo: x + 2y <= 4
    model.addConstr(x + 2 * y <= 4, "c0");

    // Massimizzazione
    model.setObjective(x + y, GRB_MAXIMIZE);

    // Ottimizza
    model.optimize();

    // Stampa risultati
    cout << "\n=== RISULTATO ===\n";
    cout << "x = " << x.get(GRB_DoubleAttr_X) << endl;
    cout << "y = " << y.get(GRB_DoubleAttr_X) << endl;
    cout << "Obj = " << model.get(GRB_DoubleAttr_ObjVal) << endl;
  } catch (GRBException &e) {
    cout << "Errore Gurobi " << e.getErrorCode() << ": " << e.getMessage()
         << endl;
  } catch (...) {
    cout << "Errore sconosciuto." << endl;
  }

  return 0;
}