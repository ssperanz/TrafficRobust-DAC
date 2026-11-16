#include "DACCSVGenerator.h"
#include "DACData.h"
#include "Logger.h"
#include "Matrix.h"
#include "gurobi_c++.h"
// #include "utils.h"
// #include "Writer.h"
// #include "TestData.h"
#include "Master.h"
#include "Slave.h"
#include "Solver.h"
#include "Viewer.h"
#include <iostream>
#include <vector>

using namespace std;

int main() {
  int ROWS = 20;
  int COLS = 20;
  int NHOURS = 24;
  int STARTHOUR = 0;
  int DATA_STARTHOUR = 6;
  int DATA_TIMEHORIZON_MIN = 2;
  int DATA_TIMEHORIZON_MAX = 5;
  int MIN_SECTOR_LENGTH = 3;
  int NCONFIGS = 20;
  int NFLOWS = 40;
  int NFLIGHTS = 60;
  int H = 6;
  int T_P = 6;
  double EPSILON = 1e-4;
  double MU_DEPARTURES_DIST = 0.15;
  int SEED = 42;

  try {
    std::string timestampPrefix = Logger::makeTimestampPrefix();
    std::string baseLogFileName = timestampPrefix + "-log";
    std::string summaryFileName = timestampPrefix + "-iterations-summary.txt";

    Logger::writeIterationsSummaryHeader(
        summaryFileName, ROWS, COLS, NHOURS, STARTHOUR, MIN_SECTOR_LENGTH,
        NCONFIGS, NFLOWS, NFLIGHTS, H, T_P, EPSILON, MU_DEPARTURES_DIST,
        DATA_STARTHOUR, DATA_TIMEHORIZON_MIN, DATA_TIMEHORIZON_MAX, SEED);

    std::cout << "Generazione dati DAC..." << std::endl;
    DACCSVGenerator generator(ROWS, COLS, NHOURS, STARTHOUR, MIN_SECTOR_LENGTH,
                              NCONFIGS, NFLOWS, NFLIGHTS, H, T_P, EPSILON,
                              MU_DEPARTURES_DIST);
    generator.generateDACCSV("path.csv", SEED);
    std::cout << "Dati DAC generati e salvati in path.csv" << std::endl;

    GRBEnv env(true);
    env.set("LogFile", "Gurobi.log");
    std::cout << "Inizializzazione ambiente Gurobi..." << std::endl;
    env.start();

    for (int dataTimeHorizon = DATA_TIMEHORIZON_MIN;
         dataTimeHorizon <= DATA_TIMEHORIZON_MAX; dataTimeHorizon++) {
      std::string logFileName =
          baseLogFileName + "-h" + std::to_string(dataTimeHorizon) + ".txt";
      Logger logger(logFileName);
      std::ostream &out = logger.getStream();

      out << "### PARAMETRI ITERAZIONE ###\n";
      out << "DATA_STARTHOUR = " << DATA_STARTHOUR << "\n";
      out << "DATA_TIMEHORIZON = " << dataTimeHorizon << " ore\n\n";

      out << "Lettura dati...\n\n";
      DACData data("path.csv", DATA_STARTHOUR, dataTimeHorizon);
      out << "\n### DATI ###\n";
      logger.logData(data);

      Solver solver(logger);
      SolverRunResult runResult = solver.run(data, env);

      Logger::IterationSummaryRow summaryRow{
          dataTimeHorizon,
          runResult.iterations,
          static_cast<double>(logger.getElapsedTime()),
          data.grid.getNTimeslots(),
          data.aux.totalFlights,
          runResult.finalTheta,
          runResult.finalSlaveExcess,
          runResult.converged,
          runResult.optimal};
      Logger::appendIterationSummaryRow(summaryFileName, summaryRow);

      out << "Tempo totale di esecuzione: " << logger.getElapsedTime()
          << " secondi." << endl;
    }

  } catch (GRBException &e) {
    std::cerr << "Errore Gurobi " << e.getErrorCode() << ": " << e.getMessage()
              << endl;
  } catch (const std::exception &e) {
    std::cerr << "Eccezione: " << e.what() << endl;
  } catch (...) {
    std::cerr << "Eccezione non identificata." << endl;
  }

  return 0;
}
