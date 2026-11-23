#include "DACCSVGenerator.h"
#include "DACData.h"
#include "Logger.h"
#include "Matrix.h"
#include "gurobi_c++.h"
// #include "utils.h"
// #include "Writer.h"
// #include "TestData.h"
#include "Solver.h"
#include "Test.h"
#include "Viewer.h"
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

int main() {
  int ROWS = 20;
  int COLS = 20;
  int NHOURS = 24;
  int STARTHOUR = 0;
  int DATA_STARTHOUR = 8;
  int DATA_TIMEHORIZON_MIN = 2;
  int DATA_TIMEHORIZON_MAX = 5;
  int MIN_SECTOR_LENGTH = 3;
  int NCONFIGS = 100;
  int NFLOWS = 80;
  int NFLIGHTS = 200;
  int H = 6;
  int T_P = 6;
  double EPSILON = 1e-4;
  double MU_DEPARTURES_DIST = 0.15;
  std::vector<int> seeds = {108} /*{42, 78, 103, 108, 123}*/;
  std::vector<int> datastarthours = {8, 13, 18};

  try {
    std::string timestampPrefix = Logger::makeTimestampPrefix();

    GRBEnv env(true);
    env.set("LogFile", "Gurobi.log");
    std::cout << "Inizializzazione ambiente Gurobi..." << std::endl;
    env.start();

    bool RUN_SMALL_INSTANCE = false;
    if (RUN_SMALL_INSTANCE) {
      Test test("../test/test1.csv", 0, 2, "test1-log.txt");
      test.run(env, false);
    } else {
      for (int seed : seeds) {
        std::cout << "\n=== Generazione dati per seed " << seed
                  << " ===" << std::endl;
        DACCSVGenerator generator(ROWS, COLS, NHOURS, STARTHOUR,
                                  MIN_SECTOR_LENGTH, NCONFIGS, NFLOWS, NFLIGHTS,
                                  H, T_P, EPSILON, MU_DEPARTURES_DIST, seed);
        generator.generateDACCSV("path.csv");

        for (int dataStartHour : datastarthours) {
          for (bool p_upperbound : {false}) {
            std::string baseLogFileName =
                timestampPrefix + "-seed" + std::to_string(seed) + "-hour" +
                std::to_string(dataStartHour) +
                (p_upperbound ? "-pupper" : "-pNoUpper");
            std::string summarySeedFileName =
                baseLogFileName + "-iterations-summary.txt";

            Logger::writeIterationsSummaryHeader(
                summarySeedFileName, ROWS, COLS, NHOURS, STARTHOUR,
                MIN_SECTOR_LENGTH, NCONFIGS, NFLOWS, NFLIGHTS, H, T_P, EPSILON,
                MU_DEPARTURES_DIST, dataStartHour, DATA_TIMEHORIZON_MIN,
                DATA_TIMEHORIZON_MAX, seed);

            std::cout << "\n=== Seed " << seed
                      << " / p_upperbound = " << std::boolalpha << p_upperbound
                      << " ===" << std::endl;

            for (int dataTimeHorizon = DATA_TIMEHORIZON_MIN;
                 dataTimeHorizon <= DATA_TIMEHORIZON_MAX; dataTimeHorizon++) {
              std::string logFileName = baseLogFileName + "-h" +
                                        std::to_string(dataTimeHorizon) +
                                        ".txt";
              Logger logger(logFileName);
              std::ostream &out = logger.getStream();

              out << "### PARAMETRI ITERAZIONE ###\n";
              out << "SEED = " << seed << "\n";
              out << "P_UPPERBOUND = " << std::boolalpha << p_upperbound
                  << "\n";
              out << "DATA_STARTHOUR = " << dataStartHour << "\n";
              out << "DATA_TIMEHORIZON = " << dataTimeHorizon << " ore\n\n";

              out << "Lettura dati...\n\n";
              DACData data("path.csv", dataStartHour, dataTimeHorizon);
              out << "\n### DATI ###\n";
              logger.logData(data);

              Solver solver(logger);
              SolverRunResult runResult = solver.run(data, env, p_upperbound);

              Logger::IterationSummaryRow summaryRow{
                  seed,
                  p_upperbound,
                  dataTimeHorizon,
                  runResult.iterations - 1,
                  static_cast<double>(logger.getElapsedTime()),
                  data.grid.getNTimeslots(),
                  data.aux.totalFlights,
                  runResult.finalTheta,
                  runResult.finalSlaveExcess,
                  runResult.converged,
                  runResult.optimal};
              Logger::appendIterationSummaryRow(summarySeedFileName,
                                                summaryRow);

              out << "Tempo totale di esecuzione: " << std::fixed
                  << std::setprecision(2) << logger.getElapsedTime()
                  << std::defaultfloat << " secondi." << endl;
            }
          }
        }
      }
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
