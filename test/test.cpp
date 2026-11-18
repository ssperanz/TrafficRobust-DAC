#include "Test.h"

#include <iomanip>
#include <iostream>
#include <utility>

#include "DACData.h"
#include "Logger.h"
#include "Solver.h"

Test::Test(std::string filename, int startHour, int timeHorizonHours,
           std::string logFilename)
    : filename(std::move(filename)), startHour(startHour),
      timeHorizonHours(timeHorizonHours), logFilename(std::move(logFilename)) {}

void Test::run(GRBEnv &env, bool pUpperBound) {
  Logger logger(logFilename);
  std::ostream &out = logger.getStream();

  out << "### TEST ISTANZA PICCOLA ###\n";
  out << "File: " << filename << "\n";
  out << "DATA_STARTHOUR = " << startHour << "\n";
  out << "DATA_TIMEHORIZON = " << timeHorizonHours << " ore\n";
  out << "P_UPPERBOUND = " << std::boolalpha << pUpperBound << "\n\n";

  DACData data(filename, startHour, timeHorizonHours);
  logger.logData(data);

  Solver solver(logger);
  SolverRunResult result = solver.run(data, env, pUpperBound);

  out << "\n### RISULTATO TEST ###\n";
  out << "Converged: " << std::boolalpha << result.converged << "\n";
  out << "Optimal: " << result.optimal << "\n";
  out << "Iterations: " << result.iterations << "\n";
  out << "Final theta: " << result.finalTheta << "\n";
  out << "Final slave excess: " << result.finalSlaveExcess << "\n";
  out << "Tempo totale di esecuzione: " << std::fixed << std::setprecision(2)
      << logger.getElapsedTime() << std::defaultfloat << " secondi.\n";
}