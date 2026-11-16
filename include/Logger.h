#pragma once
#include "DACData.h"
#include <chrono>
#include <fstream>
#include <string>

class Logger {
public:
  struct IterationSummaryRow {
    int dataTimeHorizon;
    int iterations;
    double elapsedSeconds;
    int timeslots;
    int totalFlights;
    double finalTheta;
    double finalSlaveExcess;
    bool converged;
    bool optimal;
  };

  Logger(const std::string &filename);

  static std::string makeTimestampPrefix();
  std::ofstream &getStream();
  void logData(DACData &data);
  int getElapsedTime();
  static void writeIterationsSummaryHeader(
      const std::string &filename, int rows, int cols, int nHours,
      int startHour, int minSectorLength, int nConfigs, int nFlows,
      int nFlights, int h, int t_p, double epsilon, double muDeparturesDist,
      int dataStartHour, int timeHorizonMin, int timeHorizonMax, int seed);
  static void appendIterationSummaryRow(const std::string &filename,
                                        const IterationSummaryRow &row);

  ~Logger();

private:
  std::ofstream logFile;
  void printGrids(DACData &data);
  std::chrono::high_resolution_clock::time_point startTime;
};