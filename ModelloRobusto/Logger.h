#pragma once
#include "generate_dac_data.h"
#include <chrono>
#include <fstream>

class Logger {
public:
  Logger(const std::string &filename);

  std::ofstream &getStream();
  void logData(DACData &data);
  int getElapsedTime();

  ~Logger();

private:
  std::ofstream logFile;
  void printGrids(DACData &data);
  std::chrono::high_resolution_clock::time_point startTime;
};