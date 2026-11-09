#include "generate_dac_data.h"
#include <fstream>

class Logger {
public:
  Logger(const std::string &filename);

  std::ofstream &getStream();
  void logData(DACData &data);

  ~Logger();

private:
  std::ofstream logFile;
  void printGrids(DACData &data);
};