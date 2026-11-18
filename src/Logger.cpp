#include "Logger.h"
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

Logger::Logger(const std::string &filename)
    : logFile(filename), startTime(std::chrono::high_resolution_clock::now()) {
  if (!logFile.is_open()) {
    throw std::runtime_error("Impossibile aprire il file di log: " + filename);
  }
}

std::string Logger::makeTimestampPrefix() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::tm tm_buf;
  localtime_r(&in_time_t, &tm_buf);

  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::ofstream &Logger::getStream() { return logFile; }

void Logger::printGrids(DACData &data) {
  int rows = data.grid.getNRows();
  int cols = data.grid.getNCols();
  int configs = data.getNConfigurations();

  std::vector<std::vector<int>> grid(configs,
                                     std::vector<int>(rows * cols, -1));

  for (auto &config : data.configurations) {
    for (auto &sector : config.sectors) {
      int sectorId = sector->getIndex();
      for (auto &airblock : sector->airblocks) {
        int abIndex = airblock->getIndex();
        grid[config.id][abIndex] = sectorId;
      }
    }
  }

  /** Prints the grid, showing sectorid in each cell, for each configuration */
  for (int c = 0; c < configs; ++c) {
    logFile << "Configurazione " << c << ":\n";
    for (int r = 0; r < rows; ++r) {
      for (int col = 0; col < cols; ++col) {
        int abIndex = r * cols + col;
        int sectorId = grid[c][abIndex];
        if (sectorId == -1) {
          logFile << std::setw(4) << "ERR";
        } else {
          logFile << std::setw(4) << sectorId;
        }
      }
      logFile << "\n";
    }
    logFile << "\n";
  }

  /* Prints the flows path singularly on the grid */
  for (const auto &flow : data.flows) {
    logFile << "Traiettoria " << flow.id << ":\n";
    logFile << "\n";
    for (int r = 0; r < rows; ++r) {
      for (int col = 0; col < cols; ++col) {
        int abIndex = r * cols + col;
        bool isFlowCell =
            std::find_if(flow.airblocks.begin(), flow.airblocks.end(),
                         [abIndex](AirBlock *ab) {
                           return ab->getIndex() == abIndex;
                         }) != flow.airblocks.end();

        if (isFlowCell) {
          logFile << std::setw(4) << "*";
        } else {
          logFile << std::setw(4) << abIndex;
        }
      }
      logFile << "\n";
    }
    logFile << "\nAirblock partenza: " << flow.airblocks.front()->getIndex()
            << ", Airblock arrivo: " << flow.airblocks.back()->getIndex()
            << "\n";
    logFile << "Partenze programmate ai timeslot: \n";
    for (int t = 0; t < flow.num_of_scheduled_departures.size(); ++t) {
      if (flow.num_of_scheduled_departures[t] > 0) {
        logFile << "  (" << data.parameters.start_hour + t / data.parameters.h
                << ":" << std::setw(2) << std::setfill('0')
                << (t % data.parameters.h) * (60 / data.parameters.h) << "), "
                << "t = " << t << ": " << flow.num_of_scheduled_departures[t]
                << "\n"
                << std::setfill(' '); // Reset fill character to default
      }
    }
    logFile << "\n\n";
  }

  // stampa domanda di traffico per ogni airblock
  logFile << "\nDomanda di traffico per ogni airblock:\n";
  for (int t = 0; t < data.grid.getNTimeslots(); t++) {
    logFile << "  Domanda degli airblock al tempo t: " << t << ":\n";
    for (int r = 0; r < rows; ++r) {
      for (int col = 0; col < cols; ++col) {
        int abIndex = r * cols + col;
        logFile << std::setw(3) << data.aux.q_t_a[t][abIndex];
      }
      logFile << "\n";
    }
    logFile << "\n";
  }
  logFile << "\n";
}

void Logger::logData(DACData &data) {
  logFile << "\n### DATI ###\n";

  logFile << "Dimensioni griglia: " << data.grid.getNRows() << "x"
          << data.grid.getNCols() << "\n";
  logFile << "Numero di airblock: " << data.airblocks.size() << "\n";
  logFile << "Numero di configurazioni: " << data.configurations.size() << "\n";
  logFile << "Numero di settori: " << data.sectors.size() << "\n";
  logFile << "Numero di traiettorie: " << data.flows.size() << "\n";
  logFile << "Numero totale di voli: " << data.aux.totalFlights << "\n";
  logFile << "Ora di inizio: " << data.parameters.start_hour << ":00\n";
  logFile << "Ora di fine: "
          << (data.parameters.start_hour +
              data.grid.getNTimeslots() / data.parameters.h)
          << ":00\n";
  logFile << "Intervallo di discretizzazione temporale: "
          << 60 / data.parameters.h << " minuti\n";
  logFile << "Numero di slot temporali: " << data.grid.getNTimeslots() << "\n";
  logFile << "Tolleranza per convergenza: " << data.parameters.epsilon << "\n";
  logFile
      << "Minimo numero di slot temporali in cui mantenere una configurazione: "
      << data.parameters.t_p << "\n";

  logFile << "\n";
  printGrids(data);

  logFile << "Domanda di traffico, capacità e eccesso per ogni settore:\n";
  for (int t = 0; t < data.grid.getNTimeslots(); t++) {
    // stampa domanda di traffico, capacità e eccesso per ogni settore
    logFile << "  Timeslot " << t << ":\n";
    for (Sector &sector : data.sectors) {
      int s = sector.getIndex();
      logFile << "    Settore " << s << ", t = " << t
              << ": domanda = " << data.aux.Q_t_s[t][s]
              << ", capacità = " << sector.getCapacity(t)
              << ", eccesso = " << data.aux.e_t_s[t][s] << "\n";
    }
  }

  logFile << "\n\n";

  logFile << "Domanda di traffico totale e eccesso per ogni configurazione:\n";
  // per ogni configurazione, stampa domanda di traffico e eccesso
  for (int t = 0; t < data.grid.getNTimeslots(); t++) {
    logFile << "  Timeslot " << t << ":\n";
    for (Configuration &config : data.configurations) {
      int domanda = 0;
      for (Sector *s : config.sectors) {
        domanda += data.aux.Q_t_s[t][s->getIndex()];
      }
      logFile << "    Configurazione " << config.id << ", t = " << t
              << ": domanda = " << domanda
              << ", eccesso = " << data.aux.E_t_c[t][config.id] << "\n";
    }
  }

  logFile << "\n";

  logFile << "Eccesso totale di ogni configurazione:\n";
  for (Configuration &config : data.configurations) {
    int totalExcess = 0;
    logFile << "  Configurazione " << config.id << ": ";
    for (int t = 0; t < data.grid.getNTimeslots(); t++) {
      totalExcess += data.aux.E_t_c[t][config.id];
    }
    logFile << totalExcess << "\n";
  }

  logFile << "\n";
}

Logger::~Logger() {
  if (logFile.is_open()) {
    logFile.close();
  }
}

double Logger::getElapsedTime() {
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = endTime - startTime;
  return std::round(elapsed.count() * 100.0) / 100.0;
}

void Logger::writeIterationsSummaryHeader(
    const std::string &filename, int rows, int cols, int nHours, int startHour,
    int minSectorLength, int nConfigs, int nFlows, int nFlights, int h, int t_p,
    double epsilon, double muDeparturesDist, int dataStartHour,
    int timeHorizonMin, int timeHorizonMax, int seed) {
  std::ofstream out(filename);
  if (!out.is_open()) {
    throw std::runtime_error("Impossibile aprire il file risultati: " +
                             filename);
  }

  out << "### RISULTATI ITERAZIONI ###\n\n";
  out << "Dati generali CSV generato:\n";
  out << "- rows: " << rows << "\n";
  out << "- cols: " << cols << "\n";
  out << "- nhours: " << nHours << "\n";
  out << "- starthour: " << startHour << "\n";
  out << "- min_sector_length: " << minSectorLength << "\n";
  out << "- nconfigs: " << nConfigs << "\n";
  out << "- nflows: " << nFlows << "\n";
  out << "- nflights: " << nFlights << "\n";
  out << "- h (slot/ora): " << h << "\n";
  out << "- t_p: " << t_p << "\n";
  out << "- epsilon: " << epsilon << "\n";
  out << "- seed: " << seed << "\n\n";

  out << "Intervallo di tempo di DACData:\n";
  out << "- data_starthour: " << dataStartHour << "\n";
  out << "- data_timehorizon range: [" << timeHorizonMin << ", "
      << timeHorizonMax << "]\n\n";

  out << std::left << std::setw(12) << "seed" << std::setw(16) << "pUpperBound"
      << std::setw(18) << "dataTimeHorizon" << std::setw(12) << "iterations"
      << std::setw(14) << "elapsed_sec" << std::setw(12) << "timeslots"
      << std::setw(14) << "totalFlights" << std::setw(16) << "finalTheta"
      << std::setw(20) << "finalSlaveExcess" << std::setw(12) << "converged"
      << std::setw(10) << "optimal" << "\n";
}

void Logger::appendIterationSummaryRow(const std::string &filename,
                                       const IterationSummaryRow &row) {
  std::ofstream out(filename, std::ios::app);
  if (!out.is_open()) {
    throw std::runtime_error("Impossibile aprire il file risultati: " +
                             filename);
  }

  out << std::left << std::setw(12) << row.seed << std::setw(16)
      << std::boolalpha << row.pUpperBound << std::setw(18)
      << row.dataTimeHorizon << std::setw(12) << row.iterations << std::setw(14)
      << std::fixed << std::setprecision(2) << row.elapsedSeconds
      << std::setw(12) << row.timeslots << std::setw(14) << row.totalFlights
      << std::setw(16) << row.finalTheta << std::setw(20)
      << row.finalSlaveExcess << std::setw(12) << std::boolalpha
      << row.converged << std::setw(10) << row.optimal << "\n";
}

void Logger::info(const std::string &message) {
  logFile << message << std::endl;
}