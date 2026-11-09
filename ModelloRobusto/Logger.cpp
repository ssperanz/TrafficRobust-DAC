#include "Logger.h"
#include <iomanip>

Logger::Logger(const std::string &filename) : logFile(filename) {
  if (!logFile.is_open()) {
    throw std::runtime_error("Impossibile aprire il file di log: " + filename);
  }
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
        logFile << "t = " << t << ": " << flow.num_of_scheduled_departures[t]
                << "\n";
      }
    }
    logFile << "\n\n";
  }

  // stampa domanda di traffico per ogni airblock
  logFile << "\nDomanda di traffico per ogni airblock:\n";
  for (int t = 0; t < data.grid.getNTimeslots(); t++) {
    logFile << "Domanda degli airblock al tempo t: " << t << ":\n";
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

  logFile << "Dimensioni griglia: " << data.grid.getNRows() << "x"
          << data.grid.getNCols() << "\n";
  logFile << "Numero di airblock: " << data.airblocks.size() << "\n";
  logFile << "Numero di configurazioni: " << data.configurations.size() << "\n";
  logFile << "Numero di settori: " << data.sectors.size() << "\n";
  logFile << "Numero di traiettorie: " << data.flows.size() << "\n";
  logFile << "Numero totale di voli: " << data.aux.totalFlights << "\n";
  logFile << "Numero di slot temporali: " << data.grid.getNTimeslots() << "\n";
  logFile << "Intervallo di discretizzazione temporale: "
          << 60 / data.parameters.h << " minuti\n";
  logFile << "Tolleranza per convergenza: " << data.parameters.epsilon << "\n";
  logFile
      << "Minimo numero di slot temporali per mantenere una configurazione: "
      << data.parameters.t_p << "\n";

  logFile << "\n";
  printGrids(data);

  logFile << "Domanda di traffico, capacità e eccesso per ogni settore:\n";
  for (int t = 0; t < data.grid.getNTimeslots(); t++) {
    // stampa domanda di traffico, capacità e eccesso per ogni settore
    logFile << "Timeslot " << t << ":\n";
    for (Sector &sector : data.sectors) {
      int s = sector.getIndex();
      logFile << "  Settore " << s << ", t = " << t
              << ": domanda = " << data.aux.Q_t_s[t][s]
              << ", capacità = " << sector.getCapacity(t)
              << ", eccesso = " << data.aux.e_t_s[t][s] << "\n";
    }
  }

  logFile << "\n";

  logFile << "Domanda di traffico totale e eccesso per ogni configurazione:\n";
  // per ogni configurazione, stampa domanda di traffico e eccesso
  for (int t = 0; t < data.grid.getNTimeslots(); t++) {
    logFile << "Timeslot " << t << ":\n";
    for (Configuration &config : data.configurations) {
      int domanda = 0;
      for (Sector *s : config.sectors) {
        domanda += data.aux.Q_t_s[t][s->getIndex()];
      }
      logFile << "  Configurazione " << config.id << ", t = " << t
              << ": domanda = " << domanda
              << ", eccesso = " << data.aux.E_t_c[t][config.id] << "\n";
    }
  }

  logFile << "\n";
}

Logger::~Logger() {
  if (logFile.is_open()) {
    logFile.close();
  }
}
