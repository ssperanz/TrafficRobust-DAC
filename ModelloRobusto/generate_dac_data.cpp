#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Matrix.h"
#include "generate_dac_data.h"

void AirBlock::setglevel(int t, int g) { gLevel[t] = g; }

void AirBlock::setplevel(int t, int p) { pLevel[t] = p; }

int AirBlock::getIndex() { return index; }

int AirBlock::getplevel(int t) { return pLevel[t]; }

int AirBlock::getglevel(int t) { return gLevel[t]; }

// versione con timeslot

//  void AirBlock::setTraffic(TimeSlot& time, double traff){
//     traffic[time.id]=traff;
// }

void AirBlock::setTraffic(int time, double traff) { traffic[time] = traff; }

double AirBlock::getTraffic(int t) { return traffic[t]; }

std::vector<Flow *> AirBlock::getFlows() { return flows; }

void AirBlock::addFlow(Flow *f) { flows.push_back(f); }

// SECTOR

double Sector::getTraffic(int t) {
  double sum = 0.0;
  for (int a = 0; a < airblocks.size(); a++) {
    sum += airblocks[a]->getTraffic(t);
  }
  return sum;
}

double Sector::getCapacity(int t) { return capacity[t]; }

int Sector::getNAirblocks() { return airblocks.size(); }

void Sector::setCapacity(std::vector<double> cap) { capacity = cap; }

int Sector::getExcess(int t) {
  double total = 0.0;
  for (int a = 0; a < airblocks.size(); a++) {
    total += airblocks[a]->getTraffic(t);
  }
  return std::max(0.0, total - capacity[t]);
}

int Sector::getIndex() { return index; }

//  CONFIGURATION

int Configuration::getExcess(int t) {
  double total = 0.0;
  for (int s = 0; s < sectors.size(); s++) {
    total += sectors[s]->getExcess(t);
  }
  return total;
}

bool Configuration::checkSimilarity(Configuration &c2) {
  int intersection = 0;
  for (int s1 = 0; s1 < sectors.size(); s1++) {
    for (int s2 = 0; s2 < c2.sectors.size(); s2++) {
      if (sectors[s1]->getIndex() == c2.sectors[s2]->getIndex()) {
        intersection++;
      }
    }
  }
  int setUnion = sectors.size() + c2.sectors.size() - intersection;
  if (setUnion > 0) {
    double result = static_cast<double>(intersection) / setUnion;
    if (result >= 0.50) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

// FLOW

int Flow::getId() { return id; }

std::vector<AirBlock *> Flow::getAirBlocks() { return airblocks; }

std::vector<Flow *> Flow::getFlowsThroughAirBlock(std::vector<Flow *> flows,
                                                  AirBlock *ab) {
  std::vector<Flow *> flows_through_ab;
  for (int i = 0; i < flows.size(); i++) {
    Flow *flow = flows[i];
    for (int j = 0; j < flow->getAirBlocks().size(); j++) {
      if (flow->getAirBlocks()[j] == ab) {
        flows_through_ab.push_back(flow);
        break;
      }
    }
  }
  return flows_through_ab;
}

int Flow::getTimeToReachAirBlock(int ab_index) {
  if (ab_index < 0 || ab_index >= time_toreach_airblock.size()) {
    throw std::out_of_range("Invalid airblock index");
  }
  return time_toreach_airblock[ab_index];
}

int Flow::getTimeThroughAirBlock(int ab_index) {
  if (ab_index < 0 || ab_index >= time_through_airblock.size()) {
    throw std::out_of_range("Invalid airblock index");
  }
  return time_through_airblock[ab_index];
}

int Flow::getScheduledDepartures(int time) {
  if (time < 0 || time >= scheduled_departures.size()) {
    throw std::out_of_range("Invalid time index");
  }
  return scheduled_departures[time];
}

int Flow::tao_f_a(int t, int ab_index) {
  return std::max(0, t - getTimeToReachAirBlock(ab_index) -
                         getTimeThroughAirBlock(ab_index) + 1);
}

/*int Flow::addScheduledDepartures(int time, int departures) {
  if (time < 0 || time >= scheduled_departures.size()) {
    throw std::out_of_range("Invalid time index");
  }
  scheduled_departures[time] += departures;
  return scheduled_departures[time];
}*/

void Flow::setScheduledDepartures(std::vector<int> departures) {
  scheduled_departures = departures;
}

// GRID

int Grid::getNRows() const { return m_rows; }

int Grid::getNCols() const { return m_cols; }

AirBlock &Grid::airBlock(int id) { return m_airBlocks[id]; }

AirBlock &Grid::airBlock(int row, int col) {
  return m_airBlocks[row * m_cols + col];
}

int Grid::getNTimeslots() { return m_time.size(); }

int Grid::get_t_p() { return t_p; }

Grid::Grid(int rows, int cols, const TimeHorizon &time, int t_p)
    : m_rows(rows), m_cols(cols), m_time(time), t_p(t_p) {
  int id = 0;

  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      m_airBlocks.emplace_back(id++, r, c, time.size());
}

Grid DACData::readGrid(const std::string &filename) {
  // SEZIONE DA MODIFICARE deve leggere quanto dura lo slot di tempo e quanti
  // slot sono da file
  std::ifstream in(filename);

  if (!in)
    throw std::runtime_error("Cannot open file.");

  std::string line;
  std::string section;

  TimeHorizon time(0, 0);
  time.slots.clear();

  int rows = 0;
  int cols = 0;
  int h = 0;

  while (std::getline(in, line)) {
    line = trim(line);

    if (line.empty())
      continue;

    if (line[0] == '[') {
      section = line;
      std::getline(in, line); // salta intestazione
      continue;
    }

    else if (section == "[TIME]") {
      std::cout << "Lettura TIME" << std::endl;
      auto f = split(line, ',');

      int id = stoi(f[0]);

      // Converte HH:MM in minuti dalla mezzanotte
      auto parseTime = [](const std::string &time) -> int {
        auto parts = split(time, ':');

        int hour = stoi(parts[0]);
        int minute = stoi(parts[1]);

        return hour * 60 + minute;
      };

      int start = parseTime(f[1]);
      int end = parseTime(f[2]);

      time.slots.emplace_back(id, start, end);
      std::cout << "Slot temporale: " << id << " (" << start << " - " << end
                << ")" << std::endl;
    } else if (section == "[GRID]") {
      std::cout << "Lettura GRID" << std::endl;
      auto f = split(line, ',');

      rows = std::stoi(f[0]);
      cols = std::stoi(f[1]);
      h = std::stoi(f[2]);
    }
  }

  return Grid(rows, cols, time, h);
}

int DACData::getNConfigurations() { return configurations.size(); }

std::vector<int> DACData::getSimilarConfigurationsIndex(Configuration &c1) {
  // algoritmo: rapporto tra settori in comune e insieme dei settori della
  // configurazione c e c'
  std::vector<int> similarIndex;
  for (int c2 = 0; c2 < configurations.size(); c2++) {
    if (c1.checkSimilarity(configurations[c2])) {
      similarIndex.push_back(c2);
    }
  }
  return similarIndex;
}

DACData::DACData(const std::string &filename)
    : grid(readGrid(filename)), epsilon(1e-4) {
  std::cout << "finito lettura griglia" << std::endl;
  readRest(filename);
}
std::string DACData::trim(std::string s) {
  while (!s.empty() && isspace(s.front()))
    s.erase(s.begin());

  while (!s.empty() && isspace(s.back()))
    s.pop_back();

  if (!s.empty() && s.front() == '"')
    s.erase(s.begin());

  if (!s.empty() && s.back() == '"')
    s.pop_back();

  return s;
}

std::vector<std::string> DACData::split(const std::string &s, char sep) {
  std::vector<std::string> out;
  std::string token;
  std::stringstream ss(s);
  std::vector<std::vector<double>> capacities;

  while (getline(ss, token, sep))
    out.push_back(trim(token));

  return out;
}

void DACData::readRest(const std::string &filename) {
  std::ifstream in(filename);

  if (!in)
    throw std::runtime_error("Cannot open file.");

  //--------------------------------------------------
  // Variabili temporanee
  //--------------------------------------------------

  std::string line;
  std::string section;

  Matrix<double> *capPtr = NULL;
  Matrix<int> *departuresPtr = NULL;

  while (getline(in, line)) {
    line = trim(line);

    if (line.empty())
      continue;

    if (line[0] == '[') {
      section = line;
      getline(in, line); // salta intestazione
      continue;
    }

    //--------------------------------------------------
    // AIRBLOCK DATA
    //--------------------------------------------------

    else if (section == "[AIRBLOCKS]") {
      auto f = split(line, ',');

      int block = stoi(f[0]);
      int t = stoi(f[1]);
      double tr = stod(f[2]);
      int p = stoi(f[3]);
      int g = stoi(f[4]);

      AirBlock &b = grid.airBlock(block);

      b.setTraffic(t, tr);
      b.setplevel(t, p);
      b.setglevel(t, g);
    }

    //--------------------------------------------------
    // SECTORS
    //--------------------------------------------------

    else if (section == "[SECTORS]") {
      auto f = split(line, ',');

      int id = stoi(f[0]);

      std::vector<AirBlock *> blocks;

      std::vector<std::string> airIds = split(f[1], ';');

      for (std::string &s : airIds) {
        int blockId = stoi(s);

        AirBlock *b = &grid.airBlock(blockId);

        blocks.push_back(b);
      }

      sectors.push_back(Sector(id, blocks));
    }

    else if (section == "[CAPACITY]") {
      auto f = split(line, ',');

      int sectorId = stoi(f[0]);
      int t = stoi(f[1]);
      double cap = stod(f[2]);
      if (capPtr == NULL) {
        capPtr = new Matrix<double>(sectors.size(), grid.getNTimeslots());
      }
      capPtr->set(sectorId, t, cap);
    }

    //--------------------------------------------------
    // CONFIGURATIONS
    //--------------------------------------------------

    else if (section == "[CONFIGURATIONS]") {
      auto f = split(line, ',');

      int id = stoi(f[0]);

      std::vector<Sector *> configSectors;

      std::vector<std::string> secIds = split(f[1], ';');

      for (std::string &s : secIds) {
        int sectorId = stoi(s);

        Sector *b = &(sectors[sectorId]);

        configSectors.push_back(b);
      }
      configurations.push_back(Configuration(id, configSectors));
    }

    else if (section == "[TRAJECTORIES]") {
      auto f = split(line, ',');

      int id = stoi(f[0]);

      std::vector<AirBlock *> flowAirBlocks;

      std::vector<std::string> abIds = split(f[1], ';');

      for (std::string &s : abIds) {
        int abId = stoi(s);

        AirBlock *ab = &(grid.airBlock(abId));

        flowAirBlocks.push_back(ab);
      }

      std::vector<int> timeToReach;
      std::vector<int> timeThrough;
      std::vector<int> scheduledDepartures;

      std::vector<std::string> ttrValues = split(f[2], ';');
      for (std::string &s : ttrValues) {
        timeToReach.push_back(stoi(s));
      }

      std::vector<std::string> ttValues = split(f[3], ';');
      for (std::string &s : ttValues) {
        timeThrough.push_back(stoi(s));
      }

      flows.push_back(Flow(id, flowAirBlocks, timeToReach, timeThrough,
                           scheduledDepartures));

      for (AirBlock *ab : flowAirBlocks) {
        ab->addFlow(&flows.back());
      }
    }

    else if (section == "[DEPARTURES]") {
      auto f = split(line, ',');

      int flowId = stoi(f[0]);
      int time = stoi(f[1]);
      int departures = stoi(f[2]);

      if (departuresPtr == NULL) {
        departuresPtr = new Matrix<int>(flows.size(), grid.getNTimeslots());
      }
      departuresPtr->set(flowId, time, departures);
    }
  }
  for (size_t i = 0; i < sectors.size(); ++i) {
    sectors[i].setCapacity(capPtr->getLine(i));
  }
  for (size_t i = 0; i < flows.size(); ++i) {
    flows[i].setScheduledDepartures(departuresPtr->getLine(i));
  }
}
