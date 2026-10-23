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

std::vector<int> AirBlock::getFlows() { return flows; }
int AirBlock::getFlow(int f) { return flows.at(f); }
void AirBlock::addFlow(int flowId) { flows.push_back(flowId); }

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

int Sector::getIndex() { return index; }

//  CONFIGURATION

int Configuration::getExcess(int t) {
  double total = 0.0;
  for (int s = 0; s < sectors.size(); s++) {
    // total += sectors[s]->getExcess(t);
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
      if (flow->getAirBlocks().at(j)->getIndex() == ab->getIndex()) {
        flows_through_ab.push_back(flow);
        break;
      }
    }
  }
  return flows_through_ab;
}

int Flow::getTimeToReachAirBlock(int ab_id) {
  auto it = time_toreach_airblock.find(ab_id);

  if (it != time_toreach_airblock.end()) {
    return it->second;
  }

  throw std::out_of_range("AirBlock ID not found in time_toreach_airblock map");
}

int Flow::getTimeThroughAirBlock(int ab_id) {
  auto it = time_through_airblock.find(ab_id);

  if (it != time_through_airblock.end()) {
    return it->second;
  }

  throw std::out_of_range("AirBlock ID not found in time_through_airblock map");
}

int Flow::getScheduledDepartures(int time) {
  if (time < 0 || time >= num_of_scheduled_departures.size()) {
    throw std::out_of_range("Invalid time index");
  }
  return num_of_scheduled_departures[time];
}

int Flow::tau_f_a(int t, int ab_id) {
  return std::max(0, t - getTimeToReachAirBlock(ab_id) -
                         getTimeThroughAirBlock(ab_id) + 1);
}

void Flow::setScheduledDepartures(std::vector<int> departures) {
  num_of_scheduled_departures = departures;
}

int Flow::findAbPositionInFlow(int ab_id) {
  for (int i = 0; i < airblocks.size(); ++i) {
    if (airblocks[i]->getIndex() == ab_id) {
      return i;
    }
  }
  throw std::out_of_range("AirBlock ID not found in flow");
}

// GRID

int Grid::getNRows() const { return m_rows; }

int Grid::getNCols() const { return m_cols; }

AirBlock &Grid::airBlock(int id) { return m_airBlocks[id]; }

AirBlock &Grid::airBlock(int row, int col) {
  return m_airBlocks[row * m_cols + col];
}

int Grid::getNTimeslots() { return m_time.size(); }

Grid::Grid(int rows, int cols, const TimeHorizon &time)
    : m_rows(rows), m_cols(cols), m_time(time) {
  int id = 0;

  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      m_airBlocks.emplace_back(id++, r, c, time.size());
}

Parameters DACData::readParameters(const std::string &filename) {
  std::ifstream in(filename);

  if (!in)
    throw std::runtime_error("Cannot open file.");

  std::string line;
  std::string section;

  int h = 0;
  int t_p = 0;
  double epsilon = 0.0;

  while (std::getline(in, line)) {
    line = trim(line);

    if (line.empty())
      continue;

    if (line[0] == '[') {
      section = line;
      std::getline(in, line); // salta intestazione
      continue;
    }

    else if (section == "[PARAMETERS]") {
      auto f = split(line, ',');

      h = stoi(f[0]);
      t_p = stoi(f[1]);
      epsilon = stod(f[2]);
    }
  }

  return Parameters(h, t_p, epsilon);
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

  return Grid(rows, cols, time);
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

void DACData::calculateAuxData() {
  int T = grid.getNTimeslots();
  int F = flows.size();
  int maxAbinFlow =
      3 *
      sqrt(
          grid.getNCols() *
          grid.getNRows()); // stima del numero massimo di airblock in un flusso

  aux.tau_f_a.resize(
      T, std::vector<std::vector<int>>(F, std::vector<int>(maxAbinFlow, 0)));
  aux.BigM.resize(T, 0);

  for (int t = 0; t < T; t++) {
    for (int f = 0; f < F; f++) {
      Flow &flow = flows[f];
      std::vector<AirBlock *> airblocksInFlow = flow.getAirBlocks();
      for (int abIdxPosition = 0; abIdxPosition < airblocksInFlow.size();
           abIdxPosition++) {
        AirBlock *ab = airblocksInFlow[abIdxPosition];
        aux.tau_f_a[t][f][abIdxPosition] = flow.tau_f_a(t, ab->getIndex());
      }
      if (t == 0) {
        for (int tau = 0; tau <= t + parameters.h - 1; tau++) {
          aux.BigM[t] += flow.getScheduledDepartures(tau);
        }
      } else if (t + parameters.h < T) {
        aux.BigM[t] =
            aux.BigM[t - 1] + flow.getScheduledDepartures(t + parameters.h);
      } else {
        aux.BigM[t] = aux.BigM[t - 1];
      }
    }
  }
}

DACData::DACData(const std::string &filename)
    : parameters(readParameters(filename)), grid(readGrid(filename)) {
  std::cout << "finito lettura griglia" << std::endl;
  readRest(filename);
  std::cout << "finito lettura file csv" << std::endl;
  calculateAuxData();
  std::cout << "finito calcolo dati ausiliari" << std::endl;
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

      if (airblocks.size() == 0 ||
          (airblocks.back().getIndex() != b.getIndex()))
        airblocks.push_back(b);
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
      int flowId = stoi(f[0]);
      int airblockId = stoi(f[1]);
      AirBlock *ab = &airblocks.at(airblockId);
      int timeThrough = stoi(f[2]);
      if (flows.empty() || flows.back().getId() != flowId) {
        flows.push_back(Flow(flowId, ab, 0, timeThrough));
      }

      else if (flows.back().getId() == flowId) {
        int prevAbId = flows.back().getAirBlocks().back()->getIndex();
        flows.back().airblocks.push_back(ab);
        flows.back().time_toreach_airblock.insert(
            {ab->getIndex(),
             timeThrough + flows.back().time_toreach_airblock.at(
                               prevAbId)}); // time to reach the new airblock
                                            // is the sum of the time to reach
                                            // the previous airblock and the
                                            // time through the new airblock
        flows.back().time_through_airblock.insert(
            {ab->getIndex(), timeThrough});
      }

      else {
        throw std::runtime_error(
            "Unexpected flow ID in [TRAJECTORIES] section");
      }

      for (auto &ab : airblocks) {
        if (ab.getIndex() == airblockId) {
          ab.addFlow(flowId);
          break;
        }
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
