#pragma once
#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

class TimeSlot {
public:
  int id;          // indice (0,1,2,...)
  int startMinute; // es. 6
  int endMinute;   // es. 7

  TimeSlot(int id, int startMinute, int endMinute)
      : id(id), startMinute(startMinute), endMinute(endMinute) {}
};

class TimeHorizon {
public:
  std::vector<TimeSlot> slots;

  TimeHorizon(int startHour, int endHour) {
    int id = 0;
    for (int h = startHour; h < endHour; ++h) {
      slots.emplace_back(id++, h, h + 1);
    }
  }

  int size() const { return slots.size(); }

  const TimeSlot &get(int t) const { return slots[t]; }
};

class Flow;

class AirBlock {
public:
  int index;
  int row;
  int col;
  std::vector<Flow *> flows; // flussi che attraversano l’airblock

  std::vector<double> traffic;
  std::vector<int> pLevel;
  std::vector<int> gLevel;

  AirBlock(int index, int r, int c, int ntimes)
      : index(index), row(r), col(c), traffic(ntimes, 0.0), pLevel(ntimes, 0),
        gLevel(ntimes, 0) {}

  double getTraffic(int t);
  // versione con timeslot
  // void setTraffic(TimeSlot& time, double traffic);
  void setTraffic(int time, double traffic);
  int getglevel(int t);
  int getplevel(int t);
  void setglevel(int t, int g);
  void setplevel(int t, int p);
  int getIndex();
  std::vector<Flow *> getFlows();
  void addFlow(Flow *f);
};

class Sector {
public:
  int index;
  std::vector<AirBlock *> airblocks;

  std::vector<double> capacity;

  Sector(int index, std::vector<AirBlock *> blocks)
      : index(index), airblocks(blocks) {}
  int getExcess(
      int t); // ritorna l'eccesso totale del settore all'intervallo temporale t
  int getIndex();
  void setCapacity(std::vector<double> cap);
  double getCapacity(int t);
  int getNAirblocks();
  double getTraffic(int t);
};

class Configuration {
public:
  int id;

  std::vector<Sector *> sectors;

  Configuration(int id, std::vector<Sector *> s) : id(id), sectors(s) {}
  bool checkSimilarity(Configuration &);

  int getExcess(int t); // ritorna l'eccesso totale della configurazione
                        // all'intervallo temporale t
};

class Flow {
private:
  int id;
  std::vector<AirBlock *> airblocks;
  std::vector<int> time_toreach_airblock; // time_toreach_airblock[i] indica il
                                          // tempo necessario per raggiungere
                                          // l'airblock i del flusso
  std::vector<int> time_through_airblock; // time_through_airblock[i] indica il
                                          // tempo necessario per attraversare
                                          // l'airblock i del flusso
  std::vector<int>
      scheduled_departures; // scheduled_departures[time] indica il numero di
                            // partenze programmate per il timeslot time

public:
  Flow(int id, std::vector<AirBlock *> ab, std::vector<int> time_to_reach,
       std::vector<int> time_through, std::vector<int> scheduled_departures)
      : id(id), airblocks(ab), time_toreach_airblock(time_to_reach),
        time_through_airblock(time_through),
        scheduled_departures(scheduled_departures) {}
  int getId();
  std::vector<AirBlock *> getAirBlocks();
  static std::vector<Flow *> getFlowsThroughAirBlock(std::vector<Flow *> flows,
                                                     AirBlock *ab);
  int getTimeToReachAirBlock(int ab_index);
  int getTimeThroughAirBlock(int ab_index);
  int tao_f_a(int t,
              int ab_index); // the earliest entry time in airspace block 'a' of
                             // aircraft in flow 'f' that are still in airspace
                             // block 'a' at time 't'.
};

class Grid {
public:
  Grid(int rows, int cols, const TimeHorizon &time, int h);
  // Grid(const std::string& filename);

  int getNRows() const;
  int getNCols() const;

  int index(int row, int col) const;
  std::pair<int, int> coordinates(int id) const;

  AirBlock &airBlock(int row, int col);
  AirBlock &airBlock(int id);

  int getNTimeslots();
  int geth();

  const std::array<double, 3> w = {
      0.10, 0.50, 0.20}; // peso di riduzione della capacit`a per un airblock
                         // colpito con gk,t = g   (parametro)
  const std::array<double, 3> pi = {
      0.0, 0.25,
      0.25}; // quota massima di airblock con pk,t = p in cui il modello di
             // incertezza ammette il verificarsi del maltempo (parametro).

private:
  int m_rows;
  int m_cols;

  std::vector<AirBlock> m_airBlocks;
  TimeHorizon m_time;

  int h; // minimo slot temporale che deve passare tra due configurazioni
         // diverse
};

class DACData {
public:
  DACData(const std::string &filename);
  Grid grid;
  std::vector<Sector> sectors;
  std::vector<Configuration> configurations;

  double epsilon; // tolleranza per la condizione di arresto

  int getNConfigurations();
  std::vector<int> getSimilarConfigurationsIndex(
      Configuration &c); // restituisce anche l'indice di c
  void readRest(const std::string &filename);
  Grid readGrid(const std::string &filename);

private:
  static std::vector<std::string> split(const std::string &s, char sep);

  static std::string trim(std::string s);
};