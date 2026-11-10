#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

enum TrajectoryShape { STRAIGHT, CURVED_1, CURVED_2 };

class DACCSVGenerator {
public:
  DACCSVGenerator(int ROWS = 20, int COLS = 20, int NHOURS = 24,
                  int STARTHOUR = 0, int sector_length = 5, int NCONFIGS = 20,
                  int NFLOWS = 50, int NTIMESINONEHOUR = 1,
                  int MINCONFKEEPTIME = 5, double EPSILON = 1e-4,
                  double MU_DEPARTURES_DIST = 0.18);
  void generateDACCSV(const std::string &filename, unsigned int seed);
  double hourlyFactor(int t);
  void generateSingleTrajectory(std::vector<int> &trajectory, int start,
                                int end, int ROWS, int COLS,
                                std::vector<std::vector<int>> &allTrajectories,
                                std::mt19937 &rng,
                                std::uniform_real_distribution<double> &noise,
                                TrajectoryShape shape = STRAIGHT);

  std::vector<std::vector<int>>
  generateAllTrajectories(std::mt19937 &rng,
                          std::vector<int> &airportPositions);
  double calculateAirblockDistance(int COLS, int ab1, int ab2);
  std::vector<int> calculateSectorAirblocks(int x1, int y1, int x2, int y2,
                                            int ROWS, int COLS);
  void
  partition(int x1, int y1, int x2, int y2, int minSectorLength, int maxSectors,
            std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
                &sectorsCoordinates,
            std::mt19937 &rng);
  void validateSectors(const std::vector<std::vector<int>> &sectors, int ROWS,
                       int COLS);

private:
  int ROWS;
  int COLS;
  int NTIMES;
  int STARTHOUR;
  int NSECTORS;
  int NCONFIGS;
  int MINSECTORLENGTH;
  int NFLOWS;
  double MU_DEPARTURES_DIST;
  int MINCONFKEEPTIME;
  int NTIMESINONEHOUR;
  double EPSILON;
  void
  generateStraightTrajectory(std::vector<int> &trajectory, int start, int end,
                             int ROWS, int COLS, std::mt19937 &rng,
                             std::uniform_real_distribution<double> &noise);
};