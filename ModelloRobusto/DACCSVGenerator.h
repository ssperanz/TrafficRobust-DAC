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

class DACCSVGenerator {
public:
  DACCSVGenerator(int ROWS = 20, int COLS = 20, int NHOURS = 24,
                  int STARTHOUR = 0, int sector_length = 5, int NCONFIGS = 20,
                  int NTIMESINONEHOUR = 1, int MINCONFKEEPTIME = 5,
                  double EPSILON = 1e-4);
  void generateDACCSV(const std::string &filename, unsigned int seed);
  double hourlyFactor(int t);
  std::vector<std::vector<int>> generateAllTrajectories(std::mt19937 &rng);
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
  int MINCONFKEEPTIME;
  int NTIMESINONEHOUR;
  double EPSILON;
};