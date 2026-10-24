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
                  int STARTHOUR = 0, int NSECTORS = 100, int NCONFIGS = 20,
                  int NTIMESINONEHOUR = 1, int MINCONFKEEPTIME = 5,
                  double EPSILON = 1e-4);
  void generateDACCSV(const std::string &filename, unsigned int seed);
  double hourlyFactor(int t);
  std::vector<std::vector<int>> generateAllTrajectories(std::mt19937 &rng);
  double calculateAirblockDistance(int COLS, int ab1, int ab2);

private:
  int ROWS;
  int COLS;
  int NTIMES;
  int STARTHOUR;
  int NSECTORS;
  int NCONFIGS;
  int MINCONFKEEPTIME;
  int NTIMESINONEHOUR;
  double EPSILON;
};