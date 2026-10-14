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
  DACCSVGenerator(int ROWS = 20, int COLS = 20, int NTIMES = 15,
                  int NSECTORS = 100, int NCONFIGS = 20);
  void generateDACCSV(const std::string &filename, unsigned int seed);
  double hourlyFactor(int t);
  std::vector<int> generateTrajectory(int ROWS, int COLS, int TRAJ_LEN_MIN,
                                      int TRAJ_LEN_MAX, std::mt19937 &rng);

private:
  int ROWS;
  int COLS;
  int NTIMES;
  int NSECTORS;
  int NCONFIGS;
};