#include "DACCSVGenerator.h"

DACCSVGenerator::DACCSVGenerator(int rows, int cols, int ntimes, int nsectors,
                                 int nconfigs, int h, int t_p, double epsilon)
    : ROWS(rows), COLS(cols), NTIMES(ntimes), NSECTORS(nsectors),
      NCONFIGS(nconfigs), NTIMESINONEHOUR(h), MINCONFKEEPTIME(t_p),
      EPSILON(epsilon) {}

double DACCSVGenerator::hourlyFactor(int t) {
  // Picchi di traffico
  if (t == 0)
    return 1.8; // 6-7
  if (t == 6)
    return 2.0; // 12-13
  if (t == 13)
    return 1.7; // 19-20

  return 0.7 + 0.4 * std::sin(3.14159265 * t / 15.0);
};

std::vector<std::vector<int>>
DACCSVGenerator::generateAllTrajectories(std::mt19937 &rng) {
  std::uniform_int_distribution<int> startDist(0, ROWS * COLS - 1);
  // std::uniform_int_distribution<int> lengthDist(TRAJ_LEN_MIN, TRAJ_LEN_MAX);
  std::uniform_real_distribution<double> noise(-0.5, 0.5);
  std::uniform_int_distribution<int> airportDist(0, ROWS * COLS - 1);

  int nAirports = ROWS * COLS / 10;
  int AirportPositions[nAirports]; // airblock id of airports
  for (int i = 0; i < nAirports; i++) {
    AirportPositions[i] = airportDist(rng);
  }

  /* Trajectories for: Airport A to Airbort B; Airport to grid border
   * and viceversa;  From border to border */
  int percBorderToBorder = 10;
  int percAirportToAirport = 60;
  int percAirportToBorder = 100 - percBorderToBorder - percAirportToAirport;
  int nFlights = 1000;
  int nFlows = nFlights / 20; // estimate, shoud be adjusted

  std::vector<std::vector<int>> allTrajectories;
  std::uniform_int_distribution<int> airportChoiceDist(0, nAirports - 1);

  /* 1. Trajectories from airport A to airport B */
  for (int i = 0; i < nFlows * percAirportToAirport / 100; i++) {
    int start = AirportPositions[airportChoiceDist(rng)];
    int end = AirportPositions[airportChoiceDist(rng)];
    while (end == start) {
      end = AirportPositions[airportChoiceDist(rng)];
    }

    std::vector<int> trajectory;
    trajectory.push_back(start);

    while (trajectory.back() != end) {
      int last = trajectory.back();
      int row = last / COLS;
      int col = last % COLS;

      double dr =
          std::clamp(static_cast<double>((end / COLS) - row), -1.0, 1.0) +
          noise(rng);
      double dc =
          std::clamp(static_cast<double>((end % COLS) - col), -1.0, 1.0) +
          noise(rng);

      int newRow = std::clamp(static_cast<int>(row + dr), 0, ROWS - 1);
      int newCol = std::clamp(static_cast<int>(col + dc), 0, COLS - 1);

      int newBlock = newRow * COLS + newCol;
      if (newBlock != trajectory.back())
        trajectory.push_back(newBlock);
    }

    allTrajectories.push_back(trajectory);
  }

  /* 2. Trajectories from airport to border and viceversa */
  for (int i = 0; i < nFlows * percAirportToBorder / 100; i++) {
    int start = AirportPositions[rng() % nAirports];
    int end;
    if (rng() % 2 == 0) {
      // Airport to border
      int borderRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
      int borderCol = rng() % COLS;
      end = borderRow * COLS + borderCol;
    } else {
      // Border to airport
      int borderRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
      int borderCol = rng() % COLS;
      end = borderRow * COLS + borderCol;
    }

    std::vector<int> trajectory;
    trajectory.push_back(start);

    while (trajectory.back() != end) {
      int last = trajectory.back();
      int row = last / COLS;
      int col = last % COLS;

      double dr =
          std::clamp(static_cast<double>((end / COLS) - row), -1.0, 1.0) +
          noise(rng);
      double dc =
          std::clamp(static_cast<double>((end % COLS) - col), -1.0, 1.0) +
          noise(rng);

      int newRow = std::clamp(static_cast<int>(row + dr), 0, ROWS - 1);
      int newCol = std::clamp(static_cast<int>(col + dc), 0, COLS - 1);

      int newBlock = newRow * COLS + newCol;
      if (newBlock != trajectory.back())
        trajectory.push_back(newBlock);
    }

    allTrajectories.push_back(trajectory);
  }

  /* 3. Trajectories from border to border */
  for (int i = 0; i < nFlows * percBorderToBorder / 100; i++) {
    int startRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
    int startCol = rng() % COLS;
    int start = startRow * COLS + startCol;

    int endRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
    int endCol = rng() % COLS;
    int end = endRow * COLS + endCol;

    std::vector<int> trajectory;
    trajectory.push_back(start);

    while (trajectory.back() != end) {
      int last = trajectory.back();
      int row = last / COLS;
      int col = last % COLS;

      double dr =
          std::clamp(static_cast<double>((end / COLS) - row), -1.0, 1.0) +
          noise(rng);
      double dc =
          std::clamp(static_cast<double>((end % COLS) - col), -1.0, 1.0) +
          noise(rng);

      int newRow = std::clamp(static_cast<int>(row + dr), 0, ROWS - 1);
      int newCol = std::clamp(static_cast<int>(col + dc), 0, COLS - 1);

      int newBlock = newRow * COLS + newCol;
      if (newBlock != trajectory.back())
        trajectory.push_back(newBlock);
    }

    allTrajectories.push_back(trajectory);
  }

  return allTrajectories;
}

void DACCSVGenerator::generateDACCSV(const std::string &filename,
                                     unsigned int seed) {
  std::mt19937 rng(seed);

  std::uniform_int_distribution<int> blockDist(0, ROWS * COLS - 1);
  std::uniform_int_distribution<int> sectorSizeDist(2, 15);
  std::uniform_real_distribution<double> capacityDist(15.0, 25.0);
  std::normal_distribution<double> capacityNoise(1.0, 0.05);
  std::normal_distribution<double> noise(1.0, 0.15);

  std::normal_distribution<double> ingressDist(0.6, 0.9);
  std::normal_distribution<double> trafficVolNoise(0.8, 1.10);

  std::ofstream out(filename);

  //-----------------------------
  // TIME
  //-----------------------------
  out << "[TIME]\n";
  out << "id,start,end\n";

  for (int t = 0; t < NTIMES; t++) {
    int startHour = 6 + t;
    int endHour = 7 + t;

    out << t << "," << startHour << ":00," << endHour << ":00\n";
  }

  out << "\n";

  //-----------------------------
  // PARAMETERS
  //-----------------------------
  out << "[PARAMETERS]\n";
  out << "h,t_p,epsilon\n";
  out << NTIMESINONEHOUR << "," << MINCONFKEEPTIME << "," << EPSILON
      << "\n\n"; // h, t_p, epsilon

  //-----------------------------
  // GRID
  //-----------------------------
  out << "[GRID]\n";
  out << "rows,cols,h\n";
  out << ROWS << "," << COLS << "," << 2 << "\n\n";

  //-----------------------------
  // SECTORS
  //-----------------------------
  out << "[SECTORS]\n";
  out << "id,airblocks\n";

  std::vector<std::vector<int>> sectorBlocks;

  for (int s = 0; s < NSECTORS; s++) {
    int size = sectorSizeDist(rng);

    std::set<int> blocks;

    while (blocks.size() < size)
      blocks.insert(blockDist(rng));

    sectorBlocks.emplace_back(blocks.begin(), blocks.end());

    out << s << ",\"";

    for (size_t i = 0; i < sectorBlocks.back().size(); i++) {
      out << sectorBlocks.back()[i];

      if (i + 1 < sectorBlocks.back().size())
        out << ";";
    }

    out << "\"\n";
  }

  out << "\n";

  out << "\n";

  out << "[CAPACITY]\n";
  out << "sector,time,capacity\n";

  for (int s = 0; s < NSECTORS; s++) {
    double baseCapacity = capacityDist(rng);

    for (int t = 0; t < NTIMES; t++) {
      int cap = baseCapacity * std::max(0.8, capacityNoise(rng));

      out << s << "," << t << "," << cap << "\n";
    }
  }

  out << "\n";

  //-----------------------------
  // AIRBLOCK DATA
  //-----------------------------
  out << "[AIRBLOCKS]\n";
  out << "airblock,time,traffic,p_level,g_level\n";

  std::uniform_int_distribution<int> pLevelDist(1, 3);
  std::uniform_int_distribution<int> gLevelDist(1, 3);

  for (int k = 0; k < ROWS * COLS; k++) {
    std::uniform_real_distribution<double> baseTraffic(2.0, 8.0);
    double base = baseTraffic(rng);

    for (int t = 0; t < NTIMES; t++) {
      double traffic = base * hourlyFactor(t) * std::max(0.2, noise(rng));

      int pLevel = pLevelDist(rng);
      int gLevel = gLevelDist(rng);

      out << k << "," << t << "," << traffic << "," << pLevel << "," << gLevel
          << "\n";
    }
  }

  out << "\n";

  //-----------------------------
  // CONFIGURATIONS
  //-----------------------------
  out << "[CONFIGURATIONS]\n";
  out << "id,sectors\n";

  std::uniform_int_distribution<int> nSectorConfDist(2, 15);
  std::uniform_int_distribution<int> sectorIdDist(0, NSECTORS - 1);

  std::vector<std::set<int>> configs;

  for (int c = 0; c < NCONFIGS; c++) {
    int n = nSectorConfDist(rng);

    std::set<int> conf;

    while (conf.size() < n)
      conf.insert(sectorIdDist(rng));

    configs.push_back(conf);

    out << c << ",\"";

    size_t k = 0;
    for (int s : conf) {
      out << s;
      if (++k < conf.size())
        out << ";";
    }

    out << "\"\n";
  }

  out << "\n";

  //-----------------------------
  // TRAJECTORIES //
  //-----------------------------
  out << "[TRAJECTORIES]\n";
  out << "id,airblock,time_through\n";

  std::vector<std::vector<int>> allTrajectories = generateAllTrajectories(rng);
  for (size_t trajectory = 0; trajectory < allTrajectories.size();
       trajectory++) {
    for (size_t i = 0; i < allTrajectories[trajectory].size(); i++) {
      out << trajectory << "," << allTrajectories[trajectory][i] << "," << 1
          << "\n";
    }
  }

  out << "\n";

  //-----------------------------
  // DEPARTURES //
  //-----------------------------

  out << "[DEPARTURES]\n";
  out << "id,time,scheduled_departures\n";
  for (int f = 0; f < allTrajectories.size(); f++) {
    for (int t = 0; t < NTIMES; t++) {
      double baseDepartures =
          ingressDist(rng) * hourlyFactor(t) * trafficVolNoise(rng);
      int departures = std::max(0, static_cast<int>(baseDepartures));
      out << f << "," << t << "," << departures << "\n";
    }
  }

  out.close();

  std::cout << "Istanza salvata in " << filename << std::endl;
}