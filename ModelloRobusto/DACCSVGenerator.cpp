#include "DACCSVGenerator.h"

DACCSVGenerator::DACCSVGenerator(int rows, int cols, int nhours, int startHour,
                                 int min_sector_length, int nconfigs, int h,
                                 int t_p, double epsilon)
    : ROWS(rows), COLS(cols), NTIMES(nhours * h), STARTHOUR(startHour),
      NSECTORS(0), MINSECTORLENGTH(min_sector_length), NCONFIGS(nconfigs),
      NTIMESINONEHOUR(h), MINCONFKEEPTIME(t_p), EPSILON(epsilon) {}

// TODO: to fix considering also h and startHour
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

double DACCSVGenerator::calculateAirblockDistance(int COLS, int ab1, int ab2) {
  int row1 = ab1 / COLS;
  int col1 = ab1 % COLS;
  int row2 = ab2 / COLS;
  int col2 = ab2 % COLS;

  return std::sqrt((row1 - row2) ^ 2 + (col1 - col2) ^ 2);
}

std::vector<int> DACCSVGenerator::calculateSectorAirblocks(int x1, int y1,
                                                           int x2, int y2,
                                                           int ROWS, int COLS) {
  std::vector<int> blocks;
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
        int blockId = y * COLS + x;
        blocks.push_back(blockId);
      }
    }
  }
  return blocks;
}

void DACCSVGenerator::validateSectors(
    const std::vector<std::vector<int>> &sectors, int ROWS, int COLS) {
  int airblockCount = 0;
  for (const auto &sector : sectors) {
    if (sector.empty()) {
      throw std::invalid_argument("Empty sector found");
    }
    for (int block : sector) {
      if (block < 0 || block >= ROWS * COLS) {
        throw std::invalid_argument("Invalid airblock ID found in sector");
      }
    }
    airblockCount += sector.size();
  }
  if (airblockCount != ROWS * COLS) {
    throw std::invalid_argument("Not all airblocks are covered by sectors");
  }
}

void DACCSVGenerator::partition(
    int x1, int y1, int x2, int y2, int minSectorLength, int maxSectors,
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
        &sectorsCoordinates,
    std::mt19937 &rng) {
  int width = x2 - x1 + 1;
  int height = y2 - y1 + 1;

  if (maxSectors <= 0 || width < 2 * minSectorLength ||
      height < 2 * minSectorLength || width <= 0 || height <= 0) {
    sectorsCoordinates.push_back(
        std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
    return;
  }
  std::uniform_int_distribution<int> distX(x1 + minSectorLength - 1,
                                           x2 - minSectorLength + 1);
  std::uniform_int_distribution<int> distY(y1 + minSectorLength - 1,
                                           y2 - minSectorLength + 1);
  if (width >= height) {
    int newX = distX(rng);
    partition(x1, y1, newX, y2, minSectorLength, maxSectors - 1,
              sectorsCoordinates, rng);
    partition(newX + 1, y1, x2, y2, minSectorLength, maxSectors - 1,
              sectorsCoordinates, rng);
  } else {
    int newY = distY(rng);
    partition(x1, y1, x2, newY, minSectorLength, maxSectors - 1,
              sectorsCoordinates, rng);
    partition(x1, newY + 1, x2, y2, minSectorLength, maxSectors - 1,
              sectorsCoordinates, rng);
  }
}

void DACCSVGenerator::generateSingleTrajectory(
    std::vector<int> &trajectory, int start, int end, int ROWS, int COLS,
    std::vector<std::vector<int>> &allTrajectories, std::mt19937 &rng,
    std::uniform_real_distribution<double> &noise) {
  trajectory.push_back(start);
  while (trajectory.back() != end) {
    int last = trajectory.back();
    int row = last / COLS;
    int col = last % COLS;

    double dr = std::clamp(static_cast<double>((end / COLS) - row), -1.0, 1.0) +
                noise(rng);
    double dc = std::clamp(static_cast<double>((end % COLS) - col), -1.0, 1.0) +
                noise(rng);

    int newRow = std::clamp(static_cast<int>(row + dr), 0, ROWS - 1);
    int newCol = std::clamp(static_cast<int>(col + dc), 0, COLS - 1);

    int newBlock = newRow * COLS + newCol;
    if (newBlock != trajectory.back())
      trajectory.push_back(newBlock);
  }
}

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
  double minAirportDistance =
      5.0; // minimum distance in airblocks between airports
           // for the airport to airport trajectories

  /* Trajectories for: Airport A to Airbort B; Airport to grid border
   * and viceversa;  From border to border */
  int percBorderToBorder = 10;
  int percAirportToAirport = 60;
  int percAirportToBorder = 100 - percBorderToBorder - percAirportToAirport;
  int nFlights = 300;
  int nFlows = nFlights / 20; // estimate, shoud be adjusted

  std::vector<std::vector<int>> allTrajectories;
  std::uniform_int_distribution<int> airportChoiceDist(0, nAirports - 1);

  /* 1. Trajectories from airport A to airport B */
  for (int i = 0; i < nFlows * percAirportToAirport / 100; i++) {
    int start = AirportPositions[airportChoiceDist(rng)];
    int end = AirportPositions[airportChoiceDist(rng)];

    int attempts = 0;
    while (end == start ||
           calculateAirblockDistance(COLS, start, end) < minAirportDistance ||
           attempts > 20) {
      end = AirportPositions[airportChoiceDist(rng)];
      attempts++;
    }

    std::vector<int> trajectory;
    generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                             allTrajectories, rng, noise);
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
    generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                             allTrajectories, rng, noise);
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
    generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                             allTrajectories, rng, noise);
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
  // PARAMETERS
  //-----------------------------
  out << "[PARAMETERS]\n";
  out << "h,t_p,epsilon\n";
  out << NTIMESINONEHOUR << "," << MINCONFKEEPTIME << "," << EPSILON
      << "\n\n"; // h, t_p, epsilon

  //-----------------------------
  // TIME
  //-----------------------------
  out << "[TIME]\n";
  out << "id,start,end\n";

  for (int t = 0; t < NTIMES; t++) {
    int startH = STARTHOUR + t / NTIMESINONEHOUR;
    int startM = (t % NTIMESINONEHOUR) * (60 / NTIMESINONEHOUR);
    int endH = STARTHOUR + (t + 1) / NTIMESINONEHOUR;
    int endM = ((t + 1) % NTIMESINONEHOUR) * (60 / NTIMESINONEHOUR);
    std::string startM_string =
        (startM < 10) ? "0" + std::to_string(startM) : std::to_string(startM);
    std::string endM_string =
        (endM < 10) ? "0" + std::to_string(endM) : std::to_string(endM);

    out << t << "," << startH << ":" << startM_string << "," << endH << ":"
        << endM_string << "\n";
  }

  out << "\n";

  //-----------------------------
  // GRID
  //-----------------------------
  out << "[GRID]\n";
  out << "rows,cols\n";
  out << ROWS << "," << COLS << "\n\n";

  //-----------------------------
  // SECTORS
  //-----------------------------

  std::vector<std::vector<int>> configs;
  int sectorId = 0;

  out << "[SECTORS]\n";
  out << "id,airblocks\n";
  for (int c = 0; c < NCONFIGS; c++) {
    int maxSector = sectorSizeDist(rng);

    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
        sectorsCoordinates;
    partition(0, 0, COLS - 1, ROWS - 1, MINSECTORLENGTH, maxSector,
              sectorsCoordinates, rng);
    NSECTORS += sectorsCoordinates.size();
    std::vector<std::vector<int>> sectors;
    for (const auto &coord : sectorsCoordinates) {
      int x1 = coord.first.first;
      int y1 = coord.first.second;
      int x2 = coord.second.first;
      int y2 = coord.second.second;

      std::vector<int> blocks =
          calculateSectorAirblocks(x1, y1, x2, y2, ROWS, COLS);
      sectors.push_back(blocks);
    }
    validateSectors(sectors, ROWS, COLS);
    std::vector<int> sectorIds;
    for (size_t i = 0; i < sectors.size(); i++) {
      sectorIds.push_back(sectorId);
      out << sectorId++ << ",\"";
      for (size_t j = 0; j < sectors[i].size(); j++) {
        out << sectors[i][j];
        if (j + 1 < sectors[i].size())
          out << ";";
      }
      out << "\"\n";
    }
    configs.push_back(sectorIds);
  }

  out << "\n";

  out << "\n";

  //-----------------------------
  // CAPACITY
  //-----------------------------

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

  for (int c = 0; c < NCONFIGS; c++) {
    out << c << ",\"";
    for (size_t i = 0; i < configs[c].size(); i++) {
      out << configs[c][i];
      if (i + 1 < configs[c].size())
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
      double baseDepartures = ingressDist(rng) *
                              hourlyFactor(t / NTIMESINONEHOUR) *
                              trafficVolNoise(rng) / NTIMESINONEHOUR;
      int departures = std::max(0, static_cast<int>(baseDepartures));
      out << f << "," << t << "," << departures << "\n";
    }
  }

  out.close();

  std::cout << "Istanza salvata in " << filename << std::endl;
}