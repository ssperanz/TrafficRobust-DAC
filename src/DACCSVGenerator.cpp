#include "DACCSVGenerator.h"

DACCSVGenerator::DACCSVGenerator(int rows, int cols, int nhours, int startHour,
                                 int min_sector_length, int nconfigs,
                                 int NFLOWS, int NFLIGHTS, int h, int t_p,
                                 double epsilon, double mu_departures_dist,
                                 unsigned int seed)
    : ROWS(rows), COLS(cols), NTIMES(nhours * h), STARTHOUR(startHour),
      NSECTORS(0), MINSECTORLENGTH(min_sector_length), NCONFIGS(nconfigs),
      NFLOWS(NFLOWS), NFLIGHTS(NFLIGHTS), NTIMESINONEHOUR(h),
      MINCONFKEEPTIME(t_p), EPSILON(epsilon),
      MU_DEPARTURES_DIST(mu_departures_dist), rng(seed) {}

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

  int dr = row1 - row2;
  int dc = col1 - col2;

  return std::sqrt(dr * dr + dc * dc);
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

void DACCSVGenerator::validateConfigurations(
    const std::vector<std::vector<int>> &configs,
    const std::vector<std::vector<int>> &sectors, int ROWS, int COLS) {
  for (const auto &config : configs) {
    int totalAirblocks = 0;
    for (int sectorId : config) {
      if (sectorId < 0 || sectorId >= sectors.size()) {
        throw std::invalid_argument("Invalid sector ID found in configuration");
      }
      /* verifica che la somma degli airblocks sia corretta */
      totalAirblocks += sectors[sectorId].size();
    }
    if (totalAirblocks != ROWS * COLS) {
      throw std::invalid_argument(
          "Incorrect total number of airblocks in configuration");
    }
  }
}

bool DACCSVGenerator::checkSectorsIdenticity(const std::vector<int> &sectorA,
                                             const std::vector<int> &sectorB) {
  if (sectorA.size() != sectorB.size()) {
    return false;
  }

  std::vector<int> sortedA = sectorA;
  std::vector<int> sortedB = sectorB;
  std::sort(sortedA.begin(), sortedA.end());
  std::sort(sortedB.begin(), sortedB.end());

  return sortedA == sortedB;
}

/*void DACCSVGenerator::partition(
    int x1, int y1, int x2, int y2, int minSectorLength, int maxSectors,
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
        &sectorsCoordinates,
    std::mt19937 &rng) {
  int width = x2 - x1 + 1;
  int height = y2 - y1 + 1;

  if (x1 > x2 || y1 > y2)
    return; // Invalid coordinates
  if (width <= 0 || height <= 0)
    return; // Invalid dimensions
  if (maxSectors <= 1 ||
      (width <= 2 * minSectorLength && height <= 2 * minSectorLength)) {
    sectorsCoordinates.push_back(
        std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
    return; // Cannot partition further
  }

  if (width >= height) {
    std::uniform_int_distribution<int> distX(x1 + minSectorLength - 1,
                                             x2 - minSectorLength);
    int newX = distX(rng);
    partition(x1, y1, newX, y2, minSectorLength, maxSectors / 2,
              sectorsCoordinates, rng);
    partition(newX + 1, y1, x2, y2, minSectorLength,
              maxSectors - (maxSectors / 2), sectorsCoordinates, rng);
  } else {
    std::uniform_int_distribution<int> distY(y1 + minSectorLength - 1,
                                             y2 - minSectorLength);
    int newY = distY(rng);
    partition(x1, y1, x2, newY, minSectorLength, maxSectors / 2,
              sectorsCoordinates, rng);
    partition(x1, newY + 1, x2, y2, minSectorLength,
              maxSectors - (maxSectors / 2), sectorsCoordinates, rng);
  }
}*/

std::vector<std::vector<int>> DACCSVGenerator::partition(
    int x1, int y1, int x2, int y2, int minSectorLength, int maxSectors,
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
        &sectorsCoordinates,
    std::vector<int> &sectorIds) {

  int width = x2 - x1 + 1;
  int height = y2 - y1 + 1;

  if (x1 > x2 || y1 > y2 || width <= 0 || height <= 0)
    return {};

  std::vector<std::vector<int>> configurations;

  int sectorId = sectorIds.size();
  sectorIds.push_back(sectorId);
  sectorsCoordinates.push_back({{x1, y1}, {x2, y2}});
  configurations.push_back({sectorId});

  bool canSplitHorizontally = width >= height && width >= 2 * minSectorLength;
  bool canSplitVertically = height >= 2 * minSectorLength;

  if (maxSectors <= 1 || (!canSplitHorizontally && !canSplitVertically)) {
    return configurations;
  }

  std::vector<std::vector<int>> subconf1;
  std::vector<std::vector<int>> subconf2;

  if (canSplitHorizontally) {
    std::uniform_int_distribution<int> distX(x1 + minSectorLength - 1,
                                             x2 - minSectorLength);
    int newX = distX(rng);
    subconf1 = partition(x1, y1, newX, y2, minSectorLength, maxSectors / 2,
                         sectorsCoordinates, sectorIds);
    subconf2 =
        partition(newX + 1, y1, x2, y2, minSectorLength,
                  maxSectors - maxSectors / 2, sectorsCoordinates, sectorIds);
  } else {
    std::uniform_int_distribution<int> distY(y1 + minSectorLength - 1,
                                             y2 - minSectorLength);
    int newY = distY(rng);
    subconf1 = partition(x1, y1, x2, newY, minSectorLength, maxSectors / 2,
                         sectorsCoordinates, sectorIds);
    subconf2 =
        partition(x1, newY + 1, x2, y2, minSectorLength,
                  maxSectors - maxSectors / 2, sectorsCoordinates, sectorIds);
  }

  configurations.reserve(1 + subconf1.size() * subconf2.size());

  for (const auto &first : subconf1) {
    for (const auto &second : subconf2) {
      std::vector<int> combined;
      combined.reserve(first.size() + second.size());
      combined.insert(combined.end(), first.begin(), first.end());
      combined.insert(combined.end(), second.begin(), second.end());
      configurations.push_back(std::move(combined));
    }
  }

  return configurations;
}

void DACCSVGenerator::generateStraightTrajectory(std::vector<int> &trajectory,
                                                 int start, int end, int ROWS,
                                                 int COLS) {

  while (trajectory.back() != end) {
    int last = trajectory.back();
    int row = last / COLS;
    int col = last % COLS;

    int dr = end / COLS - row;
    int dc = end % COLS - col;
    std::uniform_int_distribution<int> chance(1, std::abs(dr) + std::abs(dc));

    int nextRmove = 0;
    int nextCmove = 0;

    int choice = chance(rng);
    if (choice <= std::abs(dr)) {
      nextRmove = (dr > 0) ? 1 : -1;
    } else {
      nextCmove = (dc > 0) ? 1 : -1;
    }

    int newRow = row + nextRmove;
    int newCol = col + nextCmove;

    if (newRow < 0 || newRow >= ROWS || newCol < 0 || newCol >= COLS) {
      break;
    } else {
      int newBlock = newRow * COLS + newCol;
      trajectory.push_back(newBlock);
    }
    // Already adds the end block in the loop
  }
}

void DACCSVGenerator::generateSingleTrajectory(
    std::vector<int> &trajectory, int start, int end, int ROWS, int COLS,
    std::vector<std::vector<int>> &allTrajectories, TrajectoryShape shape) {

  trajectory.push_back(start);

  switch (shape) {
  case STRAIGHT:
    generateStraightTrajectory(trajectory, start, end, ROWS, COLS);
    break;

  case CURVED_1: {

    /* Generates a random point between start and end with some deviation to
     * simulate a curved trajectory */
    int midRow = 0;
    int midCol = 0;
    bool rand = rng() % 2 == 0;
    int dr = (start / COLS - end / COLS);
    int dc = (start % COLS - end % COLS);

    if (std::abs(dr) < 2) {
      midRow = (start / COLS + end / COLS) / 2 + (rand ? 1 : -1);
      if (midRow < 0 || midRow >= ROWS) {
        midRow = std::max(0, std::min(ROWS - 1, midRow));
      }
      std::uniform_int_distribution<int> midColDist(
          std::min(start % COLS + std::abs(dc) / 3,
                   end % COLS + std::abs(dc) / 3),
          std::max(start % COLS - std::abs(dc) / 3,
                   end % COLS - std::abs(dc) / 3));
      midCol = midColDist(rng);
    } else if (std::abs(dc) < 2) {
      midCol = (start % COLS + end % COLS) / 2 + (rand ? 1 : -1);
      if (midCol < 0 || midCol >= COLS) {
        midCol = std::max(0, std::min(COLS - 1, midCol));
      }
      std::uniform_int_distribution<int> midRowDist(
          std::min(start / COLS + std::abs(dr) / 3,
                   end / COLS + std::abs(dr) / 3),
          std::max(start / COLS - std::abs(dr) / 3,
                   end / COLS - std::abs(dr) / 3));
      midRow = midRowDist(rng);
    } else if (std::abs(dr) > std::abs(dc)) {
      if (dr > 0) {
        std::uniform_int_distribution<int> midRowDist(
            end / COLS + std::abs(dr) / 3, start / COLS - std::abs(dr) / 3);
        midRow = midRowDist(rng);
      } else {
        std::uniform_int_distribution<int> midRowDist(
            start / COLS + std::abs(dr) / 3, end / COLS - std::abs(dr) / 3);
        midRow = midRowDist(rng);
      }
      std::uniform_int_distribution<int> midColDist(
          std::min(start % COLS, end % COLS),
          std::max(start % COLS, end % COLS));
      midCol = midColDist(rng);
    } else if (std::abs(dr) < std::abs(dc)) {
      if (dc > 0) {
        std::uniform_int_distribution<int> midColDist(
            end % COLS + std::abs(dc) / 3, start % COLS - std::abs(dc) / 3);
        midCol = midColDist(rng);
      } else {
        std::uniform_int_distribution<int> midColDist(
            start % COLS + std::abs(dc) / 3, end % COLS - std::abs(dc) / 3);
        midCol = midColDist(rng);
      }
      std::uniform_int_distribution<int> midRowDist(
          std::min(start / COLS, end / COLS),
          std::max(start / COLS, end / COLS));
      midRow = midRowDist(rng);
    } else {
      std::uniform_int_distribution<int> midRowDist(
          std::min(start / COLS + std::abs(dr) / 3,
                   end / COLS + std::abs(dr) / 3),
          std::max(start / COLS - std::abs(dr) / 3,
                   end / COLS - std::abs(dr) / 3));
      midRow = midRowDist(rng);
      std::uniform_int_distribution<int> midColDist(
          std::min(start % COLS + std::abs(dc) / 3,
                   end % COLS + std::abs(dc) / 3),
          std::max(start % COLS - std::abs(dc) / 3,
                   end % COLS - std::abs(dc) / 3));
      midCol = midColDist(rng);
    }

    int midBlock = midRow * COLS + midCol;

    /* Now we aim for the middle point */
    generateStraightTrajectory(trajectory, start, midBlock, ROWS, COLS);
    /* Then we go to the end */
    generateStraightTrajectory(trajectory, midBlock, end, ROWS, COLS);
    break;
  }

  case CURVED_2: {

    /* Generates a random point between start and end with some deviation to
     * simulate a curved trajectory */

    int dr = (start / COLS - end / COLS);
    int dc = (start % COLS - end % COLS);

    std::uniform_int_distribution<int> midRowDist(
        std::min(start / COLS + std::abs(dr) / 3,
                 end / COLS + std::abs(dr) / 3),
        std::max(start / COLS - std::abs(dr) / 3,
                 end / COLS - std::abs(dr) / 3));
    int midRow = midRowDist(rng);
    std::uniform_int_distribution<int> midColDist(
        std::min(start % COLS + std::abs(dc) / 3,
                 end % COLS + std::abs(dc) / 3),
        std::max(start % COLS - std::abs(dc) / 3,
                 end % COLS - std::abs(dc) / 3));
    int midCol = midColDist(rng);

    int midBlock = midRow * COLS + midCol;

    /* Now we aim for the middle point */
    generateStraightTrajectory(trajectory, start, midBlock, ROWS, COLS);
    /* Then we go to the end */
    generateStraightTrajectory(trajectory, midBlock, end, ROWS, COLS);
    break;
  }

  default:
    throw std::invalid_argument("Invalid trajectory shape");
  }
}

std::vector<std::vector<int>>
DACCSVGenerator::generateAllTrajectories(std::vector<int> &airportPositions) {
  std::uniform_int_distribution<int> startDist(0, ROWS * COLS - 1);
  // std::uniform_int_distribution<int> lengthDist(TRAJ_LEN_MIN,
  // TRAJ_LEN_MAX);
  std::uniform_int_distribution<int> airportDist(0, ROWS * COLS - 1);

  int nAirports = ROWS * COLS / 10;
  int AirportPositions[nAirports]; // airblock id of airports
  for (int i = 0; i < nAirports; i++) {
    AirportPositions[i] = airportDist(rng);
    airportPositions.push_back(AirportPositions[i]);
  }
  double minAirportDistance =
      5.0; // minimum distance in airblocks between airports
           // for the airport to airport trajectories

  /* Trajectories for: Airport A to Airbort B; Airport to grid border
   * and viceversa;  From border to border */
  int percBorderToBorder = 10;
  int percAirportToAirport = 60;
  int percAirportToBorder = 100 - percBorderToBorder - percAirportToAirport;

  std::vector<std::vector<int>> allTrajectories;
  std::uniform_int_distribution<int> airportChoiceDist(0, nAirports - 1);

  /* 1. Trajectories from airport A to airport B */
  for (int i = 0; i < NFLOWS * percAirportToAirport / 100 / 3; i++) {
    int start = AirportPositions[airportChoiceDist(rng)];
    int end = AirportPositions[airportChoiceDist(rng)];

    while (end == start ||
           calculateAirblockDistance(COLS, start, end) < minAirportDistance) {
      end = AirportPositions[airportChoiceDist(rng)];
    }

    for (TrajectoryShape shape : {STRAIGHT, CURVED_1, CURVED_2}) {
      std::vector<int> trajectory;
      generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                               allTrajectories, shape);
      allTrajectories.push_back(trajectory);
    }
  }

  /* 2. Trajectories from airport to border and viceversa */
  for (int i = 0; i < NFLOWS * percAirportToBorder / 100 / 2; i++) {
    int start = AirportPositions[rng() % nAirports];
    int end;

    int borderRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
    int borderCol = rng() % COLS;
    end = borderRow * COLS + borderCol;

    if (i % 2 == 0) {
      std::swap(start,
                end); // half of the time, start from border to airport
    } // else half of the time, start from airport to border

    for (TrajectoryShape shape : {STRAIGHT, CURVED_1}) {
      std::vector<int> trajectory;
      generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                               allTrajectories, shape);
      allTrajectories.push_back(trajectory);
    }
  }

  /* 3. Trajectories from border to border */
  for (int i = 0; i < NFLOWS * percBorderToBorder / 100; i++) {
    int startRow, startCol, endRow, endCol;
    if (rng() % 2 == 0) {
      startRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
      startCol = rng() % COLS;

    } else {
      startRow = rng() % ROWS;
      startCol = (rng() % 2 == 0) ? 0 : COLS - 1;
    }

    int start = startRow * COLS + startCol;

    if (rng() % 2 == 0) {
      do {
        endRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
      } while (endRow == startRow);
      endCol = rng() % COLS;

    } else {
      do {
        endCol = (rng() % 2 == 0) ? 0 : COLS - 1;
      } while (endCol == startCol);
      endRow = rng() % ROWS;
    }

    int end = endRow * COLS + endCol;

    std::vector<int> trajectory;
    generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                             allTrajectories);
    allTrajectories.push_back(trajectory);
  }

  return allTrajectories;
}

void DACCSVGenerator::generateDACCSV(const std::string &filename) {
  std::uniform_int_distribution<int> blockDist(0, ROWS * COLS - 1);
  std::uniform_int_distribution<int> sectorSizeDist(2, 15);
  std::uniform_real_distribution<double> capacityDist(40.0, 50.0);
  std::normal_distribution<double> capacityNoise(1.0, 0.05);
  std::normal_distribution<double> noise(1.0, 0.15);

  std::normal_distribution<double> ingressDist(MU_DEPARTURES_DIST, 0.4);
  std::normal_distribution<double> trafficVolNoise(0.8, 1.6);

  std::ofstream out(filename);

  //-----------------------------
  // PARAMETERS
  //-----------------------------
  out << "[PARAMETERS]\n";
  out << "h,t_p,epsilon\n";
  out << STARTHOUR << "," << NTIMESINONEHOUR << "," << MINCONFKEEPTIME << ","
      << EPSILON << "\n\n"; // h, t_p, epsilon

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
  std::vector<int> partitionSectorIds;
  std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>
      sectorsCoordinates;

  configs = partition(0, 0, COLS - 1, ROWS - 1, MINSECTORLENGTH, 64,
                      sectorsCoordinates, partitionSectorIds);

  /* Estrae tutte le istanze con meno di 5 settori */
  std::vector<std::vector<int>> filteredConfigs;
  std::vector<std::vector<int>> remainingConfigs;

  for (const auto &config : configs) {
    if (config.size() < 5) {
      filteredConfigs.push_back(config);
    } else {
      remainingConfigs.push_back(config);
    }
  }

  /* Shuffle delle configurazioni rimanenti */
  std::shuffle(remainingConfigs.begin(), remainingConfigs.end(), rng);

  /* Aggiunge configurazioni casuali fino a raggiungere NCONFIGS */
  const size_t remaining =
      NCONFIGS > filteredConfigs.size() ? NCONFIGS - filteredConfigs.size() : 0;

  if (remainingConfigs.size() > remaining) {
    remainingConfigs.resize(remaining);
  }

  filteredConfigs.insert(filteredConfigs.end(), remainingConfigs.begin(),
                         remainingConfigs.end());

  configs = std::move(filteredConfigs);

  if (configs.empty()) {
    throw std::runtime_error("Partition generated no configurations");
  }

  std::vector<std::vector<int>> sectors;
  sectors.reserve(sectorsCoordinates.size());
  for (const auto &coord : sectorsCoordinates) {
    sectors.push_back(calculateSectorAirblocks(
        coord.first.first, coord.first.second, coord.second.first,
        coord.second.second, ROWS, COLS));
  }
  validateConfigurations(configs, sectors, ROWS, COLS);

  std::vector<std::vector<int>> uniqueSectors;
  std::vector<int> coordinateIdToCsvSectorId(sectors.size(), -1);

  out << "[SECTORS]\n";
  out << "id,airblocks\n";

  int csvSectorId = 0;
  for (size_t i = 0; i < sectors.size(); ++i) {
    int currentSectorId = -1;

    for (size_t u = 0; u < uniqueSectors.size(); ++u) {
      if (checkSectorsIdenticity(uniqueSectors[u], sectors[i])) {
        currentSectorId = static_cast<int>(u);
        break;
      }
    }

    if (currentSectorId == -1) {
      currentSectorId = csvSectorId++;
      uniqueSectors.push_back(sectors[i]);

      out << currentSectorId << ",\"";
      for (size_t j = 0; j < sectors[i].size(); ++j) {
        out << sectors[i][j];
        if (j + 1 < sectors[i].size()) {
          out << ";";
        }
      }
      out << "\"\n";
    }

    coordinateIdToCsvSectorId[i] = currentSectorId;
  }

  for (auto &configuration : configs) {
    for (int &coordinateId : configuration) {
      if (coordinateId < 0 ||
          coordinateId >= static_cast<int>(coordinateIdToCsvSectorId.size())) {
        throw std::runtime_error(
            "Invalid sector ID in partition configuration");
      }
      coordinateId = coordinateIdToCsvSectorId[coordinateId];
    }
  }

  NSECTORS = csvSectorId;
  // NCONFIGS = static_cast<int>(configs.size());

  out << "\n\n";

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

  std::vector<int> airportPositions;
  std::vector<std::vector<int>> allTrajectories =
      generateAllTrajectories(airportPositions);
  for (size_t trajectory = 0; trajectory < allTrajectories.size();
       trajectory++) {
    for (size_t i = 0; i < allTrajectories[trajectory].size(); i++) {
      out << trajectory << "," << allTrajectories[trajectory][i] << "," << 1
          << "\n";
    }
  }

  out << "\n";

  //-----------------------------
  // AIRPORTS //
  //-----------------------------

  out << "[AIRPORTS]\n";
  out << "id,airblock\n";
  for (size_t i = 0; i < airportPositions.size(); i++) {
    out << i << "," << airportPositions[i] << "\n";
  }

  out << "\n";

  //-----------------------------
  // DEPARTURES //
  //-----------------------------

  out << "[DEPARTURES]\n";
  out << "id,time,scheduled_departures\n";
  int nDeparturesForFlow[allTrajectories.size()] = {0};
  std::uniform_int_distribution<int> flightsDist(0, allTrajectories.size() - 1);
  std::uniform_int_distribution<int> timeDist(0, NTIMES - 1);

  for (int f = 0; f < NFLIGHTS; f++) {
    int flowId = flightsDist(rng);
    nDeparturesForFlow[flowId] += 1;
  }

  for (int f = 0; f < allTrajectories.size(); f++) {
    int departures[NTIMES] = {0};
    for (int flight = 0; flight < nDeparturesForFlow[f]; flight++) {
      int time = 0;
      do {
        time = timeDist(rng);
      } while (departures[time] >=
               1); // Ensure only one departure per time slot
      departures[time] += 1;
    }
    for (int t = 0; t < NTIMES; t++) {
      out << f << "," << t << "," << departures[t] << "\n";
    }
  }

  // for (int f = 0; f < allTrajectories.size(); f++) {
  //   for (int t = 0; t < NTIMES; t++) {
  //     double baseDepartures = ingressDist(rng); /* hourlyFactor(t /
  //                             NTIMESINONEHOUR) /** trafficVolNoise(rng); */
  //     int departures = std::max(0, static_cast<int>(baseDepartures));
  //     departures = std::min(departures, 1);
  //     out << f << "," << t << "," << departures << "\n";
  //   }
  // }

  out.close();

  std::cout << "Istanza salvata in " << filename << std::endl;
}