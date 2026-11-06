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
}
/* TODO: to fix*/
void DACCSVGenerator::generateStraightTrajectory(
    std::vector<int> &trajectory, int start, int end, int ROWS, int COLS,
    std::mt19937 &rng, std::uniform_real_distribution<double> &noise) {

  while (trajectory.back() != end) {
    int last = trajectory.back();
    int row = last / COLS;
    int col = last % COLS;

    double dr = std::clamp((double)(end / COLS - row), -1.0, 1.0) + noise(rng);
    double dc = std::clamp((double)(end % COLS - col), -1.0, 1.0) + noise(rng);

    int nextRmove = 0;
    int nextCmove = 0;

    if (std::abs(dr) > std::abs(dc)) {
      nextRmove = (dr > 0) ? 1 : -1;
    } else if (std::abs(dc) > std::abs(dr)) {
      nextCmove = (dc > 0) ? 1 : -1;
    } else {
      // If dr and dc are equal, randomly choose to move in either direction
      if (rng() % 2 == 0) {
        nextRmove = (dr > 0) ? 1 : -1;
      } else {
        nextCmove = (dc > 0) ? 1 : -1;
      }
    }

    int newRow = row + nextRmove;
    int newCol = col + nextCmove;

    int newBlock = newRow * COLS + newCol;

    trajectory.push_back(newBlock);
  }
  // Already adds the end block in the loop
}

void DACCSVGenerator::generateSingleTrajectory(
    std::vector<int> &trajectory, int start, int end, int ROWS, int COLS,
    std::vector<std::vector<int>> &allTrajectories, std::mt19937 &rng,
    std::uniform_real_distribution<double> &noise, TrajectoryShape shape) {

  trajectory.push_back(start);

  switch (shape) {
  case STRAIGHT:
    generateStraightTrajectory(trajectory, start, end, ROWS, COLS, rng, noise);
    break;

  case CURVED_1: {

    /* Generates a random point between start and end with some deviation to
     * simulate a curved trajectory */
    int midRow = 0;
    int midCol = 0;

    if (std::abs(start / COLS - end / COLS) < 2) {
      std::uniform_int_distribution<int> midRowDist(
          std::min(std::max(0, start / COLS - 2), std::max(0, end / COLS - 2)),
          std::max(std::min(ROWS - 1, start / COLS + 2),
                   std::min(ROWS - 1, end / COLS + 2)));
      midRow = midRowDist(rng);
    } else {
      std::uniform_int_distribution<int> midRowDist(
          std::min(start / COLS, end / COLS),
          std::max(start / COLS, end / COLS));
      midRow = midRowDist(rng);
    }
    if (std::abs(start % COLS - end % COLS) < 2) {
      std::uniform_int_distribution<int> midColDist(
          std::min(std::max(0, start % COLS - 2), std::max(0, end % COLS - 2)),
          std::max(std::min(COLS - 1, start % COLS + 2),
                   std::min(COLS - 1, end % COLS + 2)));
      midCol = midColDist(rng);
    } else {
      std::uniform_int_distribution<int> midColDist(
          std::min(start % COLS, end % COLS),
          std::max(start % COLS, end % COLS));
      midCol = midColDist(rng);
    }

    int midBlock = midRow * COLS + midCol;

    /* Now we aim for the middle point */
    generateStraightTrajectory(trajectory, start, midBlock, ROWS, COLS, rng,
                               noise);
    /* Then we go to the end */
    generateStraightTrajectory(trajectory, midBlock, end, ROWS, COLS, rng,
                               noise);
    break;
  }

  case CURVED_2: {

    /* Generates two random points between start and end with some deviation to
     * simulate a more complex curved trajectory */

    /* Generates the first midpoint between the start and the end */
    int midRow1 = 0;
    int midCol1 = 0;

    if (std::abs(start / COLS - end / COLS) < 2) {
      std::uniform_int_distribution<int> midRow1Dist(
          std::min(std::max(0, start / COLS - 2), std::max(0, end / COLS - 2)),
          std::max(std::min(ROWS - 1, start / COLS + 2),
                   std::min(ROWS - 1, end / COLS + 2)));
      midRow1 = midRow1Dist(rng);
    } else {
      std::uniform_int_distribution<int> midRow1Dist(
          std::min(start / COLS, end / COLS),
          std::max(start / COLS, end / COLS));
      midRow1 = midRow1Dist(rng);
    }
    if (std::abs(start % COLS - end % COLS) < 2) {
      std::uniform_int_distribution<int> midCol1Dist(
          std::min(std::max(0, start % COLS - 2), std::max(0, end % COLS - 2)),
          std::max(std::min(COLS - 1, start % COLS + 2),
                   std::min(COLS - 1, end % COLS + 2)));
      midCol1 = midCol1Dist(rng);
    } else {
      std::uniform_int_distribution<int> midCol1Dist(
          std::min(start % COLS, end % COLS),
          std::max(start % COLS, end % COLS));
      midCol1 = midCol1Dist(rng);
    }

    int midBlock1 = midRow1 * COLS + midCol1;

    /* Generates the second midpoint between the first midpoint and the end */
    std::uniform_int_distribution<int> midRow2Dist(
        std::min(midBlock1 / COLS, end / COLS),
        std::max(midBlock1 / COLS, end / COLS));
    std::uniform_int_distribution<int> midCol2Dist(
        std::min(midBlock1 % COLS, end % COLS),
        std::max(midBlock1 % COLS, end % COLS));

    int midRow2 = midRow2Dist(rng);
    int midCol2 = midCol2Dist(rng);
    int midBlock2 = midRow2 * COLS + midCol2;

    /* Goes to the first midpoint */
    generateStraightTrajectory(trajectory, start, midBlock1, ROWS, COLS, rng,
                               noise);

    /* Goes to the second midpoint */
    generateStraightTrajectory(trajectory, midBlock1, midBlock2, ROWS, COLS,
                               rng, noise);
    /* Goes to the end */
    generateStraightTrajectory(trajectory, midBlock2, end, ROWS, COLS, rng,
                               noise);
    break;
  }

  default:
    throw std::invalid_argument("Invalid trajectory shape");
  }
}

std::vector<std::vector<int>>
DACCSVGenerator::generateAllTrajectories(std::mt19937 &rng,
                                         std::vector<int> &airportPositions) {
  std::uniform_int_distribution<int> startDist(0, ROWS * COLS - 1);
  // std::uniform_int_distribution<int> lengthDist(TRAJ_LEN_MIN,
  // TRAJ_LEN_MAX);
  std::uniform_real_distribution<double> noise(-0.15, 0.15);
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
  int nFlights = 300;
  int nFlows = nFlights / 10; // estimate, shoud be adjusted

  std::vector<std::vector<int>> allTrajectories;
  std::uniform_int_distribution<int> airportChoiceDist(0, nAirports - 1);

  /* 1. Trajectories from airport A to airport B */
  for (int i = 0; i < nFlows * percAirportToAirport / 100 / 3; i++) {
    int start = AirportPositions[airportChoiceDist(rng)];
    int end = AirportPositions[airportChoiceDist(rng)];

    while (end == start ||
           calculateAirblockDistance(COLS, start, end) < minAirportDistance) {
      end = AirportPositions[airportChoiceDist(rng)];
    }

    for (TrajectoryShape shape : {STRAIGHT, CURVED_1, CURVED_2}) {
      std::vector<int> trajectory;
      generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                               allTrajectories, rng, noise, shape);
      allTrajectories.push_back(trajectory);
    }
  }

  /* 2. Trajectories from airport to border and viceversa */
  for (int i = 0; i < nFlows * percAirportToBorder / 100 / 2; i++) {
    int start = AirportPositions[rng() % nAirports];
    int end;

    int borderRow = (rng() % 2 == 0) ? 0 : ROWS - 1;
    int borderCol = rng() % COLS;
    end = borderRow * COLS + borderCol;

    if (i % 2 == 0) {
      std::swap(start, end); // half of the time, start from border to airport
    } // else half of the time, start from airport to border

    for (TrajectoryShape shape : {STRAIGHT, CURVED_1}) {
      std::vector<int> trajectory;
      generateSingleTrajectory(trajectory, start, end, ROWS, COLS,
                               allTrajectories, rng, noise, shape);
      allTrajectories.push_back(trajectory);
    }
  }

  /* 3. Trajectories from border to border */
  for (int i = 0; i < nFlows * percBorderToBorder / 100; i++) {
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

  std::normal_distribution<double> ingressDist(0.8 / NTIMESINONEHOUR,
                                               0.4 / NTIMESINONEHOUR);
  std::normal_distribution<double> trafficVolNoise(0.8, 1.6);

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

  std::vector<int> airportPositions;
  std::vector<std::vector<int>> allTrajectories =
      generateAllTrajectories(rng, airportPositions);
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
  for (int f = 0; f < allTrajectories.size(); f++) {
    for (int t = 0; t < NTIMES; t++) {
      double baseDepartures =
          ingressDist(rng) * hourlyFactor(t / NTIMESINONEHOUR) /**
                             trafficVolNoise(rng)*/
          ;
      int departures = std::max(0, static_cast<int>(baseDepartures));
      out << f << "," << t << "," << departures << "\n";
    }
  }

  out.close();

  std::cout << "Istanza salvata in " << filename << std::endl;
}