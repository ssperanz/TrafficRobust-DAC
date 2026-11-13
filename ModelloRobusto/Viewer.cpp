#include "Viewer.h"
#include <array>
#include <iomanip>
#include <vector>

Viewer::Viewer(DACData &data)
    : data(data),
      grid(data.getNConfigurations(),
           std::vector<int>(data.grid.getNRows() * data.grid.getNCols(), -1)) {
  /** Initialize the grid for each configuration */
  for (auto &config : data.configurations) {
    for (auto &sector : config.sectors) {
      int sectorId = sector->getIndex();
      for (auto &airblock : sector->airblocks) {
        int abIndex = airblock->getIndex();
        grid[config.id][abIndex] = sectorId;
      }
    }
  }
}

void Viewer::showConfigurations() {

  /** Prints the grid, showing sectorid in each cell, for each configuration */
  for (int c = 0; c < configs; ++c) {
    std::cout << "Configuration " << c << ":\n";
    for (int r = 0; r < rows; ++r) {
      for (int col = 0; col < cols; ++col) {
        int abIndex = r * cols + col;
        int sectorId = grid[c][abIndex];
        if (sectorId == -1) {
          std::cout << "ERR "; // No sector assigned
        } else {
          // Prints sectorId with background color
          // sectorId formatted to 3 digits for alignment
          auto color = palette[sectorId % palette.size()];
          /* Prints id in bold text for airports */
          if (data.airblocks[abIndex].hasAirport) {
            std::cout << "\033[1m\033[48;2;" << color[0] << ";" << color[1]
                      << ";" << color[2] << "m" << std::setw(4) << sectorId
                      << "\033[0m";
          } else {
            std::cout << "\033[48;2;" << color[0] << ";" << color[1] << ";"
                      << color[2] << "m" << std::setw(4) << sectorId
                      << "\033[0m";
          }
        }
      }
      std::cout << "\n";
    }
    std::cout << "\n";
  }
}

void Viewer::showFlows() {

  /* Prints the flows path singularly on the grid */
  for (const auto &flow : data.flows) {
    std::cout << "Flow " << flow.id << " path:\n";
    for (int r = 0; r < rows; ++r) {
      for (int col = 0; col < cols; ++col) {
        int abIndex = r * cols + col;
        /* Prints every airblock in the grid, colours the flow path*/
        if (std::find_if(flow.airblocks.begin(), flow.airblocks.end(),
                         [abIndex](AirBlock *ab) {
                           return ab->getIndex() == abIndex;
                         }) != flow.airblocks.end()) {
          auto color = palette[flow.id % palette.size()];

          std::cout << (data.airblocks[abIndex].hasAirport ? "\033[1m\033[48;2;"
                                                           : "\033[48;2;")
                    << color[0] << ";" << color[1] << ";" << color[2] << "m"
                    << std::setw(4) << abIndex << "\033[0m";
        } else {
          std::cout << std::setw(4) << abIndex << "\033[0m";
        }
      }
      std::cout << "\n";
    }
    std::cout << "\n";
  }
}