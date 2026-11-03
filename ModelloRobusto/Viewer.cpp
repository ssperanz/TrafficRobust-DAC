#include "Viewer.h"
#include <array>
#include <iomanip>
#include <vector>

Viewer::Viewer(DACData &data) : data(data) {}

void Viewer::show() {
  // Implementation for showing the viewer
  int rows = data.grid.getNRows();
  int cols = data.grid.getNCols();
  int configs = data.getNConfigurations();
  std::vector<std::array<int, 3>> palette = {
      {255, 179, 186}, // Rosa
      {255, 223, 186}, // Pesca
      {255, 255, 186}, // Giallo
      {186, 255, 201}, // Menta
      {186, 225, 255}, // Azzurro
      {218, 186, 255}, // Lavanda
      {255, 186, 245}, // Rosa lilla
      {255, 204, 204}, // Rosa chiaro
      {255, 218, 185}, // Albicocca
      {250, 240, 190}, // Crema
      {204, 255, 229}, // Verde acqua
      {190, 230, 255}, // Celeste
      {220, 208, 255}, // Viola chiaro
      {245, 204, 229}, // Cipria
      {255, 200, 160}, // Corallo pastello
      {230, 230, 180}, // Giallo oliva
      {180, 230, 200}, // Verde salvia
      {175, 220, 230}, // Turchese
      {200, 190, 230}, // Malva
      {230, 190, 200}, // Rosa antico
      {255, 230, 210}, // Champagne
      {240, 245, 200}, // Lime pastello
      {200, 240, 220}, // Acquamarina
      {200, 220, 245}, // Blu polvere
      {230, 210, 245}, // Glicine
      {245, 220, 220}, // Rosa pallido
      {245, 230, 200}, // Sabbia
      {215, 235, 190}, // Pistacchio
      {190, 235, 235}, // Acqua
  };

  /** Initialize the grid for each configuration */
  std::vector<std::vector<int>> grid(configs,
                                     std::vector<int>(rows * cols, -1));

  for (auto &config : data.configurations) {
    for (auto &sector : config.sectors) {
      int sectorId = sector->getIndex();
      for (auto &airblock : sector->airblocks) {
        int abIndex = airblock->getIndex();
        grid[config.id][abIndex] = sectorId;
      }
    }
  }

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
          std::cout << "\033[48;2;" << color[0] << ";" << color[1] << ";"
                    << color[2] << "m" << std::setw(4) << abIndex
                    << (data.airblocks[abIndex].hasAirport ? "A" : " ")
                    << "\033[0m";
        }
      }
      std::cout << "\n";
    }
    std::cout << "\n";
  }

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

          std::cout << "\033[48;2;" << color[0] << ";" << color[1] << ";"
                    << color[2] << "m" << std::setw(4) << abIndex
                    << (data.airblocks[abIndex].hasAirport ? "A" : " ")
                    << "\033[0m";
        } else {
          std::cout << std::setw(4) << abIndex
                    << (data.airblocks[abIndex].hasAirport ? "A" : " ");
        }
      }
      std::cout << "\n";
    }
    std::cout << "\n";
  }
}