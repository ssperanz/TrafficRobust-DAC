#pragma once
#include "generate_dac_data.h"
#include <vector>

/* classe per visualizzare griglia con settori colorati */
class Viewer {
private:
  DACData &data;
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
  std::vector<std::vector<int>> grid;

public:
  Viewer(DACData &data);
  void showConfigurations();
  void showFlows();
};