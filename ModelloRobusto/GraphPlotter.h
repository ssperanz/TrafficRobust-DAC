
#include <sciplot/sciplot.hpp>

#include "DACData.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace sciplot;

class GraphPlotter {
private:
  DACData &data;
  int maxOccupancy;

public:
  explicit GraphPlotter(DACData &data) : data(data) {
    int maxAbOccupancy = 0;
    for (int t = 0; t < data.grid.getNTimeslots(); t++) {
      for (int ab_id = 0; ab_id < data.grid.getNRows() * data.grid.getNCols();
           ab_id++) {
        int occupancy = data.aux.q_t_a[t][ab_id];
        if (occupancy > maxAbOccupancy) {
          maxAbOccupancy = occupancy;
        }
      }
    }
    maxOccupancy = maxAbOccupancy;
  }

  void plotAirblockDemandHeatmap(int timeSlot) {

    int T = data.grid.getNTimeslots();
    double nx = data.grid.getNCols();
    double ny = data.grid.getNRows();

    if (timeSlot < 0 || timeSlot >= T)
      throw std::out_of_range("Time slot non valido");

    int N = nx * ny;

    if (static_cast<int>(data.aux.q_t_a[timeSlot].size()) < N) {
      throw std::runtime_error("Numero di occupazioni insufficiente");
    }

    // Esportazione della griglia
    std::ofstream file("airspace_heatmap.dat");

    if (!file)
      throw std::runtime_error("Errore apertura file");

    std::vector<double> x, y, occupancy;

    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {

        const int block = j * nx + i;

        const double value = data.aux.q_t_a[timeSlot][block];

        x.push_back(i + 0.5);
        y.push_back(j + 0.5);
        occupancy.push_back(value);
      }
    }

    // Creazione grafico
    Plot2D plot;

    plot.xlabel("COLUMN");
    plot.ylabel("ROW");

    // Configurazione Gnuplot
    plot.gnuplot( // Asse X in alto
        "set xrange [0:" + std::to_string(nx) +
        "]\n"
        "set xtics 1 nomirror\n"

        // Asse Y invertito
        "set yrange [" +
        std::to_string(ny) +
        ":0]\n"
        "set ytics 1 nomirror\n"

        // Griglia
        "unset mxtics\n"
        "unset mytics\n"
        "set grid xtics ytics front lc rgb '#eaeaea' lw 1\n"

        // Palette
        "set palette defined (0 'white', 1 '#006eff')\n"
        "set cbrange [0:" +
        std::to_string(maxOccupancy) +
        "]\n"
        "set colorbox\n");

    // Disegno degli airblock
    plot.drawWithVecs("image", x, y, occupancy);

    // Grafico
    Figure fig = {{plot}};
    Canvas canvas = {{fig}};

    canvas.size(1000, 800);

    std::string filename =
        "airspace_heatmap_" + std::to_string(timeSlot) + ".png";

    canvas.save(filename);
    // canvas.show();
  }

  void plotSectorDemandHeatmap(int timeSlot, int conf) {
    int T = data.grid.getNTimeslots();
    double nx = data.grid.getNCols();
    double ny = data.grid.getNRows();

    if (timeSlot < 0 || timeSlot >= T)
      throw std::out_of_range("Time slot non valido");

    if (conf < 0 || conf >= data.configurations.size())
      throw std::out_of_range("Configurazione non valida");

    int N = nx * ny;

    if (static_cast<int>(data.aux.q_t_a[timeSlot].size()) < N) {
      throw std::runtime_error("Numero di occupazioni insufficiente");
    }

    // Esportazione della griglia
    std::ofstream file("sector_heatmap.dat");

    if (!file)
      throw std::runtime_error("Errore apertura file");

    std::vector<double> x, y, occupancy;

    for (Sector *sector : data.configurations[conf].sectors) {
      int sector_id = sector->getIndex();
      int value = data.aux.Q_t_s[timeSlot][sector_id];
      for (AirBlock *ab : sector->airblocks) {
        int i = ab->col;
        int j = ab->row;

        x.push_back(i + 0.5);
        y.push_back(j + 0.5);
        occupancy.push_back(value);
      }
    }

    // Creazione grafico
    Plot2D plot;

    plot.xlabel("COLUMN");
    plot.ylabel("ROW");

    // Configurazione Gnuplot
    plot.gnuplot( // Asse X in alto
        "set xrange [0:" + std::to_string(nx) +
        "]\n"
        "set xtics 1 nomirror\n"

        // Asse Y invertito
        "set yrange [" +
        std::to_string(ny) +
        ":0]\n"
        "set ytics 1 nomirror\n"

        // Griglia
        "unset mxtics\n"
        "unset mytics\n"
        "set grid xtics ytics front lc rgb '#eaeaea' lw 1\n"

        // Palette
        "set palette defined (0 'white', 1 '#ffa33a')\n"
        "set cbrange [0:" +
        std::to_string(maxOccupancy) +
        "]\n"
        "set colorbox\n");

    // Disegno degli airblock
    plot.drawWithVecs("image", x, y, occupancy);

    // Grafico
    Figure fig = {{plot}};
    Canvas canvas = {{fig}};

    canvas.size(1000, 800);

    std::string filename = "sector_heatmap_" + std::to_string(timeSlot) + "_" +
                           std::to_string(conf) + ".png";

    canvas.save(filename);
    // canvas.show();
  }
};
