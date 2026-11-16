#pragma once

#include "DACData.h"

#include <sciplot/sciplot.hpp>

#include <string>

using namespace sciplot;

class GraphPlotter {
private:
  DACData &data;
  int maxOccupancy;

public:
  explicit GraphPlotter(DACData &data);

  void plotAirblockDemandHeatmap(int timeSlot);
  void plotSectorDemandHeatmap(int timeSlot, int conf);
};
