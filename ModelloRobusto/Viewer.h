#pragma once
#include "generate_dac_data.h"
#include <vector>

/* classe per visualizzare griglia con settori colorati */
class Viewer {
private:
  DACData &data;

public:
  Viewer(DACData &data);
  void show();
};