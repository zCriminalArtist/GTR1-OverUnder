#ifndef COEFFICIENT_H
#define COEFFICIENT_H

#include <unordered_map>
#include "control/PIDController.h"

struct CoefficientLUT {
  CoefficientLUT() {
    umap.emplace(0, Path(0,0,0));
    // umap.emplace(1, Path(0,0,0));
    // umap.emplace(2, Path(0,0,0));
  }
  void add(int key, Path p) { umap.emplace(key, p); }
  Path& get(int key) { return umap.at(key); }
  std::unordered_map<int, Path> umap;
};

#endif COEFFICIENT_H