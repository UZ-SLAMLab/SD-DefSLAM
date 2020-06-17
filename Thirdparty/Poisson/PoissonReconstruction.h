#pragma once
#include <iostream>
#include <vector>
class PoissonReconstruction {
public:
  PoissonReconstruction() = default;

  PoissonReconstruction(std::vector<float> &points, double Radius = 2,
                        int pointDensity = 200, int KSearch = 100,
                        int depth = 4);

  ~PoissonReconstruction() = default;
  void execute(std::vector<std::vector<float>> &nodes,
               std::vector<std::vector<int>> &facets);

private:
  std::vector<float> points;
  double Radius;
  int pointDensity;
  int KSearch;
  int depth;
};
