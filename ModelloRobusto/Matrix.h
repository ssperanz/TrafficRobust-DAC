#pragma once
#include <vector>

template <class P> class Matrix {
private:
  int T, C;
  std::vector<P> v;

public:
  Matrix(int T, int C) : T(T), C(C) { v.resize(T * C); }

  void set(int t, int c, P var) { v[t * C + c] = var; }

  P &at(int t, int c) { return v[t * C + c]; }

  int getnrows() { return T; };
  int getncols() { return C; };

  std::vector<P> getLine(int l) {
    std::vector<P> line;

    for (int c = 0; c < C; c++) {
      line.push_back(v[l * C + c]);
    }

    return line;
  }
};