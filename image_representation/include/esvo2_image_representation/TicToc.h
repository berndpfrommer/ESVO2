#ifndef esvo2_tictoc_H_
#define esvo2_tictoc_H_

#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace esvo2_image_representation {
class TicToc {
public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
} // namespace esvo2_image_representation

#endif // esvo2_tictoc_H_
