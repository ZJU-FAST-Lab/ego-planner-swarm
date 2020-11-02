/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
* @Create Date:   2020-11-02 13:47:44
* @Last Modified by:   JC Zhu
* @Last Modified time: 2020-11-02 13:47:47
*/

#include <chrono>
#include <iostream>

class Timer {
 public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()),
        end_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

  void stop() { end_ = std::chrono::high_resolution_clock::now(); }

  double elapsedSeconds() const {
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
        end_ - start_);
    return timeSpan.count();
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};