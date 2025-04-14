#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "spdlog/spdlog.h"

#include "veschost_tests.hpp"

TEST_CASE("test stats","[veschosts][stats]")
{
  auto dataStat = [](double* data, size_t len) -> std::tuple<double, double, double>
  {
    //spdlog::info("Average {} values", len);
    double avg = 0, var = 0, stddev = 0;
    double _avg_p=0, _vvar=0;
    size_t n = 0;
    for(size_t i=0;i<len;i++)
    {
      n++;
      auto& v = data[i];
      //spdlog::info("  Add {}/{}: {}", n, len, v);
      _avg_p = avg;
      avg += (v-avg)/n;
      //spdlog::info("    Average {}/{}: {}", n, len, avg);
      if(n>1)
      {
        _vvar += (v-_avg_p)*(v-avg);
        var = _vvar/(n-1);
        //spdlog::info("    Variance {}/{}: {}", n, len, var);
        stddev = sqrt(var);
        //spdlog::info("    StdDev {}/{}: {}", n, len, stddev);
      }
    }
    return { avg, var, stddev };
  };


  {
    double data[] = { 1, 2, 3,};
    auto [avg, var, stddev] = dataStat(data, sizeof(data)/sizeof(double));

    REQUIRE(avg == 2);
    REQUIRE(var == 1);
    REQUIRE(stddev == 1);
  }
  {
    double data[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9};
    auto [avg, var, stddev] = dataStat(data, sizeof(data)/sizeof(double));

    REQUIRE(avg == 5);
    REQUIRE(var == 7.5);
    REQUIRE_THAT(stddev, Catch::Matchers::WithinRel(2.7386128,0.0001));
  }
}