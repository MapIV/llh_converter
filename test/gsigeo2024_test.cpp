#include "llh_converter/gsigeo2024.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

void test(const double& result, const double& answer)
{
  int i_result = std::round(result * 10000);
  int i_answer = std::round(answer * 10000);
  if (i_result == i_answer)
  {
    std::cout << "\033[32;1mTEST SUCCESS: " << result << " == " << answer << "\033[m" << std::endl;
  }
  else
  {
    std::cout << "\033[31;1mTEST FAILED : " << result << " != " << answer << "\033[m" << std::endl;
  }
}

int main()
{
  llh_converter::GSIGEO2024 geoid_model;
  geoid_model.loadGeoidMap("/usr/share/GSIGEO/GSIGEO2024beta.isg");

  std::cout << "Testing (36.104394, 140.085365) ... ";
  test(geoid_model.getGeoid(36.104394, 140.085365), 40.3059);

  std::cout << "Testing (35.160410, 139.615526) ... ";
  test(geoid_model.getGeoid(35.160410, 139.615526), 36.7568);

  return 0;
}