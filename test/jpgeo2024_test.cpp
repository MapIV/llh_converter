#include "llh_converter/jpgeo2024.hpp"
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
  llh_converter::JPGEO2024 geoid_model;
  geoid_model.loadGeoidMap("/usr/share/GSIGEO/JPGEO2024.isg", "/usr/share/GSIGEO/Hrefconv2024.isg");

  std::cout << "Testing (36.104394, 140.085365) ... ";
  test(geoid_model.getGeoid(36.104394, 140.085365), 40.2920);

  std::cout << "Testing (35.160410, 139.615526) ... ";
  test(geoid_model.getGeoid(35.160410, 139.615526), 36.7434);

  std::cout << "Testing (26.591644, 128.072433) ... ";
  test(geoid_model.getGeoid(26.591644, 128.072433), 32.3939);

  return 0;
}
