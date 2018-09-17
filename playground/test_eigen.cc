// test program for eigen

#include "drake/common/eigen_types.h"
#include <iostream>
#include <map>

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

void Funcs(drake::VectorX<double>& a, drake::VectorX<double>& b)
{
  a.segment(2,3)<<b;
  PRINT_VAR(a);

}

const std::map<std::string, int> Func_mapping()
{
std::map<std::string, int> mapping;
mapping["aaa"]=10;
    mapping["bbb"]=20;
    mapping["ccc"]=30;

    return mapping;
}


int main()
{
  drake::VectorX<double> a(10);
  PRINT_VAR(a.size());
  // PRINT_VAR(a);

  a.segment<4>(2)<<1,2,3,4;
  PRINT_VAR(a);

  // PRINT_VAR(a(9));

  a(0)=std::numeric_limits<double>::quiet_NaN();
  PRINT_VAR(a);
  // PRINT_VAR(Eigen::isnan(a.array(0)));
    // std::cout<<"YES\n";
  // PRINT_VAR(a(0).isnan());

  int bsize=5;
  std::vector<int> b(bsize, 100);
  // b.resize(5);
  // b+=1e3;
  PRINT_VAR(b[0]);
  PRINT_VAR(b[1]);

  // b(0)=std::numeric_limits<double>::quiet_NaN();
  // PRINT_VAR(b);
  // // PRINT_VAR(isnan(b(0)));
  // // PRINT_VAR(isnan(b));


  std::cout<<"-------------------------\n";

  a.setZero();

  drake::VectorX<double> adder(2);
  adder<<10,20;

  Funcs(a, (drake::VectorX<double>(2) << 10, 20).finished());

    std::cout<<"-------------------------\n";

    std::map<std::string, int> mapping=Func_mapping();

    std::cout<<(mapping.find("ccc")==mapping.end())<<(mapping.find("ddd")==mapping.end())<<std::endl;

    for(std::map<std::string,int>::iterator i = mapping.begin(); i!= mapping.end();i++)
      std::cout<<i->first<<std::endl;

}