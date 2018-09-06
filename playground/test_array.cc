// test program for eigen

#include "drake/common/eigen_types.h"
#include <iostream>
#include <map>

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;


void test1()
{
  double array_var[4];

  array_var[0]=10;
  array_var[1]=105;
  array_var[2]=20;
  array_var[3]=5;
  // Vec=Eigen::VectorXd(array_var);
  Eigen::VectorXd Vec(4);
  Vec =Eigen::Map<Eigen::VectorXd>(array_var,4);

  std::cout<<Vec<<std::endl<<std::endl;

  //--------------------------
  array_var[0]=2;
  *array_var = *(Vec.data());
  std::cout<<array_var[0]<<" "<<array_var[2]<<std::endl;
  std::cout<<std::endl;
  //-------------------------

  double array_var2[4]={1,1,-1,-1};
  // array_var={10, 20,30,40};
  Eigen::ArrayXd array_var3=Eigen::Map<Eigen::ArrayXd>(array_var2, 4)
              *Eigen::Map<Eigen::ArrayXd>(array_var, 4);
  std::cout<<array_var3<<std::endl;
  std::cout<<(array_var3.abs() < 2) <<std::endl;
  std::cout<<std::endl;

  //----------------
  int k=4;
  Eigen::Array<bool, Eigen::Dynamic, 1> arrayb;
  arrayb.resize(4);
  arrayb=array_var3<0;
  Eigen::Array<bool, Eigen::Dynamic, 1> arrayb2(k);arrayb2<<true, false, true, false;
  std::cout<<"Test Array boolen:  "<<arrayb.transpose()<<std::endl;arrayb2-=arrayb;
  std::cout<<(arrayb2).transpose()<<std::endl;

  std::cout<<arrayb2.segment(0,1).all()<<std::endl<<std::endl;

  
  // -------------------
  std::vector<double> array_var4(4,13.0);
  Eigen::VectorXd::Map(&array_var4[0], 4)=Vec;
  std::cout<<array_var4[0]<<" "<<array_var4[1]<<" "<<array_var4[2]<<" "<<array_var4[3]
          <<std::endl;

  // ---------------
  std::cout<<"block experiment:\n"<<Eigen::Vector2d::Ones()*2<<std::endl;;
  std::cout<<Eigen::Vector3d(1,0,-1)<<std::endl;

}

void test2()
{
  Eigen::ArrayXd a(4),b(4);
  a<<1,2,3,4;
  b<<-1,5,-1,5;
  std::cout<<a.transpose()<<b.transpose()<<std::endl;
  std::cout<<(a*b).transpose()<<std::endl;
}

void test3()
{
  Eigen::Vector3d a{33,22,11};
  std::cout<<a<<std::endl;

}


int main()
{
  test3();

}