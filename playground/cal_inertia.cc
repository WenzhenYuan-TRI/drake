#include<iostream>
#include "drake/common/eigen_types.h"

#define PRINT_Inertia(a) std::cout << "<"#a">" << a << "</"#a">" <<std::endl;

namespace drake{
namespace multibody{
int DoMain()
{
  double mass, volume, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, rho;
  double ixx, ixy, ixz, iyy, iyz, izz;
  Vector3<double> MassCenter; 
  Vector3<double> FrameOffset;

mass =   0.038  ;
volume = 19.328e-6  ;
Ixx =    52.4784e-10  ;
Iyy=     45.8644e-10 ;
Izz=     18.4765e-10  ;
Ixy=     5.428e-13  ;
Ixz=     4.4377e-11  ;
Iyz=     -2.84e-11  ;
MassCenter << -0.006121e-2, -0.001909e-2,  2.63513e-2;
FrameOffset << 0,0,0;

  MassCenter += FrameOffset;

  rho = mass / volume; 

  ixx = rho * Ixx;
  iyy = rho * Iyy;
  izz = rho * Izz;  
  ixy = rho * Ixy;
  iyz = rho * Iyz;
  ixz = rho * Ixz; 

  PRINT_Inertia(ixx);
  PRINT_Inertia(ixy);
  PRINT_Inertia(ixz);
  PRINT_Inertia(iyy);
  PRINT_Inertia(iyz);
  PRINT_Inertia(izz);

  ixx += MassCenter(0) * MassCenter(0) * mass;
  iyy += MassCenter(1) * MassCenter(1) * mass;
  izz += MassCenter(2) * MassCenter(2) * mass;
  ixy += MassCenter(0) * MassCenter(1) * mass;
  ixz += MassCenter(0) * MassCenter(2) * mass;
  iyz += MassCenter(1) * MassCenter(2) * mass;

  std::cout<<std::endl;

  PRINT_Inertia(ixx);
  PRINT_Inertia(ixy);
  PRINT_Inertia(ixz);
  PRINT_Inertia(iyy);
  PRINT_Inertia(iyz);
  PRINT_Inertia(izz);



return 1;
}

int DoCalculate2Parts()
{
  double mass, volume, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, rho;
  double ixx, ixy, ixz, iyy, iyz, izz;
  Vector3<double> MassCenter; 
  Vector3<double> FrameOffset;

mass =   0.0418  ;
volume = 3.619e-6  ;
Ixx =    34.5988e-10  ;
Iyy=     30.3556e-10 ;
Izz=     16.1337e-10  ;
Ixy=     0 ;
Ixz=     0  ;
Iyz=     -0.00239e-10  ;
MassCenter << 0, 0.005718e-2, 1.2433e-2; 
FrameOffset << 0,0,0;

  MassCenter += FrameOffset;
  rho = mass / volume; 

  ixx = rho * Ixx;
  iyy = rho * Iyy;
  izz = rho * Izz;  
  ixy = rho * Ixy;
  iyz = rho * Iyz;
  ixz = rho * Ixz; 

  ixx += MassCenter(0) * MassCenter(0) * mass;
  iyy += MassCenter(1) * MassCenter(1) * mass;
  izz += MassCenter(2) * MassCenter(2) * mass;
  ixy += MassCenter(0) * MassCenter(1) * mass;
  ixz += MassCenter(0) * MassCenter(2) * mass;
  iyz += MassCenter(1) * MassCenter(2) * mass;

      PRINT_Inertia(ixx);
  PRINT_Inertia(ixy);
  PRINT_Inertia(ixz);
  PRINT_Inertia(iyy);
  PRINT_Inertia(iyz);
  PRINT_Inertia(izz);
 std::cout<<std::endl;



mass =   0.0138  ;
volume = 8.55565e-6  ;
Ixx =    5.69778e-10  ;
Iyy=     5.697670e-10 ;
Izz=     5.619485e-10 ;
Ixy=     0  ;
Ixz=     0  ;
Iyz=     0  ;
MassCenter << 0,  0, -0.129724e-2; 
FrameOffset << 0,0,0.0423;

  MassCenter += FrameOffset;
  rho = mass / volume; 

  ixx += rho * Ixx;
  iyy += rho * Iyy;
  izz += rho * Izz;  
  ixy += rho * Ixy;
  iyz += rho * Iyz;
  ixz += rho * Ixz; 

  ixx += MassCenter(0) * MassCenter(0) * mass;
  iyy += MassCenter(1) * MassCenter(1) * mass;
  izz += MassCenter(2) * MassCenter(2) * mass;
  ixy += MassCenter(0) * MassCenter(1) * mass;
  ixz += MassCenter(0) * MassCenter(2) * mass;
  iyz += MassCenter(1) * MassCenter(2) * mass;


    PRINT_Inertia(ixx);
  PRINT_Inertia(ixy);
  PRINT_Inertia(ixz);
  PRINT_Inertia(iyy);
  PRINT_Inertia(iyz);
  PRINT_Inertia(izz);



  return 1;
}



//   static SpatialInertia MakeFromCentralInertia(const T& mass,
//       const Vector3<T>& p_PScm_E, const RotationalInertia<T>& I_SScm_E)
//    SpatialInertia Shift(const Vector3<T>& p_PQ_E) const {
//     return SpatialInertia(*this).ShiftInPlace(p_PQ_E);
//   }

//   // This has units of kg * m^2
//   double mass1 = ;
//   RotationalInertia<double> I1_G1cm_G1(i1_xx, i1_yy, /* Values from meshlab */);
//   Vector3<double> p_G1oGcm(/* from meshlabs*/)
  

//   SpatialInertia<double> M1_G1o_G1 =
//       MakeFromCentralInertia(mass1, p_G1oGcm, I1_G1cm_G1);


//   // Blah other quantities...


//   SpatialInertia<double> M2_G2o_G2 =
//       MakeFromCentralInertia(mass2, p_G2oGcm, I2_G2cm_G2);



//   // From the SDF we know that B = G1.
//   Vector3<double> p_BoG1o(/*from SDF file*/);
//   Vector3<double> p_BoG2o(/*from SDF file*/);

//   SpatialInertia<double> M_Bo_B = 
//     M1_G1o_G1.Shift(-p_BoG1o) + M2_G2o_G2.Shift(-p_BoG2o);
// }


}} // namespaces

int main() {
  return drake::multibody::DoCalculate2Parts();
}
