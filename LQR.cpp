//=====================================================================================
// module 'LQR'
//
// description:FB2F
//  This module performs the linear quadratic regulator design and analysis for the 
//  turboelectric aircraft.
//
// J R Dowdle
// 0.0.0.0
// 14-Aug-2017
//=====================================================================================

// include headers, macros, defined constants, procedure prototypes
#include "stdafx.h"
#include "DesignTrades.h"
#include "LQR.h"

// included namespaces
using namespace std;
using namespace MatrixLibrary;
using namespace ControlSystemLibrary;

// beginning of module 'LQR'

//=====================================================================================
// method 'LQRWeights'
//
// description:
//  This method computes the weights for the performance variables in the LQR problem
//  so that the performance objectives for the control system are met:
//  
//	                -3% <     vtilda/v0     < 3%
//	                -3% <  aoatilda/aoa0    < 3%
//	               -25% < omegatilda/omega0 < 25%
//                 -100 <     dttilda       < 100%
//                  -30 <   detilda (deg)   < 30
//                -100% <  PGentilda/PGen0  < 100% 
//
//  NOTE:  for this analysis, we can capture the first two objectives with
//  the requirement that 
//
//		P1:  ((vtilda/v0)^2 + (aoatilda/aoa0)^2)/(2*(3e-02)^2) < 1
//		P2:  ((omegatilda/omega0)^2)/(0.25)^2 < 1
//
//	the control constraints can be aggregated as
//
//		P3:  ((dttilda)^2 + (detilda/30)^2 + (PGentilda/PGen0)^2)/3 < 1
//
//  then P1, P2, P3 can be combined into a cost:
//
//      J = (P1^2 + P2^2 + P3^2)/3 < 1
//
//  Note that z = [vtilda aoatilda dttilda detilda omegatilda (PGentilda/PGen0)]'
//
//  so J can be written as:
//
//      J = z' LAMBDA z, with
//
//      LAMBDA = diag(LAMBDA00; LAMBDA11; LAMBDA22; LAMBDA33; LAMBDA44; LAMBDA55)
//
//  and
//  
//      LAMBDA00 = 1/(2 DELV^2);          DELV = 3% v0
//      LAMBDA11 = 1/(2 DELAOA^2);        DELVAOA = 3% aoa0
//      LAMBDA22 = 1/(3 DTMAX^2);         DTMAX = 1
//      LAMBDA33 = 1/(3 DEMAX^2);         DEMAX = 30 (degrees)
//      LAMBDA44 = 1/(OMEGAMAX^2);        OMEGAMAX = 25% omega0
//      LAMBDA55 = 1/(3 DELPG^2);         DELPG = 1 (NOTE:  the power variable in z is already 
//                                        normalized, so this reflects a normalized variation)
//
// the goal is to design an LQR controller that results in J < 1
//
// to achieve this, lqr weights are required to penalize state and control energy to suitably 
// affect the state and control trajectories; these weights are in the weigting vector Q
// an objective is to minimize actuator bandwidth requirements as well (i.e., bw < 200Hz)
// therefore, J = 1e4 if bw > 200Hz
//
// J R Dowdle
// 26-Aug-2017
//=====================================================================================

// beginning of method 'LQRWeights'

void LQRWeights(
  SYSTEM_MODEL  *sysP,                          // input system
  MATRIX_DOUBLE *W,                             // input disturbance intensity
  MATRIX_DOUBLE *Q,                             // output lqr weights
  long double ThrustMaxBW,                      // limit on thrust loop bandwidth
  long double ElevatorMaxBW,                    // limit on elevator loop bandwidth
  long double PropulsionMaxBW)                  // limit on propulsion loop bandwidth
{
  // executable code

  // allocate space
  int n = sysP->xdim;
  int m1 = sysP->wdim;
  int m2 = sysP->udim;
  int r1 = sysP->zdim;
  int r2 = sysP->ydim;
  int N = 20;

  MATRIX_DOUBLE *LAMBDA = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(LAMBDA, "LAMBDA", 0, 6, 6);
  MATRIX_DOUBLE *F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(F, "F", 0, m2, n);
  MATRIX_DOUBLE *lqrBW = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(lqrBW, "lqrBW", 0, 1, m2);
  MATRIX_DOUBLE *X = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(X, "X", 0, n, n);
  MATRIX_DOUBLE *Z = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z, "Z", 0, r1, r1);
  MATRIX_DOUBLE *R0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(R0, "R0", 0, 25, 1);
  MATRIX_DOUBLE *delJ = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(delJ, "delJ", 0, Q->rows, 1);

  // set reference conditions on v0, aoa0, omega0
  long double v0 = sysP->xref->real[0][0];
  long double aoa0 = sysP->xref->real[1][0];
  long double omega0 = sysP->xref->real[4][0];

  // initialize the matrix LAMBDA
  //
  // requirements
  //  (1) Thrust loop bandwidth < 5 Hz
  //  (2) Elevator loop bandwidth < 5 Hz
  //  (3) Propulsion shaft rate loop bandwidth < 20 Hz
  //  (4) Normalized velocity variation < 3%
  //  (5) Normalized angle-of-attack variation < 3%
  //  (6) Normalized propulsion shaft rate variation < 25%
  //  (7) Normalized thrust variation < 10%
  //  (8) Normalized power generation < 10%
  MatrixToolbox::zeros(LAMBDA);
  LAMBDA->real[0][0] = 1/(2*DELV*DELV);
  LAMBDA->real[1][1] = 1/(2*DELAOA*DELAOA);
  LAMBDA->real[2][2] = 1/(3*DTMAX*DTMAX);
  LAMBDA->real[3][3] = 1/(3*DEMAX*DEMAX);
  LAMBDA->real[4][4] = 1/(OMEGAMAX*OMEGAMAX);
  LAMBDA->real[5][5] = 1/(3*DELPG*DELPG);

  // set evaluation weights (read in)
  long double Q0WGT0 = Q->real[0][0];
  long double Q1WGT0 = Q->real[1][0];
  long double Q2WGT0 = Q->real[2][0];
  long double Q3WGT0 = Q->real[3][0];
  long double Q4WGT0 = Q->real[4][0];
  long double Q5WGT0 = Q->real[5][0];

  // compute LQR
  long double performancev;
  long double performanceaoa;
  long double performanceomega;
  long double performancedt;
  long double performancede;
  long double performancePGen;
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  long double J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);

  cout << "Initial Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl << endl;

  // vary PGen control weight to find feasible solution
  long double Jopt = J;
  long double RScale = 1;
  MatrixToolbox::logspace(-8, 8, R0);
  for (int i = 0; i < R0->rows; i++)
  {
    // set Q[i][0]
    Q->real[0][0] = Q0WGT0;
    Q->real[1][0] = Q1WGT0;
    Q->real[2][0] = Q2WGT0;
    Q->real[3][0] = Q3WGT0;
    Q->real[4][0] = Q4WGT0;
    Q->real[5][0] = Q5WGT0*R0->real[R0->rows - 1 - i][0];

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Jopt = J;
      RScale = R0->real[R0->rows - 1 - i][0];
    }
  }
  // scale control inputs
  Q->real[0][0] = Q0WGT0;
  Q->real[1][0] = Q1WGT0;
  Q->real[2][0] = Q2WGT0;
  Q->real[3][0] = Q3WGT0;
  Q->real[4][0] = Q4WGT0;
  Q->real[5][0] = Q5WGT0*RScale;

  Q0WGT0 = Q->real[0][0];
  Q1WGT0 = Q->real[1][0];
  Q2WGT0 = Q->real[2][0];
  Q3WGT0 = Q->real[3][0];
  Q4WGT0 = Q->real[4][0];
  Q5WGT0 = Q->real[5][0];
  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Power Control Weight Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl;
  cout << setw(20) << "RScale:" << setw(20) << scientific << setprecision(8) << RScale << endl << endl;

  // vary Thrust control weight to find feasible solution
  RScale = 1;
  MatrixToolbox::logspace(-8, 8, R0);
  for (int i = 0; i < R0->rows; i++)
  {
    // set Q[i][0]
    Q->real[0][0] = Q0WGT0;
    Q->real[1][0] = Q1WGT0;
    Q->real[2][0] = Q2WGT0*R0->real[R0->rows - 1 - i][0];
    Q->real[3][0] = Q3WGT0;
    Q->real[4][0] = Q4WGT0;
    Q->real[5][0] = Q5WGT0;;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Jopt = J;
      RScale = R0->real[R0->rows - 1 - i][0];
    }
  }
  // scale control inputs
  Q->real[0][0] = Q0WGT0;
  Q->real[1][0] = Q1WGT0;
  Q->real[2][0] = Q2WGT0*RScale;
  Q->real[3][0] = Q3WGT0;
  Q->real[4][0] = Q4WGT0;
  Q->real[5][0] = Q5WGT0;

  Q0WGT0 = Q->real[0][0];
  Q1WGT0 = Q->real[1][0];
  Q2WGT0 = Q->real[2][0];
  Q3WGT0 = Q->real[3][0];
  Q4WGT0 = Q->real[4][0];
  Q5WGT0 = Q->real[5][0];
  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Thrust Control Weight Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl;
  cout << setw(20) << "RScale:" << setw(20) << scientific << setprecision(8) << RScale << endl << endl;

  // vary Elevator control weight to find feasible solution
  RScale = 1;
  MatrixToolbox::logspace(-8, 8, R0);
  for (int i = 0; i < R0->rows; i++)
  {
    // set Q[i][0]
    Q->real[0][0] = Q0WGT0;
    Q->real[1][0] = Q1WGT0;
    Q->real[2][0] = Q2WGT0;
    Q->real[3][0] = Q3WGT0*R0->real[R0->rows - 1 - i][0];
    Q->real[4][0] = Q4WGT0;
    Q->real[5][0] = Q5WGT0;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Jopt = J;
      RScale = R0->real[R0->rows - 1 - i][0];
    }
  }
  // scale control inputs
  Q->real[0][0] = Q0WGT0;
  Q->real[1][0] = Q1WGT0;
  Q->real[2][0] = Q2WGT0;
  Q->real[3][0] = Q3WGT0*RScale;
  Q->real[4][0] = Q4WGT0;
  Q->real[5][0] = Q5WGT0;

  Q0WGT0 = Q->real[0][0];
  Q1WGT0 = Q->real[1][0];
  Q2WGT0 = Q->real[2][0];
  Q3WGT0 = Q->real[3][0];
  Q4WGT0 = Q->real[4][0];
  Q5WGT0 = Q->real[5][0];
  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Elevator Control Weight Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl;
  cout << setw(20) << "RScale:" << setw(20) << scientific << setprecision(8) << RScale << endl << endl;

  // vary Velocity state weight to find feasible solution
  RScale = 1;
  MatrixToolbox::logspace(-8, 8, R0);
  for (int i = 0; i < R0->rows; i++)
  {
    // set Q[i][0]
    Q->real[0][0] = Q0WGT0*R0->real[R0->rows - 1 - i][0];
    Q->real[1][0] = Q1WGT0;
    Q->real[2][0] = Q2WGT0;
    Q->real[3][0] = Q3WGT0;
    Q->real[4][0] = Q4WGT0;
    Q->real[5][0] = Q5WGT0;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Jopt = J;
      RScale = R0->real[R0->rows - 1 - i][0];
    }
  }
  // scale control inputs
  Q->real[0][0] = Q0WGT0*RScale;
  Q->real[1][0] = Q1WGT0;
  Q->real[2][0] = Q2WGT0;
  Q->real[3][0] = Q3WGT0;
  Q->real[4][0] = Q4WGT0;
  Q->real[5][0] = Q5WGT0;

  Q0WGT0 = Q->real[0][0];
  Q1WGT0 = Q->real[1][0];
  Q2WGT0 = Q->real[2][0];
  Q3WGT0 = Q->real[3][0];
  Q4WGT0 = Q->real[4][0];
  Q5WGT0 = Q->real[5][0];
  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Velocity State Weight Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl;
  cout << setw(20) << "RScale:" << setw(20) << scientific << setprecision(8) << RScale << endl << endl;

  // vary Angle-of-Attack state weight to find feasible solution
  RScale = 1;
  MatrixToolbox::logspace(-8, 8, R0);
  for (int i = 0; i < R0->rows; i++)
  {
    // set Q[i][0]
    Q->real[0][0] = Q0WGT0;
    Q->real[1][0] = Q1WGT0*R0->real[R0->rows - 1 - i][0];
    Q->real[2][0] = Q2WGT0;
    Q->real[3][0] = Q3WGT0;
    Q->real[4][0] = Q4WGT0;
    Q->real[5][0] = Q5WGT0;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Jopt = J;
      RScale = R0->real[R0->rows - 1 - i][0];
    }
  }
  // scale control inputs
  Q->real[0][0] = Q0WGT0;
  Q->real[1][0] = Q1WGT0*RScale;
  Q->real[2][0] = Q2WGT0;
  Q->real[3][0] = Q3WGT0;
  Q->real[4][0] = Q4WGT0;
  Q->real[5][0] = Q5WGT0;

  Q0WGT0 = Q->real[0][0];
  Q1WGT0 = Q->real[1][0];
  Q2WGT0 = Q->real[2][0];
  Q3WGT0 = Q->real[3][0];
  Q4WGT0 = Q->real[4][0];
  Q5WGT0 = Q->real[5][0];
  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Angle-of-Attack State Weight Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl;
  cout << setw(20) << "RScale:" << setw(20) << scientific << setprecision(8) << RScale << endl << endl;

  // vary Propulsion shaft angular rate state weight to find feasible solution
  RScale = 1;
  MatrixToolbox::logspace(-8, 8, R0);
  for (int i = 0; i < R0->rows; i++)
  {
    // set Q[i][0]
    Q->real[0][0] = Q0WGT0;
    Q->real[1][0] = Q1WGT0;
    Q->real[2][0] = Q2WGT0;
    Q->real[3][0] = Q3WGT0;
    Q->real[4][0] = Q4WGT0*R0->real[R0->rows - 1 - i][0];
    Q->real[5][0] = Q5WGT0;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Jopt = J;
      RScale = R0->real[R0->rows - 1 - i][0];
    }
  }
  // scale control inputs
  Q->real[0][0] = Q0WGT0;
  Q->real[1][0] = Q1WGT0;
  Q->real[2][0] = Q2WGT0;
  Q->real[3][0] = Q3WGT0;
  Q->real[4][0] = Q4WGT0*RScale;
  Q->real[5][0] = Q5WGT0;

  Q0WGT0 = Q->real[0][0];
  Q1WGT0 = Q->real[1][0];
  Q2WGT0 = Q->real[2][0];
  Q3WGT0 = Q->real[3][0];
  Q4WGT0 = Q->real[4][0];
  Q5WGT0 = Q->real[5][0];
  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Propulsion Shaft Angular Velocity State Weight Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl;
  cout << setw(20) << "RScale:" << setw(20) << scientific << setprecision(8) << RScale << endl << endl;

  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  cout << "Coarse Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl << endl;

  // perform fine search for lqr weights
  Jopt = J;
  long double delta = 1.0e-04;
  long double beta = 1.0e-03;

  // using above set of weights as a starting point, perform variation to satisfy the requirements
  for (int i = 0; i < 100; i++)
  {
    // compute nominal cost

    // set weights
    Q->real[0][0] = Q0WGT0;
    Q->real[1][0] = Q1WGT0;
    Q->real[2][0] = Q2WGT0;
    Q->real[3][0] = Q3WGT0;
    Q->real[4][0] = Q4WGT0;
    Q->real[5][0] = Q5WGT0;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);

    // compute derivative of cost
    for (int j = 0; j < 6; j++)
    {
      // set jth weight to compute derivative
      Q->real[j][0] = (1 + delta)*Q->real[j][0];
      LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
      long double Jp = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
      delJ->real[j][0] = (Jp - J)/(delta*(Q->real[j][0]));

      // set weights for next loop
      Q->real[0][0] = Q0WGT0;
      Q->real[1][0] = Q1WGT0;
      Q->real[2][0] = Q2WGT0;
      Q->real[3][0] = Q3WGT0;
      Q->real[4][0] = Q4WGT0;
      Q->real[5][0] = Q5WGT0;
    }

    // update weights for next iteration - taylor series - 0 = J(Q) + delJ(Q)*delQ implies delQ = -J(Q)/delJ(Q)
    long double NormdelJsq = MatrixToolbox::norm("2", delJ);
    Q->real[0][0] = (Q0WGT0 - beta*J*delJ->real[0][0]/NormdelJsq) > 0 ? (Q0WGT0 - beta*J*delJ->real[0][0]/NormdelJsq) : Q0WGT0;
    Q->real[1][0] = (Q1WGT0 - beta*J*delJ->real[1][0]/NormdelJsq) > 0 ? (Q1WGT0 - beta*J*delJ->real[1][0]/NormdelJsq) : Q1WGT0;
    Q->real[2][0] = (Q2WGT0 - beta*J*delJ->real[2][0]/NormdelJsq) > 0 ? (Q2WGT0 - beta*J*delJ->real[2][0]/NormdelJsq) : Q2WGT0;
    Q->real[3][0] = (Q3WGT0 - beta*J*delJ->real[3][0]/NormdelJsq) > 0 ? (Q3WGT0 - beta*J*delJ->real[3][0]/NormdelJsq) : Q3WGT0;
    Q->real[4][0] = (Q4WGT0 - beta*J*delJ->real[4][0]/NormdelJsq) > 0 ? (Q4WGT0 - beta*J*delJ->real[4][0]/NormdelJsq) : Q4WGT0;
    Q->real[5][0] = (Q5WGT0 - beta*J*delJ->real[5][0]/NormdelJsq) > 0 ? (Q5WGT0 - beta*J*delJ->real[5][0]/NormdelJsq) : Q5WGT0;

    // compute LQR
    LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
    // compute cost
    J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
    if ((J > 0) && (J < Jopt))
    {
      Q0WGT0 = Q->real[0][0];
      Q1WGT0 = Q->real[1][0];
      Q2WGT0 = Q->real[2][0];
      Q3WGT0 = Q->real[3][0];
      Q4WGT0 = Q->real[4][0];
      Q5WGT0 = Q->real[5][0];
      Jopt = J;
    }
    else if ((J > Jopt) && (J > 0) && (beta > 2*EVZ))
    {
      beta = beta/2;
    }
    else
      break;
  }
  // set weights
  Q->real[0][0] = Q0WGT0;
  Q->real[1][0] = Q1WGT0;
  Q->real[2][0] = Q2WGT0;
  Q->real[3][0] = Q3WGT0;
  Q->real[4][0] = Q4WGT0;
  Q->real[5][0] = Q5WGT0;

  // compute LQR
  LQRDesign(sysP, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);
  // compute cost
  J = LQRCost(LAMBDA, Z, lqrBW, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  cout << "Fine Solution:" << endl;
  cout << setw(0) << "***** J:" << setw(20) << scientific << setprecision(8) << J << endl;
  MatrixToolbox::display(Q);
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) << "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) << "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) << "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) << "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) << "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) << "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl << endl;

  // free allocated space
  MatrixToolbox::destroy(LAMBDA);
  MatrixToolbox::destroy(F);
  MatrixToolbox::destroy(lqrBW);
  MatrixToolbox::destroy(X);
  MatrixToolbox::destroy(Z);
  MatrixToolbox::destroy(R0);
  MatrixToolbox::destroy(delJ);
}

//=====================================================================================
// end of method 'LQRWeights'
//=====================================================================================

//=====================================================================================
// method 'LQRDesign'
//
// description:
//  This method designs the lqr controller once the weights are selected.
//
// J R Dowdle
// 26-Aug-2017
//=====================================================================================

// beginning of method 'LQRDesign'

void LQRDesign(
  SYSTEM_MODEL  *sysP,                          // input system
  MATRIX_DOUBLE *W,                             // input disturbance intensity
  MATRIX_DOUBLE *Q,                             // output lqr weights
  MATRIX_DOUBLE *F,                             // lqr gain
  MATRIX_DOUBLE *lqrBW,                         // lqr loop bandwidths
  MATRIX_DOUBLE *X,                             // lqr closed-loop state covariance matrix
  MATRIX_DOUBLE *Z,                             // lqr closed-loop performance covariance matrix
  long double   *performancev,                  // velocity performance
  long double   *performanceaoa,                // angle-of-attack performance
  long double   *performanceomega,              // propulsion shaft angular rate performance
  long double   *performancedt,                 // thrust control performance
  long double   *performancede,                 // elevator control performance
  long double   *performancePGen)               // power generator control performance
{
  // executable code

  // allocate space
  int n = sysP->xdim;
  int m1 = sysP->wdim;
  int m2 = sysP->udim;
  int r1 = sysP->zdim;
  int r2 = sysP->ydim;
  SYSTEM_MODEL *sysG = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysG, "sysG", n, m1, m2, r1, r2);
  MATRIX_DOUBLE *B2F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2F, "B2F", 0, n, n);
  MATRIX_DOUBLE *Acl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Acl, "Acl", 0, n, n);
  MATRIX_DOUBLE *B1W = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1W, "B1W", 0, n, m1);
  MATRIX_DOUBLE *B1t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1t, "B1t", 0, m1, n);
  MATRIX_DOUBLE *B1WB1t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1WB1t, "B1WB1t", 0, n, n);
  MATRIX_DOUBLE *D12F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12F, "D12F", 0, r1, n);
  MATRIX_DOUBLE *Ccl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Ccl, "Ccl", 0, r1, n);
  MATRIX_DOUBLE *CclX = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(CclX, "CclX", 0, r1, n);
  MATRIX_DOUBLE *Cclt = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Cclt, "Cclt", 0, n, r1);

  // set reference conditions on v0, aoa0, omega0
  long double v0 = sysP->xref->real[0][0];
  long double aoa0 = sysP->xref->real[1][0];
  long double omega0 = sysP->xref->real[4][0];
  long double PGen0 = sysP->uref->real[2][0];

  // modify model to include weights Q
  MatrixToolbox::equate(sysP->A, sysG->A);
  MatrixToolbox::equate(sysP->B1, sysG->B1);
  MatrixToolbox::equate(sysP->B2, sysG->B2);
  MatrixToolbox::equate(sysP->C1, sysG->C1);
  MatrixToolbox::equate(sysP->C2, sysG->C2);
  MatrixToolbox::equate(sysP->D11, sysG->D11);
  MatrixToolbox::equate(sysP->D12, sysG->D12);
  MatrixToolbox::equate(sysP->D21, sysG->D21);
  MatrixToolbox::equate(sysP->D22, sysG->D22);
  sysG->C1->real[0][0] = sysP->C1->real[0][0]*Q->real[0][0];
  sysG->C1->real[1][1] = sysP->C1->real[1][1]*Q->real[1][0];
  sysG->C1->real[4][4] = sysP->C1->real[4][4]*Q->real[2][0];
  sysG->D12->real[2][0] = sysP->D12->real[2][0]*Q->real[3][0];
  sysG->D12->real[3][1] = sysP->D12->real[3][1]*Q->real[4][0];
  sysG->D12->real[5][2] = sysP->D12->real[5][2]*Q->real[5][0];

  // compute the lqr control system with weights Q
  ControlToolbox::lqr(sysG, F);

  // compute bandwidths of lqr control loops
  ControlToolbox::lqrbw(sysG, F, lqrBW);

  // compute closed-loop lqr
  MatrixToolbox::multiply(sysP->B2, F, B2F);
  MatrixToolbox::add(sysP->A, B2F, Acl);

  // compute covariance of lqr control system
  MatrixToolbox::multiply(sysP->B1, W, B1W);
  MatrixToolbox::transpose(sysP->B1, B1t);
  MatrixToolbox::multiply(B1W, B1t, B1WB1t);
  ControlToolbox::lyap(Acl, B1WB1t, X);

  // compute the covariance of z, Z, and performance variables
  MatrixToolbox::multiply(sysP->D12, F, D12F);
  MatrixToolbox::add(sysP->C1, D12F, Ccl);
  MatrixToolbox::multiply(Ccl, X, CclX);
  MatrixToolbox::transpose(Ccl, Cclt);
  MatrixToolbox::multiply(CclX, Cclt, Z);
  *performancev = sqrt(Z->real[0][0])/v0;
  *performanceaoa = sqrt(Z->real[1][1])/aoa0;
  *performanceomega = sqrt(Z->real[4][4])/omega0;
  *performancedt = sqrt(Z->real[2][2]);
  *performancede = sqrt(Z->real[3][3])/30;
  *performancePGen = sqrt(Z->real[5][5])/PGen0;
}

//=====================================================================================
// end of method 'LQRDesign'
//=====================================================================================

//=====================================================================================
// method 'LQRCost'
//
// description:
//  This method computes the cost used in selection of the LQR weights.
//
// J R Dowdle
// 26-Aug-2017
//=====================================================================================

// beginning of method 'LQRCost'

long double LQRCost(
  MATRIX_DOUBLE *LAMBDA,                        // lqr cost performance weighting matrix
  MATRIX_DOUBLE *Z,                             // lqr closed-loop performance covariance matrix
  MATRIX_DOUBLE *lqrBW,                         // lqr loop bandwidths
  long double ThrustMaxBW,                      // limit on thrust loop bandwidth
  long double ElevatorMaxBW,                    // limit on elevator loop bandwidth
  long double PropulsionMaxBW)                  // limit on propulsion loop bandwidth
{
  int r1 = LAMBDA->rows;
  MATRIX_DOUBLE *LZ = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(LZ, "LZ", 0, r1, r1);
  MatrixToolbox::multiply(LAMBDA, Z, LZ);
  for (int i = 0; i < r1; i++)
  {
    if (Z->real[i][i] < 0)
      return(INFINITY);
  }
  long double J = MatrixToolbox::trace(LZ).real;
  if ((lqrBW->real[0][0] > ThrustMaxBW*2*acos(-1)) || (lqrBW->real[0][1] > ElevatorMaxBW*2*acos(-1)) || (lqrBW->real[0][2] > PropulsionMaxBW*2*acos(-1)))
    J = J + INFINITY;
  MatrixToolbox::destroy(LZ);
  return(J);
}

//=====================================================================================
// end of module 'LQRCost'
//=====================================================================================

//=====================================================================================
// method 'DynamicLQR'
//
// description:
//   This method performs an analysis of the turboelectric aircraft based 
//   on the model
//
//    xdot =  A x +  B1 w +  B2 u
//       z = C1 x + D11 w + D12 u
//       y = C2 x + D21 w + D22 u
//
//   with u computed as an idealized LQR state feedback controller to assess achievable 
//   closed-loop performance
//
// J R Dowdle
// 01-Sep-2017
//=====================================================================================

// beginning of method 'DynamicLQR'

void DynamicLQR(
  SYSTEM_MODEL  *sysP,                          // input system
  MATRIX_DOUBLE *F,                             // lqr gain
  MATRIX_DOUBLE *K,                             // kalman filter gain
  MATRIX_DOUBLE *W,                             // input disturbance intensity
  MATRIX_DOUBLE *disturbance,                   // deterministic disturbance vector
  long double   pulseStart,                     // beginning of disturbance pulse
  long double   pulseLength,                    // duration of disturbance pulse
  MATRIX_DOUBLE *stime,                         // time vector for solutions
  MATRIX_DOUBLE *vlqr,                          // lqr velocity response to disturbance 
  MATRIX_DOUBLE *aoalqr,                        // lqr angle-of-attack response to disturbance 
  MATRIX_DOUBLE *qlqr,                          // lqr pitch rate response to disturbance 
  MATRIX_DOUBLE *omegalqr,                      // lqr propulsion shaft angular velocity response to disturbance 
  MATRIX_DOUBLE *SIGvlqr,                       // lqr velocity standard deviation response to disturbance 
  MATRIX_DOUBLE *SIGaoalqr,                     // lqr angle-of-attack standard deviation response to disturbance
  MATRIX_DOUBLE *SIGqlqr,                       // lqr pitch rate standard deviation response to disturbance
  MATRIX_DOUBLE *SIGomegalqr,                   // lqr propulsion shaft angular velocity standard deviation response to disturbance
  MATRIX_DOUBLE *vlqrkf,                        // kalman filter mean velocity estimate
  MATRIX_DOUBLE *aoalqrkf,                      // kalman filter mean angle-of-attack estimate
  MATRIX_DOUBLE *qlqrkf,                        // kalman filter mean pitch rate estimate
  MATRIX_DOUBLE *omegalqrkf,                    // kalman filter mean propulsion shaft angular rate estimate
  MATRIX_DOUBLE *SIGvlqrkf,                     // kalman filter standard deviation of velocity estimation error
  MATRIX_DOUBLE *SIGaoalqrkf,                   // kalman filter standard deviation of angle-of-attack estimation error
  MATRIX_DOUBLE *SIGqlqrkf,                     // kalman filter standard deviation of pitch rate estimation error
  MATRIX_DOUBLE *SIGomegalqrkf)                 // kalman filter standard deviation of propulsion shaft angular velocity estimation error
{
  // executable code

  // allocate space
  int n = sysP->xdim;
  int m1 = sysP->wdim;
  int r1 = sysP->zdim;
  int r2 = sysP->ydim;
  MATRIX_DOUBLE *wv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(wv, "wv", 0, m1, stime->rows);
  MATRIX_DOUBLE *Wv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv, "Wv", 0, m1, stime->rows);
  MATRIX_DOUBLE *B2F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2F, "B2F", 0, n, n);
  MATRIX_DOUBLE *Acl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Acl, "Acl", 0, n, n);
  MATRIX_DOUBLE *D12F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12F, "D12F", 0, r1, n);
  MATRIX_DOUBLE *C1cl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C1cl, "C1cl", 0, r1, n);
  MATRIX_DOUBLE *D22F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D22F, "D22F", 0, r2, n);
  MATRIX_DOUBLE *C2cl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2cl, "C2cl", 0, r2, n);
  MATRIX_DOUBLE *null = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(null, "null", 0, 0, 0);
  SYSTEM_MODEL *sysG = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysG, "sysG", n, m1, 0, r1, r2);
  MATRIX_DOUBLE *xv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(xv, "xv", 0, n, stime->rows);
  MATRIX_DOUBLE *Xv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Xv, "Xv", 0, n, stime->rows);
  MATRIX_DOUBLE *ev = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(ev, "ev", 0, n, stime->rows);
  MATRIX_DOUBLE *Ev = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Ev, "Ev", 0, n, stime->rows);

  // set up simulation variables
  for (int k = 0; k < stime->rows; k++)
  {
    if ((stime->real[k][0] > pulseStart) && (stime->real[k][0] <= (pulseStart + pulseLength)))
    {
      for (int i = 0; i < m1; i++)
      {
        wv->real[i][k] = disturbance->real[i][0];
      }
      for (int i = 0; i < m1; i++)
      {
        Wv->real[i][k] = W->real[i][0];
      }
    }
  }
  // compute closed-loop lqr system model
  MatrixToolbox::multiply(sysP->B2, F, B2F);
  MatrixToolbox::add(sysP->A, B2F, Acl);
  MatrixToolbox::multiply(sysP->D12, F, D12F);
  MatrixToolbox::add(sysP->C1, D12F, C1cl);
  MatrixToolbox::multiply(sysP->D22, F, D22F);
  MatrixToolbox::add(sysP->C2, D22F, C2cl);
  ControlToolbox::ss2sys(sysG, Acl, sysP->B1, null, C1cl, C2cl, sysP->D11, null, sysP->D21, null);

  // compute mean and variance of state vector
  ControlToolbox::SystemSimulation(sysG, stime, wv, Wv, xv, Xv);

  // unwind variables into output vectors
  for (int i = 0; i < stime->rows; i++)
  {
    vlqr->real[i][0] = xv->real[0][i];
    aoalqr->real[i][0] = xv->real[1][i];
    qlqr->real[i][0] = xv->real[3][i];
    omegalqr->real[i][0] = xv->real[4][i];
    SIGvlqr->real[i][0] = sqrt(Xv->real[0][i]);
    SIGaoalqr->real[i][0] = sqrt(Xv->real[1][i]);
    SIGqlqr->real[i][0] = sqrt(Xv->real[3][i]);
    SIGomegalqr->real[i][0] = sqrt(Xv->real[4][i]);
  }

  // compute the kalman filter error analysis vs time
  ControlToolbox::KFSimulation(sysG, K, W, stime, wv, Wv, ev, Ev);

  // unwind variables into output vectors
  for (int i = 0; i < stime->rows; i++)
  {
    vlqrkf->real[i][0] = ev->real[0][i];
    aoalqrkf->real[i][0] = ev->real[1][i];
    qlqrkf->real[i][0] = ev->real[3][i];
    omegalqrkf->real[i][0] = ev->real[4][i];
    SIGvlqrkf->real[i][0] = sqrt(Ev->real[0][i]);
    SIGaoalqrkf->real[i][0] = sqrt(Ev->real[1][i]);
    SIGqlqrkf->real[i][0] = sqrt(Ev->real[3][i]);
    SIGomegalqrkf->real[i][0] = sqrt(Ev->real[4][i]);
  }

  // free allocated space
  MatrixToolbox::destroy(wv);
  MatrixToolbox::destroy(Wv);
  MatrixToolbox::destroy(xv);
  MatrixToolbox::destroy(Xv);
  MatrixToolbox::destroy(ev);
  MatrixToolbox::destroy(Ev);
}

//=====================================================================================
// end of method 'DynamicLQR'
//=====================================================================================

//=====================================================================================
// end of module 'LQR'
//=====================================================================================