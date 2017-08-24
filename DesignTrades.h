#pragma once
#include <string>
#include "../../MatrixSolutions/ControlSystemLibrary/ControlSystemLibrary.h"
#include "../../MatrixSolutions/MatrixLibrary/MatrixLibrary.h"

#define DISTCONVERGED    1.0e-08                // convergence criteria for disturbance analysis
#define TOLDISTURBANCE   1.0e-01                // error degradation for disturbance analysis
#define TOLNOISE         1.0e-02                // error degradation for measurement noiose analysis
#define INFINITY         1.0e+17                // effective value of infinity
#define MAXDISTITERS     50                     // maximum number of disturbance analysis iterations per axis

#include <algorithm>

int main(int argc, char * argv[], char * envp[]);

// MethodName:  brief description 
void ReadData(
  string inFile, 
  string dataFolder, 
  MATRIX_DOUBLE *aircraftDAT, 
  MATRIX_DOUBLE *analysisDAT, 
  MATRIX_DOUBLE *propulsionDAT,
  MATRIX_DOUBLE *simulationDAT);

// MethodName:  brief description
void SystemModel(MATRIX_DOUBLE *aircraftDAT,
  MATRIX_DOUBLE *propulsionDAT,
  SYSTEM_MODEL *sysP,
  int trim,
  SYSTEM_MODEL *sysGv,
  SYSTEM_MODEL *sysGp,
  SYSTEM_MODEL *sysGs);


void trimComputation(
  long double v0, 
  long double h0, 
  long double gamma, 
  MATRIX_DOUBLE *xv0, 
  MATRIX_DOUBLE *uv0);

long double cost(
  MATRIX_DOUBLE *s,
  MATRIX_DOUBLE *x,
  long double gamma,
  long double h,
  MATRIX_DOUBLE *aircraft,
  MATRIX_DOUBLE *Av,
  MATRIX_DOUBLE *B1v,
  MATRIX_DOUBLE *B2v,
  MATRIX_DOUBLE *C1v,
  MATRIX_DOUBLE *D11v,
  MATRIX_DOUBLE *D12v,
  MATRIX_DOUBLE *C2v,
  MATRIX_DOUBLE *D21v,
  MATRIX_DOUBLE *D22v);

void transp(
  MATRIX_DOUBLE *aircraftDAT,
  MATRIX_DOUBLE *propulsionDAT,
  long double time,
  MATRIX_DOUBLE *x,
  MATRIX_DOUBLE *u,
  long double h,
  MATRIX_DOUBLE *xd,
  MATRIX_DOUBLE *Av,
  MATRIX_DOUBLE *B1v,
  MATRIX_DOUBLE *B2v,
  MATRIX_DOUBLE *C1v,
  MATRIX_DOUBLE *D11v,
  MATRIX_DOUBLE *D12v,
  MATRIX_DOUBLE *C2v,
  MATRIX_DOUBLE *D21v,
  MATRIX_DOUBLE *D22v);

void PropulsionSystem(
  MATRIX_DOUBLE *propulsionDAT,
  long double T0,
  long double v0,
  long double dt0,
  long double *PGen0,
  long double *mGen0,
  MATRIX_DOUBLE *Ap,
  MATRIX_DOUBLE *B1p,
  MATRIX_DOUBLE *B2p,
  MATRIX_DOUBLE *C1p,
  MATRIX_DOUBLE *D11p,
  MATRIX_DOUBLE *D12p,
  MATRIX_DOUBLE *C2p,
  MATRIX_DOUBLE *D21p,
  MATRIX_DOUBLE *D22p,
  MATRIX_DOUBLE *H1p,
  MATRIX_DOUBLE *H2p);

void ADC(long double v, 
  long double h, 
  long double *rho, 
  long double *qbar, 
  long double *mach);

void DisturbanceIntensities(SYSTEM_MODEL *sysP, 
  long double v0, 
  long double aoa0, 
  long double omega0,
  MATRIX_DOUBLE *W);

void StaticCRB();

void DynamicAnalysis();

void LQRWeights();

void LQRDesign();

void ControlDesign();