//=======================================================================================
// header 'DesignTrades'
//
// description:
//  This header contains headers, macros, defined constants and prototypes for the 
//  DesignTrades analysis module.
//
// J R Dowdle
// 25-Aug-2017
//=======================================================================================

// include headers, macros, defined constants, procedure prototypes

#pragma once

#include "../../MatrixSolutions/ControlSystemLibrary/ControlSystemLibrary.h"
#include "../../MatrixSolutions/MatrixLibrary/MatrixLibrary.h"

#define EVZ              pow(2,-52)             // effective value of zero
#define INFINITY         pow(2, 52)             // effective value of infinity

// define DesignTrades method prototypes

// main:  main method for DesignTrades module
int main(
  int           argc,                           // number of arguments to main
  char          *argv[],                        // inputs to main
  char          *envp[]);                       // environmental input to main

// ReadData:  reads input data for the CRB analysis of the turboelectric aircraft 
void ReadData(
  string        inFile,                         // input file name
  string        dataFolder,                     // data folder name
  MATRIX_DOUBLE *aircraftDAT,                   // array of aircraft data inputs
  MATRIX_DOUBLE *analysisDAT,                   // array of analysis data inputs
  MATRIX_DOUBLE *propulsionDAT,                 // array of propulsion data inputs
  MATRIX_DOUBLE *simulationDAT,                 // array of simulation data inputs
  MATRIX_DOUBLE *LQRDAT);                       // array of LQR data inputs

//=======================================================================================
// end of header 'DesignTrades'
//=======================================================================================