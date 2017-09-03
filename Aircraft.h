//=======================================================================================
// header 'Aircraft'
//
// description:
//  This header contains headers, macros, defined constants and prototypes for the 
//  Aircraft simulation module.
//
// J R Dowdle
// 25-Aug-2017
//=======================================================================================

// include headers, macros, defined constants, procedure prototypes

#pragma once

#include "../../MatrixSolutions/ControlSystemLibrary/ControlSystemLibrary.h"

// define Aircraft module prototypes

// SystemModel:  creates an integrated system model of the turboelectric aircraft
void SystemModel(
  MATRIX_DOUBLE *aircraftDAT,                   // array of aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // array of propulsion data
  int           trim,                           // trim condition (1, 2, 3)
  SYSTEM_MODEL  *sysGv,                         // system model of vehicle
  SYSTEM_MODEL  *sysGp,                         // system model of propulsion system
  SYSTEM_MODEL  *sysGs);                        // system model of integrated system

// trimComputation:  computes the trim conditions
void trimComputation(
  long double   v,                              // input trim velocity
  long double   h,                              // input trim altitude
  long double   gamma,                          // input trim flight path angle
  MATRIX_DOUBLE *xv,                            // output trim state of vehicle
  MATRIX_DOUBLE *uv,                            // output trim control of vehicle
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT);                // propulsion data

// cost: computes cost criteria for trim computation
long double cost(
  MATRIX_DOUBLE *s,                             // optimization variable
  MATRIX_DOUBLE *x,                             // vehicle state
  long double   gamma,                          // vehicle flight path angle
  long double   h,                              // vehicle alitude
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT);                // propulsion data

// trimSearch: computes trim condition via search algorithm
void trimSearch(
  long double   h,                              // input trim altitude
  long double   gamma,                          // input trim flight path angle
  MATRIX_DOUBLE *xv,                            // output trim state of vehicle
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  long double   thrustmin,                      // minimum thrust
  long double   thrustmax,                      // maximum thrust
  long double   elevatormin,                    // minimum elevator
  long double   elevatormax,                    // maximum elevator
  long double   aoamin,                         // minimum angle-of-attack
  long double   aoamax,                         // maximum angle-of-attack
  long double   NSearch,                        // number of points search per axis
  long double   *trim_thrust,                   // thrust at trim
  long double   *trim_elevator,                 // elevator at trim
  long double   *trim_aoa,                      // angle-of-attack at trim
  long double   *f_trim);                       // cost at trim


 // trimGradient:  computes gradient of cost near the trim condition
void trimGradient(
  long double   h,                              // input trim altitude
  long double   gamma,                          // input trim flight path angle
  MATRIX_DOUBLE *xv,                            // output trim state of vehicle
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  long double   trim_thrust,                    // thrust at trim
  long double   trim_elevator,                  // elevator at trim
  long double   trim_aoa,                       // angle-of-attack at trim
  long double   del_thrust,                     // thrust perturbation for computing derivative
  long double   del_elevator,                   // elevator perturbation for computing derivative
  long double   del_aoa,                        // angle-of-attack perturbation for computing derivative
  MATRIX_DOUBLE *gradient);                     // gradient of cost

// aircraftDyn:  creates dynamic model of the aircraft
void aircraftDyn(
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  long double   stime,                          // simulation time
  MATRIX_DOUBLE *xv,                            // output trim state of vehicle
  MATRIX_DOUBLE *uv,                            // output trim control of vehicle
  long double   h,                              // vehicle alitude
  MATRIX_DOUBLE *xd,                            // derivative of state, xv
  MATRIX_DOUBLE *Av,                            // vehicle model array
  MATRIX_DOUBLE *B1v,                           // vehicle model array
  MATRIX_DOUBLE *B2v,                           // vehicle model array
  MATRIX_DOUBLE *C1v,                           // vehicle model array
  MATRIX_DOUBLE *D11v,                          // vehicle model array
  MATRIX_DOUBLE *D12v,                          // vehicle model array
  MATRIX_DOUBLE *C2v,                           // vehicle model array
  MATRIX_DOUBLE *D21v,                          // vehicle model array
  MATRIX_DOUBLE *D22v);                         // vehicle model array

// PropulsionSystem:  creates dynamic model of the propulsion system
void PropulsionSystem(
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  long double   T,                              // vehicle thrust
  long double   v,                              // input trim velocity
  long double   dt,                             // thrust command
  long double   *PGen,                          // power from generator
  long double   *mGen,                          // mass of generator
  MATRIX_DOUBLE *Ap,                            // propulsion model array
  MATRIX_DOUBLE *B1p,                           // propulsion model array
  MATRIX_DOUBLE *B2p,                           // propulsion model array
  MATRIX_DOUBLE *C1p,                           // propulsion model array
  MATRIX_DOUBLE *D11p,                          // propulsion model array
  MATRIX_DOUBLE *D12p,                          // propulsion model array
  MATRIX_DOUBLE *C2p,                           // propulsion model array
  MATRIX_DOUBLE *D21p,                          // propulsion model array
  MATRIX_DOUBLE *D22p,                          // propulsion model array
  MATRIX_DOUBLE *H1p,                           // propulsion model array
  MATRIX_DOUBLE *H2p);                          // propulsion model array

// atmosphere:  models effect of atmospheric variations at altitude
void atmosphere(
  long double   v,                              // velocity
  long double   h,                              // altitude
  long double   *rho,                           // atmospheric density
  long double   *qbar,                          // dynamic pressure
  long double   *mach);                         // mach number

//=======================================================================================
// end of header 'Aircraft'
//=======================================================================================