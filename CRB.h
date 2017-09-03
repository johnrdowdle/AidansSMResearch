//=======================================================================================
// header 'CRB'
//
// description:
//  This header contains headers, macros, defined constants and prototypes for the 
//  Cramer-Rao Bounding analysis module.
//
// J R Dowdle
// 25-Aug-2017
//=======================================================================================

// include headers, macros, defined constants, procedure prototypes

#pragma once

#include <string>
#include "../../MatrixSolutions/ControlSystemLibrary/ControlSystemLibrary.h"
#include "../../MatrixSolutions/MatrixLibrary/MatrixLibrary.h"

#define DISTCONVERGED    1.0e-08                // convergence criteria for disturbance analysis
#define MAXDISTITERS     1000                   // maximum number of disturbance analysis iterations per axis
#define TOLDISTURBANCE   1.0e-01                // error degradation for disturbance analysis
#define TOLNOISE         1.0e-02                // error degradation for measurement noiose analysis

// define CRB module prototypes

void StaticCRB(
  SYSTEM_MODEL  *sysP,                          // integrated system model
  long double   v0,                             // velocity
  long double   aoa0,                           // angle-of-attack
  long double   omega0,                         // propulsion system shaft angular rate
  MATRIX_DOUBLE *W,                             // nominal disturbance intensity matrix
  MATRIX_DOUBLE *W00,                           // sensitivity matrix for velocity disturbance variations around W
  MATRIX_DOUBLE *W11,                           // sensitivity matrix for angle-of-attack disturbance variations around W
  MATRIX_DOUBLE *W22,                           // sensitivity matrix for pitch rate disturbance variations around W
  MATRIX_DOUBLE *W33,                           // sensitivity matrix for velocity sensor noise variations around W
  MATRIX_DOUBLE *W44,                           // sensitivity matrix for angle-of-attack sensor noise variations around W
  MATRIX_DOUBLE *W55,                           // sensitivity matrix for pitch rate sensor noise variations around W
  MATRIX_DOUBLE *W66,                           // sensitivity matrix for propulsion shaft rate sensor noise variations around W
  MATRIX_DOUBLE *K,                             // kalman filter gain
  MATRIX_DOUBLE *bw,                            // kalman filter loop bandwidths associated with intensity W
  long double   *performancev,                  // velocity performance
  long double   *performanceaoa,                // angle-of-attack performance
  long double   *performanceomega);             // propulsion shaft angular rate performance

void DynamicCRB(
  SYSTEM_MODEL  *sysP,                          // integrated system model
  MATRIX_DOUBLE *K,                             // kalman filter gain
  MATRIX_DOUBLE *W,                             // nominal disturbance intensity matrix
  MATRIX_DOUBLE *disturbance,                   // deterministic disturbance vector
  long double   pulseStart,                     // beginning of disturbance pulse
  long double   pulseLength,                    // duration of disturbance pulse
  MATRIX_DOUBLE *stime,                         // time vector for solutions
  MATRIX_DOUBLE *vol,                           // open-loop velocity response to disturbance
  MATRIX_DOUBLE *aoaol,                         // open-loop angle-of-attack response to disturbance
  MATRIX_DOUBLE *qol,                           // open-loop pitch rate response to disturbance
  MATRIX_DOUBLE *omegaol,                       // open-loop propulsion shaft angular rate response to disturbance
  MATRIX_DOUBLE *SIGvol,                        // open-loop velocity standard deviation response to disturbance
  MATRIX_DOUBLE *SIGaoaol,                      // open-loop angle-of-attack standard deviation response to disturbance
  MATRIX_DOUBLE *SIGqol,                        // open-loop pitch rate standard deviation response to disturbance
  MATRIX_DOUBLE *SIGomegaol,                    // open-loop propulsion shaft angular rate standard deviation response to disturbance
  MATRIX_DOUBLE *vkf,                           // kalman filter mean velocity estimate
  MATRIX_DOUBLE *aoakf,                         // kalman filter mean angle-of-attack estimate
  MATRIX_DOUBLE *qkf,                           // kalman filter mean pitch rate estimate
  MATRIX_DOUBLE *omegakf,                       // kalman filter mean propulsion shaft angular rate estimate
  MATRIX_DOUBLE *SIGvkf,                        // kalman filter standard deviation of velocity estimation error
  MATRIX_DOUBLE *SIGaoakf,                      // kalman filter standard deviation of angle-of-attack estimation error
  MATRIX_DOUBLE *SIGqkf,                        // kalman filter standard deviation of pitch rate estimation error
  MATRIX_DOUBLE *SIGomegakf);                   // kalman filter standard deviation of propulsion shaft angular rate estimation error

//=======================================================================================
// end of header 'CRB'
//=======================================================================================