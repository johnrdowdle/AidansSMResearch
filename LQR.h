//=======================================================================================
// header 'LQR'
//
// description:
//  This header contains headers, macros, defined constants and prototypes for the 
//  LQR analysis module.
//
// J R Dowdle
// 25-Aug-2017
//=======================================================================================

// include headers, macros, defined constants, procedure prototypes

#pragma once

#define DELV              0.01*v0               // desired maximum excursion of v0
#define DELAOA            0.01*aoa0             // desired maximum excursion of aoa0
#define DTMAX             0.10                  // desired maximum excursion of dttilda
#define DEMAX             30.0                  // desired maximum excursion of detilda
#define OMEGAMAX          0.25*omega0           // desired maximum excursion of omegatilda
#define DELPG             0.10                  // desired maximum excursion of (PGen/PGen0)

// define LQR method prototypes

// LQRWeights:  computes LQR weights
void LQRWeights(
  SYSTEM_MODEL  *sysP,                          // input system
  MATRIX_DOUBLE *W,                             // input disturbance intensity
  MATRIX_DOUBLE *Q,                             // output lqr weights
  long double ThrustMaxBW,                      // limit on thrust loop bandwidth
  long double ElevatorMaxBW,                    // limit on elevator loop bandwidth
  long double PropulsionMaxBW);                 // limit on propulsion loop bandwidth

// LQRDesign:  performs lqr feedback design
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
  long double   *performancePGen);              // power generator control performance

// LQRCost:  computes cost associated with the lqr problem
long double LQRCost(
  MATRIX_DOUBLE *LAMBDA,                        // lqr cost performance weighting matrix
  MATRIX_DOUBLE *Z,                             // lqr closed-loop performance covariance matrix
  MATRIX_DOUBLE *lqrBW,                         // lqr loop bandwidths
  long double ThrustMaxBW,                      // limit on thrust loop bandwidth
  long double ElevatorMaxBW,                    // limit on elevator loop bandwidth
  long double PropulsionMaxBW);                 // limit on propulsion loop bandwidth

// DynamicLQR:  simulates the LQR control system
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
  MATRIX_DOUBLE *SIGomegalq);                   // lqr propulsion shaft angular velocity standard deviation response to disturbance

//=======================================================================================
// end of header 'LQR'
//=======================================================================================