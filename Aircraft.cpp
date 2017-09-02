//=====================================================================================
// module 'aircraft'
//
// description:
//  This module contains the methods required to develop the integrated aircraft model.
//
// J R Dowdle
// 14-Aug-2017
//=====================================================================================

// include headers, macros, defined constants, procedure prototypes
#include "stdafx.h"
#include "Aircraft.h"
#include "DesignTrades.h"

// included namespaces
using namespace std;
using namespace MatrixLibrary;
using namespace ControlSystemLibrary;

// beginning of module 'aircraft'

//=====================================================================================
// method 'SystemModel'
//
// description:
//  This method sets up an integrated model of the turboelectric aircraft of 
//  the form
//
//  xtildadot =  A*xvtilda +  B1*w +  B2*utilda
//     ytilda = C2*xvtilda + D21*w + D22*utilda
//
//  where xtilda is the state, utilda is the control, w is the disturbance, 
//  ytilda is the output, and
//
//  x = x0 + xtilda;     u = u0 + utilda;     y = y0 + ytilda
//  x = [v;   aoa;   theta;   q;   omega], 
//      where v = speed, aoa = angle-of-attack, theta = pitch angle, 
//            q = pitch rate and omega = generator shaft angular rate
//  u = [dt;   de;   PGen],
//      where dt = throttle, de = elevator and PGen = generator power
//  y = [v;   aoa;   q;   omega],
//      where v = speed, aoa = angle-of-attack, q = pitch rate and
//            omega = generator speed
//  w = [w1;   w2;   w3;   w4;   v1;   v2;   v3;   v4]
//  (w1, w2, w3) = disturbances on states (v, aoa, q)
//  (v1, v2, v3, v4) = measurement noises on (v, aoa, q, omega)
//  Ts = maximum thrust
//  x0 = [v0;   aoa0;   theta0;   q0;   omega0]
//  u0 = [dt0;   de0;   PGen0];   yv0 = [v0;   aoa0;   q0;   PGen0]
//
// for purposes of control system design, an unweighted performance vector 
// is also defined as:
//
//  ztilda = [vtilda; aoatilda; dttilda; detilda; omegatilda; PGentilda] 
//
//
//  there are three trim conditions of interest:
//  Trim condition 1
//  Ts = 40000;                                 // thrust, lbf
//  v0 = 0.4*995;                               // fps
//  h0 = 30000;                                 // feet
//  gamma = 0;                                  // degrees
// 
//  Trim condition 2  
//  Ts = 40000;                                 // thrust, lbf
//  v0 = 0.5*995;                               // fps
//  h0 = 30000;                                 // feet
//  gamma = 0;                                  // degrees
//
//  Trim condition 3  
//  Ts = 40000;                                 // thrust, lbf
//  v0 = 0.6*995;                               // fps
//  h0 = 30000;                                 // feet
//  gamma = 0;                                  // degrees
//
// J R Dowdle
// 08-Aug-2017
//=====================================================================================

// beginning of method 'SystemModel'

void SystemModel(
  MATRIX_DOUBLE *aircraftDAT,                   // array of aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // array of propulsion data
  int trim,                                     // trim condition (1, 2, 3)
  SYSTEM_MODEL *sysGv,                          // system model of vehicle
  SYSTEM_MODEL *sysGp,                          // system model of propulsion system
  SYSTEM_MODEL *sysGs)                          // system model of integrated system
{

  // executable code

  // allocate space
  MATRIX_DOUBLE *xv0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(xv0, "xv0", 0, 4, 1);
  MATRIX_DOUBLE *yv0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(yv0, "yv0", 0, 3, 1);
  MATRIX_DOUBLE *uv0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(uv0, "uv0", 0, 2, 1);
  MATRIX_DOUBLE *xvdot = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(xvdot, "xvdot", 0, 4, 1);
  MATRIX_DOUBLE *Av = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Av, "Av", 0, 4, 4);
  MATRIX_DOUBLE *B1v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1v, "B1v", 0, 4, 6);
  MATRIX_DOUBLE *B2v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2v, "B2v", 0, 4, 2);
  MATRIX_DOUBLE *C1v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C1v, "C1v", 0, 4, 4);
  MATRIX_DOUBLE *D11v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D11v, "D11v", 0, 4, 6);
  MATRIX_DOUBLE *D12v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12v, "D12v", 0, 4, 2);
  MATRIX_DOUBLE *C2v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2v, "C2v", 0, 3, 4);
  MATRIX_DOUBLE *D21v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D21v, "D21v", 0, 3, 6);
  MATRIX_DOUBLE *D22v = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D22v, "D22v", 0, 3, 2);
  MATRIX_DOUBLE *xp0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(xp0, "xp0", 0, 1, 1);
  MATRIX_DOUBLE *yp0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(yp0, "yp0", 0, 1, 1);
  MATRIX_DOUBLE *up0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(up0, "up0", 0, 1, 1);
  MATRIX_DOUBLE *Ap = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Ap, "Ap", 0, 1, 1);
  MATRIX_DOUBLE *B1p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1p, "B1p", 0, 1, 1);
  MATRIX_DOUBLE *B2p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2p, "B2p", 0, 1, 1);
  MATRIX_DOUBLE *C1p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C1p, "C1p", 0, 2, 1);
  MATRIX_DOUBLE *D11p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D11p, "D11p", 0, 2, 1);
  MATRIX_DOUBLE *D12p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12p, "D12p", 0, 2, 1);
  MATRIX_DOUBLE *C2p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2p, "C2p", 0, 1, 1);
  MATRIX_DOUBLE *D21p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D21p, "D21p", 0, 1, 1);
  MATRIX_DOUBLE *D22p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D22p, "D22p", 0, 1, 1);
  MATRIX_DOUBLE *H1p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(H1p, "H1p", 0, 1, 4);
  MATRIX_DOUBLE *H2p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(H2p, "H2p", 0, 1, 2);
  MATRIX_DOUBLE *xs0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(xs0, "xs0", 0, 5, 1);
  MATRIX_DOUBLE *ys0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(ys0, "ys0", 0, 4, 1);
  MATRIX_DOUBLE *us0 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(us0, "us0", 0, 3, 1);
  MATRIX_DOUBLE *As = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(As, "As", 0, 5, 5);
  MATRIX_DOUBLE *B1s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1s, "B1s", 0, 5, 7);
  MATRIX_DOUBLE *B2s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2s, "B2s", 0, 5, 3);
  MATRIX_DOUBLE *C1s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C1s, "C1s", 0, 6, 5);
  MATRIX_DOUBLE *D11s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D11s, "D11s", 0, 6, 7);
  MATRIX_DOUBLE *D12s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12s, "D12s", 0, 6, 3);
  MATRIX_DOUBLE *C2s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2s, "C2s", 0, 4, 5);
  MATRIX_DOUBLE *D21s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D21s, "D21s", 0, 4, 7);
  MATRIX_DOUBLE *D22s = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D22s, "D22s", 0, 4, 3);
  MATRIX_DOUBLE *Z1 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z1, "Z1", 0, 4, 1);
  MATRIX_DOUBLE *Z2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z2, "Z2", 0, 1, 6);
  MATRIX_DOUBLE *Z3 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z3, "Z3", 0, 2, 4);
  MATRIX_DOUBLE *Z4 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z4, "Z4", 0, 3, 1);
  MATRIX_DOUBLE *Z5 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z5, "Z5", 0, 1, 4);
  MATRIX_DOUBLE *Z6 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z6, "Z6", 0, 1, 2);
  MATRIX_DOUBLE *Z7 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z7, "Z7", 0, 2, 6);
  MATRIX_DOUBLE *Z8 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z8, "Z8", 0, 2, 2);

  // develop the aircraft vehicle model valid near trim
  // the model has the form
  //
  //  xvtildadot =  Av*xvtilda +  B1v*wv +  B2v*uvtilda
  //     yvtilda = C2v*xvtilda + D21v*wv + D22v*uvtilda
  //
  // where xvtilda is the state, uvtilda is the control, wv is the disturbance, 
  // yvtilda is the output, and
  //
  //  xv = xv0 + xvtilda;     uv = uv0 + uvtilda;     yv = yv0 + yvtilda
  //  xv = [v;   aoa;   theta;   q]
  //  uv = [dt;   de]
  //  yv = [v;   aoa;   q] 
  //  wv = [w1;   w2;   w3;   v1;   v2;   v3]
  //  (w1, w2, w3) = disturbanced on states (v, aoa, q)
  //  (v1, v2, v3) = measurement noises on (v, aoa, q)
  //  Ts = maximum thrust
  //  xv0 = [v0;   aoa0;   theta0;   q0]
  //  uv0 = [dt0;   de0]
  //  yv0 = [v0;   aoa0;   q0]
  //
  // for purposes of control system design, an unweighted performance vector 
  // is also defined as:
  //
  //  zvtilda = [vtilda; aoatilda; dttilda; detilda] 
  //

  // get desired trim condition
  long double T0, v0, h0, gamma;
  if (trim == 1)
  {
    aircraftDAT->real[4][0] = 40000;            // thrust, lbf
    T0 = 40000;                                 // thrust, lbf
    v0 = 0.4*995;                               // fps
    h0 = 30000;                                 // feet
    gamma = 0;                                  // degrees
  }
  else if (trim == 2)
  {
    aircraftDAT->real[4][0] = 40000;            // thrust, lbf
    T0 = 40000;                                 // thrust, lbf
    v0 = 0.5*995;                               // fps
    h0 = 30000;                                 // feet
    gamma = 0;                                  // degrees
  }
  else if (trim == 3)
  {
    aircraftDAT->real[4][0] = 40000;            // thrust, lbf
    T0 = 40000;                                 // thrust, lbf
    v0 = 0.6*995;                               // fps
    h0 = 30000;                                 // feet
    gamma = 0;                                  // degrees
  }

  // compute the trim conditions   
  trimComputation(trim, v0, h0, gamma, xv0, uv0);

  // set up the vehicle state-space model
  aircraftDyn(aircraftDAT, propulsionDAT, 0.0, xv0, uv0, h0, xvdot, Av, B1v, B2v, C1v, D11v, D12v, C2v, D21v, D22v);

  // set the variables for trim condition
  long double Ts, theta0, aoa0, q0, dt0, de0;
  Ts = T0;                                      // thrust (N)
  v0 = xv0->real[0][0];                         // trim velocity (fps)
  aoa0 = xv0->real[1][0];                       // angle-of-attack (rad)
  theta0 = xv0->real[2][0];                     // pitch angle (rad)
  q0 = xv0->real[3][0];                         // pitch rate (rps)
  dt0 = uv0->real[0][0];                        // throttle (unitless)
  de0 = uv0->real[1][0];                        // elevator (deg)

  // set the nominal measurement
  yv0->real[0][0] = v0;                         // trim velocity (fps)
  yv0->real[1][0] = aoa0;                       // angle-of-attack (rad)
  yv0->real[2][0] = q0;                         // pitch rate (rps)

  // set the system model for the vehicle dynamics
  ControlToolbox::ss2sys(sysGv, Av, B1v, B2v, C1v, C2v, D11v, D12v, D21v, D22v);

  // set the reference state, output, control for the vehicle system model
  MatrixToolbox::equate(xv0, sysGv->xref);
  MatrixToolbox::equate(yv0, sysGv->yref);
  MatrixToolbox::equate(uv0, sysGv->uref);
  ControlToolbox::SystemAnalysis(sysGv);

  // develop the propulsion system model of the form
  //
  // xptildadot =  Ap*xptilda +  B1p*wp +  B2p*PGentilda + H1p*Xvtilda + H2p*Uvtilda
  //    yptilda = C2p*xptilda + D21p*wp + D22p*PGentilda
  //
  // where xptilda is the state, PGentilda is the control, wp is the disturbance, 
  // yptilda is the output, and
  //
  //  xp = omega0 + xptilda;     PGen = PGen0 + PGentilda;     yp = yp0 + yptilda
  //  xp = generator shaft speed (rps)
  //  yp = generator shaft speed (rps)
  //  wp = v4
  //  dp = shaft disturbance angular acceleration
  //  vp = shaft speed measurement noise
  //
  // for purposes of control system design, an unweighted performance vector 
  // is also defined as:
  //
  //  zptilda = [xptilda; PGentilda/PGen0] 
  //

  // set up the propulsion system state-space model
  long double PGen0, mGen0, omega0;
  PropulsionSystem(propulsionDAT, Ts, v0, dt0, &PGen0, &mGen0, Ap, B1p, B2p, C1p, D11p, D12p, C2p, D21p, D22p, H1p, H2p);

  // set the nominal outputs
  omega0 = propulsionDAT->real[4][0];
  xp0->real[0][0] = omega0;
  yp0->real[0][0] = omega0;
  up0->real[0][0] = PGen0*dt0;

  // set up the system model, Gp(s)
  ControlToolbox::ss2sys(sysGp, Ap, B1p, B2p, C1p, C2p, D11p, D12p, D21p, D22p);

  // set the reference state, output, control for the propulsion system model
  MatrixToolbox::equate(xp0, sysGp->xref);
  MatrixToolbox::equate(yp0, sysGp->yref);
  MatrixToolbox::equate(up0, sysGp->uref);
  ControlToolbox::SystemAnalysis(sysGp);

  // develop the integrated model of the turboelectric aircraft of the form
  //
  //  xtildadot =  A*xvtilda +  B1*w +  B2*utilda
  //     ytilda = C2*xvtilda + D21*w + D22*utilda
  //
  // where xtilda is the state, utilda is the control, w is the disturbance, 
  // ytilda is the output, and
  //
  //  x = x0 + xtilda;     u = u0 + utilda;     y = y0 + ytilda
  //  x = [v;   aoa;   theta;   q;   omega], 
  //      where v = speed, aoa = angle-of-attack, theta = pitch angle, 
  //            q = pitch rate and omega = generator shaft angular rate
  //  u = [dt;   de;   PGen],
  //      where dt = throttle, de = elevator and PGen = generator power
  //  y = [v;   aoa;   q;   omega],
  //      where v = speed, aoa = angle-of-attack, q = pitch rate and
  //            omega = generator speed
  //  w = [w1;   w2;   w3;   w4;   v1;   v2;   v3;   v4]
  //  (w1, w2, w3) = disturbances on states (v, aoa, q)
  //  (v1, v2, v3, v4) = measurement noises on (v, aoa, q, omega)
  //  Ts = maximum thrust
  //  x0 = [v0;   aoa0;   theta0;   q0;   omega0]
  //  u0 = [dt0;   de0;   PGen0];   yv0 = [v0;   aoa0;   q0;   PGen0]
  //
  // for purposes of control system design, an unweighted performance vector 
  // is also defined as:
  //
  //  ztilda = [vtilda; aoatilda; dttilda; detilda; omegatilda; PGentilda/PGen0] 
  //

  // develop the integrated model of the aircraft and propulsioin system
  MatrixToolbox::zeros(Z1);
  MatrixToolbox::zeros(Z2);
  MatrixToolbox::zeros(Z3);
  MatrixToolbox::zeros(Z4);
  MatrixToolbox::zeros(Z5);
  MatrixToolbox::zeros(Z6);
  MatrixToolbox::zeros(Z7);
  MatrixToolbox::zeros(Z8);
  MatrixToolbox::augment(Av, Z1, H1p, Ap, As);
  MatrixToolbox::augment(B1v, Z1, Z2, B1p, B1s);
  MatrixToolbox::augment(B2v, Z1, H2p, B2p, B2s);
  MatrixToolbox::augment(C1v, Z1, Z3, C1p, C1s);
  MatrixToolbox::augment(D11v, Z1, Z7, D11p, D11s);
  MatrixToolbox::augment(D12v, Z1, Z8, D12p, D12s);
  MatrixToolbox::augment(C2v, Z4, Z5, C2p, C2s);
  MatrixToolbox::augment(D21v, Z4, Z2, D21p, D21s);
  MatrixToolbox::augment(D22v, Z4, Z6, D22p, D22s);

  // set the nominal state, output and control vectors
  for (int i = 0; i < xv0->rows; i++)
    xs0->real[i][0] = xv0->real[i][0];
  for (int i = xv0->rows; i < (xv0->rows + 1); i++)
    xs0->real[i][0] = xp0->real[0][0];

  for (int i = 0; i < yv0->rows; i++)
    ys0->real[i][0] = yv0->real[i][0];
  for (int i = yv0->rows; i < (yv0->rows + 1); i++)
    ys0->real[i][0] = yp0->real[0][0];

  for (int i = 0; i < uv0->rows; i++)
    us0->real[i][0] = uv0->real[i][0];
  for (int i = uv0->rows; i < (uv0->rows + 1); i++)
    us0->real[i][0] = up0->real[0][0];

  // set up the system model, Gs(s)
  ControlToolbox::ss2sys(sysGs, As, B1s, B2s, C1s, C2s, D11s, D12s, D21s, D22s);

  // set the reference state, output, control for the integrated system model
  MatrixToolbox::equate(xs0, sysGs->xref);
  MatrixToolbox::equate(ys0, sysGs->yref);
  MatrixToolbox::equate(us0, sysGs->uref);
  ControlToolbox::SystemAnalysis(sysGs);

  // free allocated space
  MatrixToolbox::destroy(xv0);
  MatrixToolbox::destroy(yv0);
  MatrixToolbox::destroy(uv0);
  MatrixToolbox::destroy(xvdot);
  MatrixToolbox::destroy(Av);
  MatrixToolbox::destroy(B1v);
  MatrixToolbox::destroy(B2v);
  MatrixToolbox::destroy(C1v);
  MatrixToolbox::destroy(D11v);
  MatrixToolbox::destroy(D12v);
  MatrixToolbox::destroy(C2v);
  MatrixToolbox::destroy(D21v);
  MatrixToolbox::destroy(D22v);
  MatrixToolbox::destroy(xp0);
  MatrixToolbox::destroy(yp0);
  MatrixToolbox::destroy(up0);
  MatrixToolbox::destroy(Ap);
  MatrixToolbox::destroy(B1p);
  MatrixToolbox::destroy(B2p);
  MatrixToolbox::destroy(C1p);
  MatrixToolbox::destroy(D11p);
  MatrixToolbox::destroy(D12p);
  MatrixToolbox::destroy(C2p);
  MatrixToolbox::destroy(D21p);
  MatrixToolbox::destroy(D22p);
  MatrixToolbox::destroy(H1p);
  MatrixToolbox::destroy(H2p);
  MatrixToolbox::destroy(xs0);
  MatrixToolbox::destroy(ys0);
  MatrixToolbox::destroy(us0);
  MatrixToolbox::destroy(As);
  MatrixToolbox::destroy(B1s);
  MatrixToolbox::destroy(B2s);
  MatrixToolbox::destroy(C1s);
  MatrixToolbox::destroy(D11s);
  MatrixToolbox::destroy(D12s);
  MatrixToolbox::destroy(C2s);
  MatrixToolbox::destroy(D21s);
  MatrixToolbox::destroy(D22s);
  MatrixToolbox::destroy(Z1);
  MatrixToolbox::destroy(Z2);
  MatrixToolbox::destroy(Z3);
  MatrixToolbox::destroy(Z4);
  MatrixToolbox::destroy(Z5);
  MatrixToolbox::destroy(Z6);
  MatrixToolbox::destroy(Z7);
  MatrixToolbox::destroy(Z8);
}
//=====================================================================================
// end of method 'SystemModel'
//=====================================================================================

//=====================================================================================
// method 'trimComputuation'
//
// description:
//  This method ...
//
// J R Dowdle
// 08-Aug-2017
//=====================================================================================

// beginning of method 'trimComputuation'

void trimComputation(
  int trim,                                     // trim condition (1, 2, or 3) 
  long double v,                                // input trim velocity
  long double h,                                // input trim altitude
  long double gamma,                            // input trim flight path angle
  MATRIX_DOUBLE *xv,                            // output trim state of vehicle
  MATRIX_DOUBLE *uv)                            // output trim control of vehicle
{
  // executable code

  // check inputs

  // allocate space
  if (trim == 1)
  {
    xv->real[0][0] = 3.98000000e+02;
    xv->real[1][0] = 1.80450199e-01;
    xv->real[2][0] = 1.80450199e-01;
    xv->real[3][0] = 0;
    uv->real[0][0] = 4.06258498e-01;
    uv->real[1][0] = -1.06200961e+01;
  }

  else if (trim == 2)
  {
    xv->real[0][0] = 4.97500000e+02;
    xv->real[1][0] = 1.02828344e-01;
    xv->real[2][0] = 1.02828344e-01;
    xv->real[3][0] = 0;
    uv->real[0][0] = 4.17913704e-01;
    uv->real[1][0] = -4.71301552e+00;
  }

  else if (trim == 3)
  {
    xv->real[0][0] = 5.97000000e+02;
    xv->real[1][0] = 6.00112419e-02;
    xv->real[2][0] = 6.00112419e-02;
    xv->real[3][0] = 0;
    uv->real[0][0] = 5.22214142e-01;
    uv->real[1][0] = -1.41547882e+00;
  }

  // free allocated space
}

//=====================================================================================
// end of method 'trimComputuation'
//=====================================================================================

//=====================================================================================
// method 'cost'
//
// description:
//  This method computes the cost functional for trim condition optimization.
//
// J R Dowdle
// 08-Aug-2017
//=====================================================================================

// beginning of method 'cost'

long double cost(
  MATRIX_DOUBLE *s,                             // optimization variable
  MATRIX_DOUBLE *x,                             // vehicle state
  long double gamma,                            // vehicle flight path angle
  long double h,                                // vehicle alitude
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  MATRIX_DOUBLE *Av,                            // vehicle model array
  MATRIX_DOUBLE *B1v,                           // vehicle model array
  MATRIX_DOUBLE *B2v,                           // vehicle model array
  MATRIX_DOUBLE *C1v,                           // vehicle model array
  MATRIX_DOUBLE *D11v,                          // vehicle model array
  MATRIX_DOUBLE *D12v,                          // vehicle model array
  MATRIX_DOUBLE *C2v,                           // vehicle model array
  MATRIX_DOUBLE *D21v,                          // vehicle model array
  MATRIX_DOUBLE *D22v)                          // vehicle model array
{
  // executable code

  // allocate space
  MATRIX_DOUBLE *u = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(u, "u", 0, 3, 1);
  MATRIX_DOUBLE *y = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(y, "y", 0, 4, 1);
  MATRIX_DOUBLE *yd = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(yd, "yd", 0, 4, 1);

  // s represents optimization variables
  u->real[0][0] = s->real[0][0];                // throttle
  u->real[1][0] = s->real[1][0];                // elevator
  y->real[0][0] = x->real[0][0];                // speed
  y->real[1][0] = s->real[2][0];                // angle-of-attack
  y->real[2][0] = y->real[1][0] + gamma;        // pitch angle
  y->real[3][0] = x->real[3][0];                // pitch rate

// evaluate nonlinear simulation
  aircraftDyn(aircraftDAT, propulsionDAT, 0.0, y, u, h, yd, Av, B1v, B2v, C1v, D11v, D12v, C2v, D21v, D22v);

  // cost function
  long double f = yd->real[0][0]*yd->real[0][0] + yd->real[1][0]*yd->real[1][0] + yd->real[3][0]*yd->real[3][0];

  // free allocated space
  MatrixToolbox::destroy(u);
  MatrixToolbox::destroy(y);
  MatrixToolbox::destroy(yd);

  return(f);
}

//=====================================================================================
// end of method 'cost'
//=====================================================================================

//=====================================================================================
// method 'aircraftDyn'
//
// description:
//  This method computes the state derivative about an operating point for a 
//  transport aircraft's longitudinal dynamics, as well as the state-space
//  model matrices about that same condition:
//
//  xvtildadot =  Av*xvtilda +  B2v*uvtilda
//
// J R Dowdle
// 08-Aug-2017
//=====================================================================================

// beginning of method 'aircraftDyn'

void aircraftDyn(
  MATRIX_DOUBLE *aircraftDAT,                   // aircraft data
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  long double time,                             // simulation time
  MATRIX_DOUBLE *x,                             // output trim state of vehicle
  MATRIX_DOUBLE *u,                             // output trim control of vehicle
  long double h,                                // vehicle alitude
  MATRIX_DOUBLE *xd,                            // derivative of state, xv
  MATRIX_DOUBLE *Av,                            // vehicle model array
  MATRIX_DOUBLE *B1v,                           // vehicle model array
  MATRIX_DOUBLE *B2v,                           // vehicle model array
  MATRIX_DOUBLE *C1v,                           // vehicle model array
  MATRIX_DOUBLE *D11v,                          // vehicle model array
  MATRIX_DOUBLE *D12v,                          // vehicle model array
  MATRIX_DOUBLE *C2v,                           // vehicle model array
  MATRIX_DOUBLE *D21v,                          // vehicle model array
  MATRIX_DOUBLE *D22v)                          // vehicle model array
{
  // executable code

  // declare variables
  long double S, CBAR, MASS, IYY, TSTAT;
  long double DTDV, ZE, CDCLS, CLA, CMA;
  long double CMDE, CMQ, CMADOT, CLADOT, RTOD;
  long double GD, CLO_CLEAN, CLO_LAND, CDO_CLEAN, CDO_LAND;
  long double CMO_CLEAN, CMO_LAND, DCDG_CLEAN, DCDG_LAND, DCMG_CLEAN;
  long double DCMG_LAND, eta, powerDensity, fPropeller, JPropeller, omega0;

  // set aircraft parameters
  S = aircraftDAT->real[0][0];
  CBAR = aircraftDAT->real[1][0];
  MASS = aircraftDAT->real[2][0];
  IYY = aircraftDAT->real[3][0];
  TSTAT = aircraftDAT->real[4][0];
  DTDV = aircraftDAT->real[5][0];
  ZE = aircraftDAT->real[6][0];
  CDCLS = aircraftDAT->real[7][0];
  CLA = aircraftDAT->real[8][0];
  CMA = aircraftDAT->real[9][0];
  CMDE = aircraftDAT->real[10][0];
  CMQ = aircraftDAT->real[11][0];
  CMADOT = aircraftDAT->real[12][0];
  CLADOT = aircraftDAT->real[13][0];
  RTOD = aircraftDAT->real[14][0];
  GD = aircraftDAT->real[15][0];
  CLO_CLEAN = aircraftDAT->real[16][0];
  CLO_LAND = aircraftDAT->real[17][0];
  CDO_CLEAN = aircraftDAT->real[18][0];
  CDO_LAND = aircraftDAT->real[19][0];
  CMO_CLEAN = aircraftDAT->real[20][0];
  CMO_LAND = aircraftDAT->real[21][0];
  DCDG_CLEAN = aircraftDAT->real[22][0];
  DCDG_LAND = aircraftDAT->real[23][0];
  DCMG_CLEAN = aircraftDAT->real[24][0];
  DCMG_LAND = aircraftDAT->real[25][0];

  long double XCG, THTL, ELEV, v, ALPHA, THETA, Q, H;
  XCG = 0.25;                                 // center of gravity

// state and control variables
  THTL = u->real[0][0];                         // throttle
  ELEV = u->real[1][0];                         // elevator
  v = x->real[0][0];                            // wind speed in fps 
  ALPHA = 90/asin(1.0)*x->real[1][0];           // angle-of-attack (deg)
  THETA = x->real[2][0];                        // pitch angle 
  Q = x->real[3][0];                            // pitch rate
  H = h;                                        // altitude 

// set up the propulsion system model
  long double dt0, PGen0, mGen;
  MATRIX_DOUBLE *Ap = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Ap, "Ap", 0, 1, 1);
  MATRIX_DOUBLE *B1p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1p, "B1p", 0, 1, 1);
  MATRIX_DOUBLE *B2p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2p, "B2p", 0, 1, 1);
  MATRIX_DOUBLE *C1p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C1p, "C1p", 0, 2, 1);
  MATRIX_DOUBLE *D11p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D11p, "D11p", 0, 2, 1);
  MATRIX_DOUBLE *D12p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12p, "D12p", 0, 2, 1);
  MATRIX_DOUBLE *C2p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2p, "C2p", 0, 1, 1);
  MATRIX_DOUBLE *D21p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D21p, "D21p", 0, 1, 1);
  MATRIX_DOUBLE *D22p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D22p, "D22p", 0, 1, 1);
  MATRIX_DOUBLE *H1p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(H1p, "H1p", 0, 1, 4);
  MATRIX_DOUBLE *H2p = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(H2p, "H2p", 0, 1, 2);

  dt0 = THTL;
  PropulsionSystem(propulsionDAT, TSTAT, v, dt0, &PGen0, &mGen, Ap, B1p, B2p, C1p, D11p, D12p, C2p, D21p, D22p, H1p, H2p);

  // add propulsion system mass to vehicle mass
  MASS = MASS + mGen;

  // compute Mach and aerodynamic pressure
  long double MACH, QBAR, RHO, QS, SALP, CALP, GAM, SGAM, CGAM;
  atmosphere(v, h, &RHO, &QBAR, &MACH);
  QS = QBAR*S;             // aerodynamic force
  SALP = sin(x->real[1][0]);
  CALP = cos(x->real[1][0]);
  GAM = THETA - x->real[1][0];
  SGAM = sin(GAM);
  CGAM = cos(GAM);

  // complete setting aero stability derivatives based upon flight mode
  int LAND = 0;               //  landing flag
  long double CLO, CDO, CMO, DCDG, DCMG;
  if (LAND == 0)            //  clean 
  {
    CLO = CLO_CLEAN;
    CDO = CDO_CLEAN;
    CMO = CMO_CLEAN;
    DCDG = DCDG_CLEAN;
    DCMG = DCMG_CLEAN;
  }
  else if (LAND == 1)        //  landing 
  {
    CLO = CLO_LAND;
    CDO = CDO_LAND;
    CMO = CMO_LAND;
    DCDG = DCDG_LAND;
    DCMG = DCMG_LAND;
  }

  // set thrust
  long double THR = (TSTAT + v*DTDV)*max(THTL, 0);

  // set force and moment coefficients
  long double CL = CLO + CLA*ALPHA;
  long double CD = DCDG + CDO + CDCLS*CL*CL;
  long double CM = DCMG + CMO + CMA*ALPHA + CMDE*ELEV + CL*(XCG-.25);

  // compute time derivative of state vector 
  long double Momentum = MASS*v + QS*CLADOT;
  xd->real[0][0] = (THR*CALP - QS*CD)/MASS - GD*SGAM;                     // vdot
  xd->real[1][0] =(-THR*SALP - QS*CL + MASS*(v*Q + GD*CGAM))/Momentum;    //  alphadot
  xd->real[2][0] = Q;                                                     // thetadot
  long double D = 0.5*CBAR*(CMQ*Q + CMADOT*xd->real[1][0])/v;             // pitch damping 
  xd->real[3][0] = (QS*CBAR*(CM + D) + THR*ZE)/IYY;                       // qdot 

  // set up state model
  //
  //  xvtildadot =  Av*xvtilda +  B2v*uvtilda
  //
  // where xvtilda is the state, uvtilda is the control
  //
  //  xv = xv0 + xvtilda;     uv = uv0 + uvtilda;     yv = yv0 + yvtilda
  //  xv = [v;   aoa;   theta;   q]
  //  uv = [dt;   de] = [throttle; elevator]
  //  yv = [v;   aoa;   q] = [wind speed; angle-of-attack; pitch rate]
  //  wv = [w1;   w2;   w3;   v1;   v2;   v3]
  //  (w1, w2, w3) = disturbanced on states (v, aoa, q)
  //  (v1, v2, v3) = measurement noises on (v, aoa, q)
  //  T0 = nominal thrust
  //  xv0 = [v0;   aoa0;   theta0;   q0]
  //  uv0 = [dt0;   de0]
  //  yv0 = [v0;   aoa0;   q0]
  //
  // for purposes of control system design, an unweighted performance vector 
  // is also defined as:
  //
  //  zvtilda = [vtilda; aoatilda; dttilda; detilda] 
  //

  // compute Av
  long double RHOSV = RHO*S*v;
  Av->real[0][0] = (DTDV*THTL*CALP - CD*RHOSV)/MASS;
  Av->real[0][1] = -(THR*SALP + QS*CDCLS*2*CL*CLA*90/asin(1.0))/MASS + GD*CGAM;
  Av->real[0][2] = -GD*CGAM;
  Av->real[0][3] = 0;

  Av->real[1][0] = (-DTDV*THTL*SALP - RHOSV*CL + MASS*Q)/Momentum -
    (-THR*SALP - QS*CL + MASS*(v*Q + GD*CGAM))*(MASS + CLADOT*RHOSV)/pow(Momentum, 2);
  Av->real[1][1] = (-THR*CALP - QS*CLA*90/asin(1.0) + MASS*GD*SGAM)/Momentum;
  Av->real[1][2] = -MASS*GD*SGAM/Momentum;
  Av->real[1][3] = MASS*v/Momentum;

  Av->real[2][0] = 0;
  Av->real[2][1] = 0;
  Av->real[2][2] = 0;
  Av->real[2][3] = 1;

  Av->real[3][0] = CBAR*RHOSV*(CM + D)/IYY - CBAR*CBAR*QS*Q*CMQ/(2*IYY*v*v) +
    CBAR*(-CMADOT*xd->real[1][0]/(2*v*v) + CBAR*QS*CMADOT*Av->real[1][0])/(2*v*IYY) +
    ZE*DTDV*THTL/IYY;
  Av->real[3][1] = (CBAR*QS/IYY)*((CMA + (XCG - 0.25)*CLA)*90/asin(1.0) +
    CMADOT*CBAR/(2*v)*Av->real[1][1]);
  Av->real[3][2] = QS*CBAR*CBAR/(2*IYY*v)*CMADOT*Av->real[1][2];
  Av->real[3][3] = CBAR/IYY*QS*(CBAR/(2*v))*(CMQ + CMADOT*Av->real[1][3]);
  double x1 = CBAR/IYY*QS*(CBAR/(2*v))*(CMQ + CMADOT*Av->real[1][3]);

  // compute B2v
  B2v->real[0][0] = (TSTAT + v*DTDV)*CALP/MASS;
  B2v->real[0][1] = 0;
  B2v->real[1][0] = -(TSTAT + v*DTDV)*SALP/Momentum;
  B2v->real[1][1] = 0;
  B2v->real[2][0] = 0;
  B2v->real[2][1] = 0;
  B2v->real[3][0] = QS*CBAR*CBAR/(2*v*IYY)*CMADOT*B2v->real[1][0] + ZE/IYY*(TSTAT + v*DTDV);
  B2v->real[3][1] = QS*CBAR/IYY*CMDE;

  // assume disturbances for v, aoa and q; call these [w1; w2; w3] 

  // add measurement noise for each component of ytilda; call these [v1; v2; v3]
  MatrixToolbox::zeros(B1v);
  B1v->real[0][0] = 1;
  B1v->real[1][1] = 1;
  B1v->real[3][2] = 1;

  // set observation matrix for ztilda
  MatrixToolbox::zeros(C1v);
  C1v->real[0][0] = 1;
  C1v->real[1][1] = 1;

  // set coupling matrix from disturbanace to ztilda
  MatrixToolbox::zeros(D11v);

  // set coupling matrix from control to ztilda
  MatrixToolbox::zeros(D12v);
  D12v->real[2][0] = 1;
  D12v->real[3][1] = 1;

  // set observation matrix for ytilda
  MatrixToolbox::zeros(C2v);
  C2v->real[0][0] = 1;
  C2v->real[1][1] = 1;
  C2v->real[2][3] = 1;

  // set up remaining matrices for measurement process
  MatrixToolbox::zeros(D21v);
  D21v->real[0][3] = 1;
  D21v->real[1][4] = 1;
  D21v->real[2][5] = 1;
  MatrixToolbox::zeros(D22v);

  // free allocated space
  MatrixToolbox::destroy(Ap);
  MatrixToolbox::destroy(B1p);
  MatrixToolbox::destroy(B2p);
  MatrixToolbox::destroy(C1p);
  MatrixToolbox::destroy(D11p);
  MatrixToolbox::destroy(D12p);
  MatrixToolbox::destroy(C2p);
  MatrixToolbox::destroy(D21p);
  MatrixToolbox::destroy(D22p);
  MatrixToolbox::destroy(H1p);
  MatrixToolbox::destroy(H2p);
}

//=====================================================================================
// end of method 'aircraftDyn'
//=====================================================================================

//=====================================================================================
// method 'PropulsionSystem'
//
// description:
//  This method computes the nominal power and mass of the propulsion 
//  system at a nominal operating point.  In addition, a model valid near the
//  nominal operating point is developed of the form:
//
//  xptildadot =  Ap*xptilda +  B1p*wp +  B2p*PGentilda + H1p*Xvtilda + H2p*Uvtilda
//     yptilda = C2p*xptilda + D21p*wp + D22p*PGentilda
//
//  where xptilda is the state, PGentilda is the control, wp is the disturbance, 
//  yptilda is the output, and
//
//  xp = omega0 + xptilda;     PGen = PGen0 + PGentilda;     yp = y0 + yptilda
//  xp = generator shaft speed (rps)
//  yp = generator shaft speed (rps)
//  wp = v4
//  dp = shaft disturbance angular acceleration
//  vp = shaft speed measurement noise
//
// J R Dowdle
// 14-Aug-2017
//=====================================================================================

// beginning of method 'PropulsionSystem'

void PropulsionSystem(
  MATRIX_DOUBLE *propulsionDAT,                 // propulsion data
  long double T0,                               // vehicle thrust
  long double v0,                               // input trim velocity
  long double dt0,                              // thrust command
  long double *PGen0,                           // power from generator
  long double *mGen0,                           // mass of generator
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
  MATRIX_DOUBLE *H2p)                           // propulsion model array
{
  // executable code

  // set constants
  long double eta = propulsionDAT->real[0][0];    // subsystem efficiency
  long double PDen = propulsionDAT->real[1][0];   // power density
  long double fprop = propulsionDAT->real[2][0];  // propulsion damping factor
  long double Jprop = propulsionDAT->real[3][0];  // propulsion inertia
  long double omega0 = propulsionDAT->real[4][0]; // nominal shaft rotational rate

  // compute generator nominal power at trim
  *PGen0 = T0*v0/eta;

  // compute generator nominal weight at trim
  long double mGenkg = (*PGen0)/PDen;           // generator mass (kg)
  *mGen0 = mGenkg*2.205/32.2;                   // generator mass (slugs)

  // compute propulsion system dynamic model
  Ap->real[0][0] = -fprop/Jprop;
  B2p->real[0][0] = -1/(Jprop*omega0);
  MatrixToolbox::zeros(H1p);
  H1p->real[0][0] = T0*dt0/(Jprop*omega0*eta);
  MatrixToolbox::zeros(H2p);
  H2p->real[0][0] = T0*v0/(Jprop*omega0*eta);

  // set up performance variable model for propulsion system
  MatrixToolbox::zeros(C1p);
  C1p->real[0][0] = 1;
  MatrixToolbox::zeros(D11p);
  MatrixToolbox::zeros(D12p);
  D12p->real[1][0] = 1;

  // set up the observation model for propulsion system, assuming the shaft 
  // angular velocity is measured
  C2p->real[0][0] = 1;

  // add measurement noise for the shaft angular velocity measurement; call it v4
  B1p->real[0][0] = 0;
  D21p->real[0][0] = 1;
  D22p->real[0][0] = 0;
}

//=====================================================================================
// end of method 'PropulsionSystem'
//=====================================================================================

//=====================================================================================
// method 'atmosphere'
//
// description:
//  This method computes Mach number and aerodynamic pressure for input speed and 
//  altitude.
//
// J R Dowdle
// 14-Aug-2017
//=====================================================================================

// beginning of method 'atmosphere'

void atmosphere(
  long double v,                                // velocity
  long double h,                                // altitude
  long double *rho,                             // atmospheric density
  long double *qbar,                            // dynamic pressure
  long double *mach)                            // mach number
{
  // executable code

  //  set atmospheric density, mach number, aerodynamic pressure
  long double R0 = 2.377e-03;
  long double TFAC = 1.0 - 0.703e-05*h;
  long double T = 519*TFAC;
  if (h >= 35000)
    T = 390;
  *rho = R0*pow(TFAC, 4.14);
  *qbar = 0.5*(*rho)*v*v;
  *mach = v/sqrt(1.4*1716.3*T);
}

//=====================================================================================
// end of method 'atmosphere'
//=====================================================================================

//=====================================================================================
// end of module 'aircraft'
//=====================================================================================