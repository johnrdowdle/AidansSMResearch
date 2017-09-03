//=====================================================================================
// module 'CRB'
//
// description:
//  This module performs the Cramer-Rao Bounding analysis for the 
//  turboelectric aircraft.
//
// J R Dowdle
// 0.0.0.0
// 14-Aug-2017
//=====================================================================================

// include headers, macros, defined constants, procedure prototypes
#include "stdafx.h"
#include "CRB.h"
#include "DesignTrades.h"

// included namespaces
using namespace std;
using namespace MatrixLibrary;
using namespace ControlSystemLibrary;

// beginning of module 'CRB'

//=====================================================================================
// method 'StaticCRB'
//
// description:
//  This method computes the process and measurement noises that result in 10% open-
//  variations in the key states, and 1% estimation error when a Kalman filter is 
//  implemented.
//
// J R Dowdle
// 24-Aug-2017
//=====================================================================================

// beginning of method 'StaticCRB'

void StaticCRB(
  SYSTEM_MODEL *sysP,                           // integrated system model
  long double v0,                               // velocity
  long double aoa0,                             // angle-of-attack
  long double omega0,                           // propulsion system shaft angular rate
  MATRIX_DOUBLE *W,                             // nominal disturbance intensity matrix
  MATRIX_DOUBLE *Wv00,                          // sensitivity matrix for velocity disturbance variations around W
  MATRIX_DOUBLE *Wv11,                          // sensitivity matrix for angle-of-attack disturbance variations around W
  MATRIX_DOUBLE *Wv22,                          // sensitivity matrix for pitch rate disturbance variations around W
  MATRIX_DOUBLE *Wv33,                          // sensitivity matrix for velocity sensor noise variations around W
  MATRIX_DOUBLE *Wv44,                          // sensitivity matrix for angle-of-attack sensor noise variations around W
  MATRIX_DOUBLE *Wv55,                          // sensitivity matrix for pitch rate sensor noise variations around W
  MATRIX_DOUBLE *Wv66,                          // sensitivity matrix for propulsion shaft rate sensor noise variations around W
  MATRIX_DOUBLE *K,                             // kalman filter gain
  MATRIX_DOUBLE *kfBW,                          // kalman filter loop bandwidths associated with intensity W
  long double   *performancev,                  // velocity performance
  long double   *performanceaoa,                // angle-of-attack performance
  long double   *performanceomega)              // propulsion shaft angular rate performance
{
  // executable code

  // allocated space
  int n = sysP->xdim;
  int m1 =  sysP->wdim;
  int m2 =  sysP->udim;
  int r1 = sysP->zdim;
  int r2 = sysP->ydim;
  MATRIX_DOUBLE *B1W = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1W, "B1W", 0, n, m1);
  MATRIX_DOUBLE *B1t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1t, "B1t", 0, m1, n);
  MATRIX_DOUBLE *B1WB1t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1WB1t, "B1WB1t", 0, n, n);
  MATRIX_DOUBLE *sqrtW = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(sqrtW, "sqrtW", 0, m1, m1);
  MATRIX_DOUBLE *B1 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1, "B1", 0, n, m1);
  MATRIX_DOUBLE *D21 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D21, "D21", 0, r2, m1);
  MATRIX_DOUBLE *Y = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Y, "Y", 0, n, n);
  MATRIX_DOUBLE *C2t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2t, "C2t", 0, n, r2);
  MATRIX_DOUBLE *C2Y = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2Y, "C2Y", 0, r2, n);
  MATRIX_DOUBLE *X = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(X, "X", 0, r2, r2);
  SYSTEM_MODEL *sysG = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysG, "sysG", n, m1, m2, r1, r2);

  // find distrubances that result in TOLDISTURBANCE variation of the states from their means
  long double W00 = 0;
  long double W11 = 0;
  long double W22 = 0;
  for (int i = 0; i < 3; i++)
  {
    MatrixToolbox::zeros(W);
    long double Wmax = INFINITY;
    long double hmax = INFINITY;
    long double Wmin = 100*EVZ;
    long double hmin = 0;
    long double performance = INFINITY;
    while (abs(1 - performance/TOLDISTURBANCE) > DISTCONVERGED)
    {
      W->real[i][i] = pow(10, 0.5*(log10(Wmax) + log10(Wmin)));
      MatrixToolbox::multiply(sysP->B1, W, B1W);
      MatrixToolbox::transpose(sysP->B1, B1t);
      MatrixToolbox::multiply(B1W, B1t, B1WB1t);
      ControlToolbox::lyap(sysP->A, B1WB1t, Y);
      MatrixToolbox::transpose(sysP->C2, C2t);
      MatrixToolbox::multiply(sysP->C2, Y, C2Y);
      MatrixToolbox::multiply(C2Y, C2t, X);

      if (i == 0)
      {
        performance = sqrt(X->real[0][0])/v0;
        W00 = W->real[i][i];
      }
      else if (i == 1)
      {
        performance = sqrt(X->real[1][1])/aoa0;
        W11 = W->real[i][i];
      }
      else if (i == 2)
      {
        performance = sqrt(X->real[1][1])/aoa0;
        W22 = W->real[i][i];
      }

      // update (Wmax, hmax), (Wmin, hmin)
      if (performance > TOLDISTURBANCE)
      {
        Wmax = W->real[i][i];
        hmax = performance;
      }
      else
      {
        Wmin = W->real[i][i];
        hmin = performance;
      }
    }
  }

  // combine the disturbance into a single covariance analysis
  W->real[0][0] = W00;                          // v disturbance
  W->real[1][1] = W11;                          // aoa disturbance
  W->real[2][2] = W22;                          // q disturbance
  MatrixToolbox::multiply(sysP->B1, W, B1W);
  MatrixToolbox::transpose(sysP->B1, B1t);
  MatrixToolbox::multiply(B1W, B1t, B1WB1t);
  ControlToolbox::lyap(sysP->A, B1WB1t, Y);
  MatrixToolbox::transpose(sysP->C2, C2t);
  MatrixToolbox::multiply(sysP->C2, Y, C2Y);
  MatrixToolbox::multiply(C2Y, C2t, X);
  cout << "open-loop disturbance response:" << endl;
  *performancev = sqrt(X->real[0][0])/v0;
  *performanceaoa = sqrt(X->real[1][1])/aoa0;
  *performanceomega = sqrt(X->real[3][3])/omega0;

  // balance plant model
  SYSTEM_MODEL *sysPb = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysPb, "sysPb", n, m1, m2, r1, r2);
  ControlToolbox::balance(sysP, sysPb);

  // set up system model for Cramer-Rao bounding analysis
  MatrixToolbox::equate(sysPb->A, sysG->A);
  MatrixToolbox::equate(sysPb->B1, sysG->B1);
  MatrixToolbox::equate(sysPb->B2, sysG->B2);
  MatrixToolbox::equate(sysPb->C1, sysG->C1);
  MatrixToolbox::equate(sysPb->C2, sysG->C2);
  MatrixToolbox::equate(sysPb->D11, sysG->D11);
  MatrixToolbox::equate(sysPb->D12, sysG->D12);
  MatrixToolbox::equate(sysPb->D21, sysG->D21);
  MatrixToolbox::equate(sysPb->D22, sysG->D22);

  // with the above disturbances, determine the measurement noise that results in TOLNOISE estimation errors
  long double W33 = 0;
  long double W44 = 0;
  long double W55 = 0;
  long double W66 = 0;
  for (int i = 3; i < 7; i++)
  {
    // initialize W[i][i] for all sensor noises to infinity
    W->real[3][3] = INFINITY;                   // v sensor noise
    W->real[4][4] = INFINITY;                   // aoa sensor noise
    W->real[5][5] = INFINITY;                   // q sensor noise
    W->real[6][6] = INFINITY;                   // omega sensor noise

    // set limits on W[i][i]
    long double Wmax = W->real[i][i];
    long double hmax = TOLDISTURBANCE;
    long double Wmin = 100*EVZ;
    long double hmin = 0;
    long double performance = INFINITY;
    int iterations = 0;
    while ((abs(1 - performance/TOLNOISE) > DISTCONVERGED) && (iterations < MAXDISTITERS))
    {
      W->real[i][i] = sqrt(Wmin*Wmax);
      MatrixToolbox::sqrtm(W, sqrtW);
      MatrixToolbox::multiply(sysPb->B1, sqrtW, B1);
      MatrixToolbox::multiply(sysPb->D21, sqrtW, D21);
      MatrixToolbox::equate(B1, sysG->B1);
      MatrixToolbox::equate(D21, sysG->D21);
      ControlToolbox::kf(sysG, K, Y);
      MatrixToolbox::transpose(sysG->C2, C2t);
      MatrixToolbox::multiply(sysG->C2, Y, C2Y);
      MatrixToolbox::multiply(C2Y, C2t, X);
      if (i == 3)
      {
        performance = sqrt(X->real[0][0])/v0;
        W33 = W->real[i][i];
      }
      else if (i == 4)
      {
        performance = sqrt(X->real[1][1])/aoa0;
        W44 = W->real[i][i];
      }
      else if (i == 5)
      {
        performance = sqrt(X->real[1][1])/aoa0;
        W55 = W->real[i][i];
      }
      else if (i == 6)
      {
        performance = sqrt(X->real[3][3])/omega0;
        W66 = W->real[i][i];
      }

      // update (Wmax, hmax), (Wmin, hmin)
      if (performance > TOLNOISE)
      {
        Wmax = W->real[i][i];
        hmax = performance;
      }
      else
      {
        Wmin = W->real[i][i];
        hmin = performance;
      }
      iterations++;
    }
  }

  // put results into W
  W->real[0][0] = W00;                          // v disturbance
  W->real[1][1] = W11;                          // aoa disturbance
  W->real[2][2] = W22;                          // q disturbance
  W->real[3][3] = W33;                          // v sensor noise
  W->real[4][4] = W44;                          // aoa sensor noise
  W->real[5][5] = W55;                          // q sensor noise
  W->real[6][6] = W66;                          // omega sensor noise

// vary each element of W b y a factor of 0.1 to 10, 11 points each

// put disturbance levels in first column
  MatrixToolbox::linspace(W00/10, 10*W00, Wv00);
  MatrixToolbox::linspace(W11/10, 10*W11, Wv11);
  MatrixToolbox::linspace(W22/10, 10*W22, Wv22);
  MatrixToolbox::linspace(W33/10, 10*W33, Wv33);
  MatrixToolbox::linspace(W44/10, 10*W44, Wv44);
  MatrixToolbox::linspace(W55/10, 10*W55, Wv55);
  MatrixToolbox::linspace(W66/10, 10*W66, Wv66);

  for (int i = 0; i < W->rows; i++)
  {
    // initialize W[i][i] for all sensor noises to infinity
    W->real[0][0] = W00;                        // v disturbance
    W->real[1][1] = W11;                        // aoa disturbance
    W->real[2][2] = W22;                        // q disturbance
    W->real[3][3] = W33;                        // v sensor noise
    W->real[4][4] = W44;                        // aoa sensor noise
    W->real[5][5] = W55;                        // q sensor noise
    W->real[6][6] = W66;                        // omega sensor noise
    for (int j = 0; j < Wv00->rows; j++)
    {
      // set noise intensity
      if (i == 0)
        W->real[i][i] = Wv00->real[j][0];
      else if (i == 1)
        W->real[i][i] = Wv11->real[j][0];
      else if (i == 2)
        W->real[i][i] = Wv22->real[j][0];
      else if (i == 3)
        W->real[i][i] = Wv33->real[j][0];
      else if (i == 4)
        W->real[i][i] = Wv44->real[j][0];
      else if (i == 5)
        W->real[i][i] = Wv55->real[j][0];
      else if (i == 6)
        W->real[i][i] = Wv66->real[j][0];

      // compute performance
      MatrixToolbox::sqrtm(W, sqrtW);
      MatrixToolbox::multiply(sysPb->B1, sqrtW, B1);
      MatrixToolbox::multiply(sysPb->D21, sqrtW, D21);
      MatrixToolbox::equate(B1, sysG->B1);
      MatrixToolbox::equate(D21, sysG->D21);
      ControlToolbox::kf(sysG, K, Y);
      MatrixToolbox::transpose(sysG->C2, C2t);
      MatrixToolbox::multiply(sysG->C2, Y, C2Y);
      MatrixToolbox::multiply(C2Y, C2t, X);

      // save performance
      if (i == 0)
      {
        Wv00->real[j][1] = sqrt(X->real[0][0])/v0;
      }
      else if (i == 1)
      {
        Wv11->real[j][1] = sqrt(X->real[1][1])/aoa0;
      }
      else if (i == 2)
      {
        Wv22->real[j][1] = sqrt(X->real[1][1])/aoa0;
      }
      else if (i == 3)
      {
        Wv33->real[j][1] = sqrt(X->real[0][0])/v0;
      }
      else if (i == 4)
      {
        Wv44->real[j][1] = sqrt(X->real[1][1])/aoa0;
      }
      else if (i == 5)
      {
        Wv55->real[j][1] = sqrt(X->real[1][1])/aoa0;
      }
      else if (i == 6)
      {
        Wv66->real[j][1] = sqrt(X->real[3][3])/omega0;
      }
    }
  }

  // reset W and compute expected performance
  W33 = INFINITY;
  W55 = INFINITY;
  W->real[0][0] = W00;                          // v disturbance
  W->real[1][1] = W11;                          // aoa disturbance
  W->real[2][2] = W22;                          // q disturbance
  W->real[3][3] = W33;                          // v sensor noise
  W->real[4][4] = W44;                          // aoa sensor noise
  W->real[5][5] = W55;                          // q sensor noise
  W->real[6][6] = W66;                          // omega sensor noise

  // compute the performance with all disturbances and sensor noises active, using the input model sysP
  MatrixToolbox::equate(sysP->A, sysG->A);
  MatrixToolbox::equate(sysP->B1, sysG->B1);
  MatrixToolbox::equate(sysP->B2, sysG->B2);
  MatrixToolbox::equate(sysP->C1, sysG->C1);
  MatrixToolbox::equate(sysP->C2, sysG->C2);
  MatrixToolbox::equate(sysP->D11, sysG->D11);
  MatrixToolbox::equate(sysP->D12, sysG->D12);
  MatrixToolbox::equate(sysP->D21, sysG->D21);
  MatrixToolbox::equate(sysP->D22, sysG->D22);

  MatrixToolbox::sqrtm(W, sqrtW);
  MatrixToolbox::multiply(sysP->B1, sqrtW, B1);
  MatrixToolbox::multiply(sysP->D21, sqrtW, D21);
  MatrixToolbox::equate(B1, sysG->B1);
  MatrixToolbox::equate(D21, sysG->D21);
  ControlToolbox::kf(sysG, K, Y);
  MatrixToolbox::transpose(sysG->C2, C2t);
  MatrixToolbox::multiply(sysG->C2, Y, C2Y);
  MatrixToolbox::multiply(C2Y, C2t, X);

  // compute required bandwidths
  ControlToolbox::kfbw(sysG, K, kfBW);

  // display the results
  *performancev = sqrt(X->real[0][0])/v0;
  *performanceaoa = sqrt(X->real[1][1])/aoa0;
  *performanceomega = sqrt(X->real[3][3])/omega0;

  // free allocated space
  MatrixToolbox::destroy(B1W);
  MatrixToolbox::destroy(B1t);
  MatrixToolbox::destroy(B1WB1t);
  MatrixToolbox::destroy(sqrtW);
  MatrixToolbox::destroy(B1);
  MatrixToolbox::destroy(D21);
  MatrixToolbox::destroy(Y);
  MatrixToolbox::destroy(C2t);
  MatrixToolbox::destroy(C2Y);
  MatrixToolbox::destroy(X);
  ControlToolbox::destroy(sysPb);
  ControlToolbox::destroy(sysG);
}

//=====================================================================================
// end of method 'StaticCRB'
//=====================================================================================

//=====================================================================================
// method 'DynamicCRB'
//
// description:
//   This method performs an analysis of the turboelectric aircraft based 
//   on the model (see notes in SystemModel.sci)
//
//    xtildadot =  A*xvtilda +  B1*wv +  B2*uvtilda
//       ytilda = C2*xvtilda + D21*wv + D21*utilda
//
//   to find the performance from the disturbance levels computed in StaticCRB 
//   when the system uses an optimal kalman filter as the state estimator
//
// J R Dowdle
// 26-Aug-2017
//=====================================================================================

// beginning of method 'DynamicCRB'

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
  MATRIX_DOUBLE *SIGomegakf)                    // kalman filter standard deviation of propulsion shaft angular rate estimation error
{
  // executable code

  // allocate space
  int n = sysP->xdim;
  int m1 = sysP->wdim;
  MATRIX_DOUBLE *wv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(wv, "wv", 0, m1, stime->rows);
  MATRIX_DOUBLE *Wv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv, "Wv", 0, m1, stime->rows);
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
    if ((stime->real[k][0] >= pulseStart) && (stime->real[k][0] <= (pulseStart + pulseLength)))
    {
      for (int i = 0; i < m1; i++)
      {
        wv->real[i][k] = disturbance->real[i][0];
      }
      for (int i = 0; i < m1; i++)
      {
        Wv->real[i][k] = W->real[i][i];
      }
    }
  }
  // compute mean and variance of state vector
  ControlToolbox::SystemSimulation(sysP, stime, wv, Wv, xv, Xv);

  // unwind variables into output vectors
  for (int i = 0; i < stime->rows; i++)
  {
    vol->real[i][0] = xv->real[0][i];
    aoaol->real[i][0] = xv->real[1][i];
    qol->real[i][0] = xv->real[3][i];
    omegaol->real[i][0] = xv->real[4][i];
    SIGvol->real[i][0] = sqrt(Xv->real[0][i]);
    SIGaoaol->real[i][0] = sqrt(Xv->real[1][i]);
    SIGqol->real[i][0] = sqrt(Xv->real[3][i]);
    SIGomegaol->real[i][0] = sqrt(Xv->real[4][i]);
  }

  // compute the kalman filter error analysis vs time
  ControlToolbox::KFSimulation(sysP, K, W, stime, wv, Wv, ev, Ev);

  // unwind variables into output vectors
  for (int i = 0; i < stime->rows; i++)
  {
    vkf->real[i][0] = ev->real[0][i];
    aoakf->real[i][0] = ev->real[1][i];
    qkf->real[i][0] = ev->real[3][i];
    omegakf->real[i][0] = ev->real[4][i];
    SIGvkf->real[i][0] = sqrt(Ev->real[0][i]);
    SIGaoakf->real[i][0] = sqrt(Ev->real[1][i]);
    SIGqkf->real[i][0] = sqrt(Ev->real[3][i]);
    SIGomegakf->real[i][0] = sqrt(Ev->real[4][i]);
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
// end of method 'DynamicCRB'
//=====================================================================================

//=====================================================================================
// end of module 'CRB'
//=====================================================================================