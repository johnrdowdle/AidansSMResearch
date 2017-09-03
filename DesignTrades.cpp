//=====================================================================================
// module 'DesignTrades'
//
// description:
//  This module runs the full set of methods that perform design trades for the 
//  turboelectric aircraft.
//
// J R Dowdle
// 14-Aug-2017
//=====================================================================================

// include headers, macros, defined constants, procedure prototypes
#include "stdafx.h"
#include "Aircraft.h"
#include "CRB.h"
#include "DesignTrades.h"
#include "LQR.h"
#include <fstream>

// included namespaces
using namespace std;  
using namespace MatrixLibrary;
using namespace ControlSystemLibrary;

// beginning of module 'DesignTrades'

//=====================================================================================
// method 'main'
//
// description:
//  This method inputs data and runs the design trades for the turboelectric aircraft.
//
// J R Dowdle
// 24-Aug-2017
//=====================================================================================

// beginning of method 'main'

int main(
  int argc,                                     // number of arguments to main
  char * argv[],                                // inputs to main
  char * envp[])                                // environmental input to main
{
  // executable code

  // set file names from input arguments
  string inFile, dataFolder;
  if (argc >= 3)
  {
    inFile = argv[1];                           // input file name
    dataFolder = argv[2];                       // output data folder name
  }
  else
  {
    inFile = "U:\\John\\Documents\\Software Projects\\AidansSMResearch\\CramerRaoBound\\CRB\\input.dat";
    dataFolder = "U:\\John\\Documents\\Software Projects\\AidansSMResearch\\CramerRaoBound\\CRB";
  }

  // set desired trim condition (1, 2 or 3)
  int trim;
  do
  {
    cout << "Enter trim condition (1, 2, 3): " << endl;
    cin >> trim;
  } while ((trim != 1) && (trim != 2) && (trim != 3));

  // set ouotput folder based upon trim condition
  string TrimFolder;
  if (trim == 1)
    TrimFolder = "TrimPoint1";
  else if (trim == 2)
    TrimFolder = "TrimPoint2";
  else if (trim == 3)
    TrimFolder = "TrimPoint3";

  // set file names
  string diaryFile = dataFolder + "//" + TrimFolder + "//diary.dat";
  string resultsFile = dataFolder + "//" + TrimFolder + "//results.dat";
  string freqrspFile = dataFolder + "//" + TrimFolder + "//freqrsp.dat";
  string timerspFile = dataFolder + "//" + TrimFolder + "//timersp.dat";

  // open diary and results files
  ofstream diary(diaryFile);
  ofstream results(resultsFile);
  ofstream timersp(timerspFile);
  ofstream freqrsp(freqrspFile);

  // check that all io streams are open
  if (!diary)
  {
    cout << "WARNING:  diary file not open, execution terminating";
    exit(1);
  }
  else if (!results)
  {
    cout << "WARNING:  results file not open, execution terminating";
    exit(1);
  }
  else if (!timersp)
  {
    cout << "WARNING:  timersp file not open, execution terminating";
    exit(1);
  }
  else if (!freqrsp)
  {
    cout << "WARNING:  freqrsp file not open, execution terminating";
    exit(1);
  }

  // read input data 
  // print header to screen
  cout << endl << endl << "Executing:  ReadData" << endl;
  diary << endl << endl << "Executing:  ReadData" << endl;
  time_t rawtime;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  MATRIX_DOUBLE *aircraftDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(aircraftDAT, "aircraftDAT", 0, 26, 1);
  MATRIX_DOUBLE *analysisDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(analysisDAT, "analysisDAT", 0, 6, 1);
  MATRIX_DOUBLE *propulsionDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(propulsionDAT, "propulsionDAT", 0, 5, 1);
  MATRIX_DOUBLE *simulationDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(simulationDAT, "simulationDAT", 0, 9, 1);
  MATRIX_DOUBLE *LQRDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(LQRDAT, "LQRDAT", 0, 9, 1);

  ReadData(inFile, dataFolder, aircraftDAT, analysisDAT, propulsionDAT, simulationDAT, LQRDAT);

  // set up the model
  // print header to screen
  cout << endl << endl << "Executing:  SystemModel" << endl;
  diary << endl << endl << "Executing:  SystemModel" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  SYSTEM_MODEL *sysGv = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysGv, "sysGv", 4, 6, 2, 4, 3);
  SYSTEM_MODEL *sysGp = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysGp, "sysGp", 1, 1, 1, 2, 1);
  SYSTEM_MODEL *sysGs = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysGs, "sysGs", 5, 7, 3, 6, 4);

  SystemModel(aircraftDAT, propulsionDAT, trim, sysGv, sysGp, sysGs);

  int n = sysGs->xdim;
  int m1 = sysGs->wdim;
  int m2 = sysGs->udim;
  int r1 = sysGs->zdim;
  int r2 = sysGs->ydim;

  ControlToolbox::display(sysGv);
  ControlToolbox::display(sysGp);
  ControlToolbox::display(sysGs);
  ControlToolbox::display(sysGv, diary);
  ControlToolbox::display(sysGp, diary);
  ControlToolbox::display(sysGs, diary);

  // compute open-loop frequency responses of integrated model
  // print header to screen
  cout << endl << endl << "Executing:  OpenLoopFrequencyResponse" << endl;
  diary << endl << endl << "Executing:  OpenLoopFrequencyResponse" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  int fDecMin = analysisDAT->real[0][0];
  int fDecMax = analysisDAT->real[1][0];
  int fNpts = analysisDAT->real[2][0];
  MATRIX_DOUBLE *frequency = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(frequency, "frequency", 0, fNpts, 1);
  MATRIX_DOUBLE *svGsmin = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGsmin, "svGsmin", 0, fNpts, 1);
  MATRIX_DOUBLE *svGsmax = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGsmax, "svGsmax", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs11min = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs11min, "svGs11min", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs11max = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs11max, "svGs11max", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs12min = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs12min, "svGs12min", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs12max = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs12max, "svGs12max", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs21min = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs21min, "svGs21min", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs21max = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs21max, "svGs21max", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs22min = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs22min, "svGs22min", 0, fNpts, 1);
  MATRIX_DOUBLE *svGs22max = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svGs22max, "svGs22max", 0, fNpts, 1);

  MatrixToolbox::logspace(fDecMin, fDecMax, frequency);

  // compute frequency response
  ControlToolbox::freqrsp(frequency, sysGs, svGsmin, svGsmax, svGs11min, svGs11max, svGs12min, svGs12max,
    svGs21min, svGs21max, svGs22min, svGs22max);

  cout << setw(19) << "frequency [r/s]" << setw(16) << "Gs-minimum" << setw(20) << "Gs-maximum" << setw(22) <<
    "Gs11-minimum" << setw(20) << "Gs11-maximum" << setw(20) << "Gs12-minimum" << setw(20) << "Gs12-maximum" << setw(20) <<
    "Gs21-minimum" << setw(20) << "Gs21-maximum" << setw(20) << "Gs22-minimum" << setw(20) << "Gs22-maximum" << endl;
  diary << setw(19) << "frequency [r/s]" << setw(16) << "Gs-minimum" << setw(20) << "Gs-maximum" << setw(22) <<
    "Gs11-minimum" << setw(20) << "Gs11-maximum" << setw(20) << "Gs12-minimum" << setw(20) << "Gs12-maximum" << setw(20) <<
    "Gs21-minimum" << setw(20) << "Gs21-maximum" << setw(20) << "Gs22-minimum" << setw(20) << "Gs22-maximum" << endl;
  freqrsp << setw(19) << "frequency [r/s]" << setw(16) << "Gs-minimum" << setw(20) << "Gs-maximum" << setw(22) <<
    "Gs11-minimum" << setw(20) << "Gs11-maximum" << setw(20) << "Gs12-minimum" << setw(20) << "Gs12-maximum" << setw(20) <<
    "Gs21-minimum" << setw(20) << "Gs21-maximum" << setw(20) << "Gs22-minimum" << setw(20) << "Gs22-maximum" << endl;
  for (int i = 0; i < fNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svGsmin->real[i][0] << setw(20) << scientific << setprecision(8) << svGsmax->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svGs11min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs11max->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svGs12min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs12max->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svGs21min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs21max->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svGs22min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs22max->real[i][0] << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGsmin->real[i][0] << setw(20) << scientific << setprecision(8) << svGsmax->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs11min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs11max->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs12min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs12max->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs21min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs21max->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs22min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs22max->real[i][0] << endl;
    freqrsp << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGsmin->real[i][0] << setw(20) << scientific << setprecision(8) << svGsmax->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs11min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs11max->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs12min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs12max->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs21min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs21max->real[i][0] << setw(20) << scientific << setprecision(8) <<
      svGs22min->real[i][0] << setw(20) << scientific << setprecision(8) << svGs22max->real[i][0] << endl;
  }

  // free allocated space
  MatrixToolbox::destroy(svGsmin);
  MatrixToolbox::destroy(svGsmax);
  MatrixToolbox::destroy(svGs11min);
  MatrixToolbox::destroy(svGs11max);
  MatrixToolbox::destroy(svGs12min);
  MatrixToolbox::destroy(svGs12max);
  MatrixToolbox::destroy(svGs21min);
  MatrixToolbox::destroy(svGs21max);
  MatrixToolbox::destroy(svGs22min);
  MatrixToolbox::destroy(svGs22max);

  // perform static Cramer-Rao Bounding analysis   
  // print header to screen
  cout << endl << endl << "Executing:  StaticCRB" << endl;
  diary << endl << endl << "Executing:  StaticCRB" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  MATRIX_DOUBLE *W = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(W, "W", 0, 7, 7);
  MATRIX_DOUBLE *Wv00 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv00, "Wv00", 0, 11, 2);
  MATRIX_DOUBLE *Wv11 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv11, "Wv11", 0, 11, 2);
  MATRIX_DOUBLE *Wv22 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv22, "Wv22", 0, 11, 2);
  MATRIX_DOUBLE *Wv33 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv33, "Wv33", 0, 11, 2);
  MATRIX_DOUBLE *Wv44 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv44, "Wv44", 0, 11, 2);
  MATRIX_DOUBLE *Wv55 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv55, "Wv55", 0, 11, 2);
  MATRIX_DOUBLE *Wv66 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv66, "Wv66", 0, 11, 2);
  MATRIX_DOUBLE *Wv00t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv00t, "velocity disturbance intensity [(ft/s)^2/s]", 0, 2, 11);
  MATRIX_DOUBLE *Wv11t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv11t, "angle-of-attack disturbance intensity [(r)^2/s]", 0, 2, 11);
  MATRIX_DOUBLE *Wv22t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv22t, "pitch rate disturbance intensity [(r/s)^2/s]", 0, 2, 11);
  MATRIX_DOUBLE *Wv33t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv33t, "velocity sensor noise intensity [(ft/s)^2/(r/s)]", 0, 2, 11);
  MATRIX_DOUBLE *Wv44t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv44t, "angle-of-attack sensor noise intensity [(r)^2/(r/s)]", 0, 2, 11);
  MATRIX_DOUBLE *Wv55t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv55t, "pitch rate sensor noise intensity [(r/s)^2/(r/s)]", 0, 2, 11);
  MATRIX_DOUBLE *Wv66t = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv66t, "propulsion shaft angular rate sensor noise intensity [(r/s)^2/(r/s)]", 0, 2, 11);
  MATRIX_DOUBLE *K = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(K, "K", 0, n, r2);
  MATRIX_DOUBLE *kfbandwidth = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(kfbandwidth, "kfbandwidth", 0, 1, sysGs->ydim);
  MATRIX_DOUBLE *Wv = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wv, "Wv", 0, 1, 7);

  long double performancev;
  long double performanceaoa;
  long double performanceomega;
  StaticCRB(sysGs, sysGs->xref->real[0][0], sysGs->xref->real[1][0], sysGs->xref->real[4][0], 
    W, Wv00, Wv11, Wv22, Wv33, Wv44, Wv55, Wv66, K, kfbandwidth, &performancev, &performanceaoa, &performanceomega);
  cout << "closed-loop disturbance response:" << endl;
  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) <<
    "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) <<
    "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "W[0][0] = " << setw(20) << scientific << setprecision(8) << W->real[0][0] << setw(20) <<
    "W[1][1] = " << setw(20) << scientific << setprecision(8) << W->real[1][1] << setw(20) <<
    "W[2][2] = " << setw(20) << scientific << setprecision(8) << W->real[2][2] << setw(20) <<
    "W[3][3] = " << setw(20) << scientific << setprecision(8) << W->real[3][3] << setw(20) <<
    "W[4][4] = " << setw(20) << scientific << setprecision(8) << W->real[4][4] << setw(20) <<
    "W[5][5] = " << setw(20) << scientific << setprecision(8) << W->real[5][5] << setw(20) <<
    "W[6][6] = " << setw(20) << scientific << setprecision(8) << W->real[6][6] << endl << endl;
  cout << setw(20) << "Sensor Bandwidth Requirements:" << endl;
  cout << setw(20) << "Velocity [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][0]/2/acos(-1) << setw(20) <<
    "AOA [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][1]/2/acos(-1) << setw(20) <<
    "Pitch Rate [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][2]/2/acos(-1) << setw(20) <<
    "Propulsion [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][3]/2/acos(-1) << endl << endl;
  MatrixToolbox::display(K);

  diary << "closed-loop disturbance response:" << endl;
  diary << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) <<
    "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) <<
    "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  diary << setw(20) << "W[0][0] = " << setw(20) << scientific << setprecision(8) << W->real[0][0] << setw(20) <<
    "W[1][1] = " << setw(20) << scientific << setprecision(8) << W->real[1][1] << setw(20) <<
    "W[2][2] = " << setw(20) << scientific << setprecision(8) << W->real[2][2] << setw(20) <<
    "W[3][3] = " << setw(20) << scientific << setprecision(8) << W->real[3][3] << setw(20) <<
    "W[4][4] = " << setw(20) << scientific << setprecision(8) << W->real[4][4] << setw(20) <<
    "W[5][5] = " << setw(20) << scientific << setprecision(8) << W->real[5][5] << setw(20) <<
    "W[6][6] = " << setw(20) << scientific << setprecision(8) << W->real[6][6] << endl << endl;
  diary << setw(20) << "Sensor Bandwidth Requirements:" << endl;
  diary << setw(20) << "Velocity [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][0]/2/acos(-1) << setw(20) <<
    "AOA [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][1]/2/acos(-1) << setw(20) <<
    "Pitch Rate [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][2]/2/acos(-1) << setw(20) <<
    "Propulsion [Hz]:" << setw(20) << scientific << setprecision(8) << kfbandwidth->real[0][3]/2/acos(-1) << endl << endl;
  MatrixToolbox::display(K, diary);

  for (int i = 0; i < 7; i++)
    Wv->real[0][i] = W->real[i][i];
  MatrixToolbox::transpose(Wv00, Wv00t);
  MatrixToolbox::transpose(Wv11, Wv11t);
  MatrixToolbox::transpose(Wv22, Wv22t);
  MatrixToolbox::transpose(Wv33, Wv33t);
  MatrixToolbox::transpose(Wv44, Wv44t);
  MatrixToolbox::transpose(Wv55, Wv55t);
  MatrixToolbox::transpose(Wv66, Wv66t);

  cout << "nominal disturbance intensities:" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][0] << " (ft/s)^2/s" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][1] << " (r)^2/s" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][2] << " (r/s)^2/s" << endl << endl;
  MatrixToolbox::display(Wv00t);
  MatrixToolbox::display(Wv11t);
  MatrixToolbox::display(Wv22t);
  cout << "nominal sensor noise intensities:" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][3] << " (ft/s)^2/(r/s)" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][4] << " (r)^2/(r/s)" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][5] << " (r/s)^2/(r/s)" << endl;
  cout << setw(20) << scientific << setprecision(8) << Wv->real[0][6] << " (r/s)^2/(r/s)" << endl << endl;
  MatrixToolbox::display(Wv33t);
  MatrixToolbox::display(Wv44t);
  MatrixToolbox::display(Wv55t);
  MatrixToolbox::display(Wv66t);
  diary << "// " << "nominal disturbance intensities:" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][0] << " (ft/s)^2/s" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][1] << " (r)^2/s" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][2] << " (r/s)^2/s" << endl << endl;
  MatrixToolbox::display(Wv00t, diary);
  MatrixToolbox::display(Wv11t, diary);
  MatrixToolbox::display(Wv22t, diary);
  diary <<"// " <<  "nominal sensor noise intensities:" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][3] << " (ft/s)^2/(r/s)" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][4] << " (r)^2/(r/s)" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][5] << " (r/s)^2/(r/s)" << endl;
  diary << setw(20) << scientific << setprecision(8) << Wv->real[0][6] << " (r/s)^2/(r/s)" << endl << endl;
  MatrixToolbox::display(Wv33t, diary);
  MatrixToolbox::display(Wv44t, diary);
  MatrixToolbox::display(Wv55t, diary);
  MatrixToolbox::display(Wv66t, diary);

  // free allocated space
  MatrixToolbox::destroy(Wv00);
  MatrixToolbox::destroy(Wv11);
  MatrixToolbox::destroy(Wv22);
  MatrixToolbox::destroy(Wv33);
  MatrixToolbox::destroy(Wv44);
  MatrixToolbox::destroy(Wv55);
  MatrixToolbox::destroy(Wv66);
  MatrixToolbox::destroy(Wv00t);
  MatrixToolbox::destroy(Wv11t);
  MatrixToolbox::destroy(Wv22t);
  MatrixToolbox::destroy(Wv33t);
  MatrixToolbox::destroy(Wv44t);
  MatrixToolbox::destroy(Wv55t);
  MatrixToolbox::destroy(Wv66t);
  MatrixToolbox::destroy(kfbandwidth);
  MatrixToolbox::destroy(Wv);

  // perform frequency domain analysis of Cramer-Rao Bounding analysis   
  // print header to screen
  cout << endl << endl << "Executing:  CRBFrequencyResponse" << endl;
  diary << endl << endl << "Executing:  CRBFrequencyResponse" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // set up system model for the kalman filter

  // allocate space
  MATRIX_DOUBLE *KC2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(KC2, "KC2", 0, n, n);
  MATRIX_DOUBLE *Acl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Acl, "Acl", 0, n, n);
  SYSTEM_MODEL *sysKF = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysKF, "sysKF", n, r2, 0, r2, 0);
  MATRIX_DOUBLE *null = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(null, "null", 0, 0, 0);
  MATRIX_DOUBLE *zero = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(zero, "zero", 0, r2, r2);
  MATRIX_DOUBLE *svKFmin = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svKFmin, "svKFmin", 0, fNpts, 1);
  MATRIX_DOUBLE *svKFmax = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svKFmax, "svKFmax", 0, fNpts, 1);

  // compute kalman filter system model
  MatrixToolbox::multiply(K, sysGs->C2, KC2);
  MatrixToolbox::subtract(sysGs->A, KC2, Acl);
  ControlToolbox::ss2sys(sysKF, Acl, K, null, sysGs->C2, null, zero, null, null, null);
  ControlToolbox::SystemAnalysis(sysKF);
  ControlToolbox::display(sysKF);
  ControlToolbox::display(sysKF, diary);

  // compute frequency response
  ControlToolbox::freqrsp(frequency, sysKF, svKFmin, svKFmax);

  cout << setw(19) << "frequency [r/s]" << setw(16) << "KF sv-minimum" << setw(20) << "KF sv-maximum" << endl;
  diary << setw(19) << "frequency [r/s]" << setw(16) << "KF sv-minimum" << setw(20) << "KF sv-maximum" << endl;
  freqrsp << setw(19) << "frequency [r/s]" << setw(16) << "KF sv-minimum" << setw(20) << "KF sv-maximum" << endl;
  for (int i = 0; i < fNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svKFmin->real[i][0] << setw(20) << scientific << setprecision(8) << svKFmax->real[i][0] << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svKFmin->real[i][0] << setw(20) << scientific << setprecision(8) << svKFmax->real[i][0] << endl;
    freqrsp << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svKFmin->real[i][0] << setw(20) << scientific << setprecision(8) << svKFmax->real[i][0] << endl;
  }

  // free allocated space
  MatrixToolbox::destroy(K);
  MatrixToolbox::destroy(KC2);
  MatrixToolbox::destroy(Acl);
  ControlToolbox::destroy(sysKF);
  MatrixToolbox::destroy(zero);
  MatrixToolbox::destroy(svKFmin);
  MatrixToolbox::destroy(svKFmax);

  // compute time-domain analysis of Cramer-Rao Bounding analysis  
  // print header to screen
  cout << endl << endl << "Executing:  DynamicCRB" << endl;
  diary << endl << endl << "Executing:  DynamicCRB" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // set inputs
  long double tStart = analysisDAT->real[3][0];
  long double tEnd = analysisDAT->real[4][0];
  long double timeStep = analysisDAT->real[5][0];
  long double tNpts = (tEnd - tStart)/timeStep + 1;
  long double pulseStart = simulationDAT->real[0][0];
  long double pulseLength = simulationDAT->real[1][0];

  // allocate space
  MATRIX_DOUBLE *disturbance = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(disturbance, "disturbance", 0, 7, 1);
  MATRIX_DOUBLE *stime = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(stime, "stime", 0, tNpts, 1);
  MATRIX_DOUBLE *vol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(vol, "vol", 0, tNpts, 1);
  MATRIX_DOUBLE *aoaol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(aoaol, "aoaol", 0, tNpts, 1);
  MATRIX_DOUBLE *qol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(qol, "qol", 0, tNpts, 1);
  MATRIX_DOUBLE *omegaol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(omegaol, "omegaol", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGvol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGvol, "SIGvol", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGaoaol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGaoaol, "SIGaoaol", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGqol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGqol, "SIGqol", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGomegaol = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGomegaol, "SIGomegaol", 0, tNpts, 1);
  MATRIX_DOUBLE *vkf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(vkf, "vkf", 0, tNpts, 1);
  MATRIX_DOUBLE *aoakf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(aoakf, "aoakf", 0, tNpts, 1);
  MATRIX_DOUBLE *qkf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(qkf, "qkf", 0, tNpts, 1);
  MATRIX_DOUBLE *omegakf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(omegakf, "omegakf", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGvkf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGvkf, "SIGvkf", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGaoakf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGaoakf, "SIGaoakf", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGqkf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGqkf, "SIGqkf", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGomegakf = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGomegakf, "SIGomegakf", 0, tNpts, 1);

  W->real[3][3] = INFINITY;
  W->real[5][5] = INFINITY;

  disturbance->real[0][0] = simulationDAT->real[2][0];
  disturbance->real[1][0] = simulationDAT->real[3][0];
  disturbance->real[2][0] = simulationDAT->real[4][0];
  disturbance->real[3][0] = simulationDAT->real[5][0];
  disturbance->real[4][0] = simulationDAT->real[6][0];
  disturbance->real[5][0] = simulationDAT->real[7][0];
  disturbance->real[6][0] = simulationDAT->real[8][0];

  MatrixToolbox::linspace(0.0, tEnd, stime);

  DynamicCRB(sysGs, K, W, disturbance, pulseStart, pulseLength, stime, vol, aoaol, qol, omegaol, SIGvol, SIGaoaol, SIGqol, SIGomegaol, 
    vkf, aoakf, qkf, omegakf, SIGvkf, SIGaoakf, SIGqkf, SIGomegakf);

  cout << setw(19) << "time [s]" << setw(16) << "vol" << setw(20) << "aoaol" << setw(22) << "qol" << setw(20) << "omegaol" << setw(20) <<
    "SIGvol" << setw(20) << "SIGaoaol" << setw(20) << "SIGqol" << setw(20) <<  "SIGomegaol" << setw(20) << "vkf" << setw(20) << "aoakf" << setw(22) <<
    "qkf" << setw(20) << "omegakf" << setw(20) << "SIGvkf" << setw(20) << "SIGaoakf" << setw(20) << "SIGqkf" << setw(20) << "SIGomegakf" << endl;
  diary << setw(19) << "time [s]" << setw(16) << "vol" << setw(20) << "aoaol" << setw(22) << "qol" << setw(20) << "omegaol" << setw(20) <<
    "SIGvol" << setw(20) << "SIGaoaol" << setw(20) << "SIGqol" << setw(20) <<  "SIGomegaol" << setw(20) << "vkf" << setw(20) << "aoakf" << setw(22) <<
    "qkf" << setw(20) << "omegakf" << setw(20) << "SIGvkf" << setw(20) << "SIGaoakf" << setw(20) << "SIGqkf" << setw(20) << "SIGomegakf" << endl;
  timersp << setw(19) << "time [s]" << setw(16) << "vol" << setw(20) << "aoaol" << setw(22) << "qol" << setw(20) << "omegaol" << setw(20) <<
    "SIGvol" << setw(20) << "SIGaoaol" << setw(20) << "SIGqol" << setw(20) <<  "SIGomegaol" << setw(20) << "vkf" << setw(20) << "aoakf" << setw(22) <<
    "qkf" << setw(20) << "omegakf" << setw(20) << "SIGvkf" << setw(20) << "SIGaoakf" << setw(20) << "SIGqkf" << setw(20) << "SIGomegakf" << endl;
  for (int i = 0; i < tNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) <<
      stime->real[i][0] << setw(20) << scientific << setprecision(8) <<vol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      aoaol->real[i][0] << setw(20) << scientific << setprecision(8) << qol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      omegaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvol->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqol->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      vkf->real[i][0] << setw(20) << scientific << setprecision(8) << aoakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      qkf->real[i][0] << setw(20) << scientific << setprecision(8) << omegakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvkf->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqkf->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegakf->real[i][0] << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) <<
      stime->real[i][0] << setw(20) << scientific << setprecision(8) <<vol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      aoaol->real[i][0] << setw(20) << scientific << setprecision(8) << qol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      omegaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvol->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqol->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      vkf->real[i][0] << setw(20) << scientific << setprecision(8) << aoakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      qkf->real[i][0] << setw(20) << scientific << setprecision(8) << omegakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvkf->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqkf->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegakf->real[i][0] << endl;
    timersp << "   " << setw(15) << scientific << setprecision(8) <<
      stime->real[i][0] << setw(20) << scientific << setprecision(8) <<vol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      aoaol->real[i][0] << setw(20) << scientific << setprecision(8) << qol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      omegaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvol->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqol->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegaol->real[i][0] << setw(20) << scientific << setprecision(8) <<
      vkf->real[i][0] << setw(20) << scientific << setprecision(8) << aoakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      qkf->real[i][0] << setw(20) << scientific << setprecision(8) << omegakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvkf->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoakf->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqkf->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegakf->real[i][0] << endl;
  }

  // free allocated space
  MatrixToolbox::destroy(vol);
  MatrixToolbox::destroy(aoaol);
  MatrixToolbox::destroy(qol);
  MatrixToolbox::destroy(omegaol);
  MatrixToolbox::destroy(SIGvol);
  MatrixToolbox::destroy(SIGaoaol);
  MatrixToolbox::destroy(SIGqol);
  MatrixToolbox::destroy(SIGomegaol);
  MatrixToolbox::destroy(vkf);
  MatrixToolbox::destroy(aoakf);
  MatrixToolbox::destroy(qkf);
  MatrixToolbox::destroy(omegakf);
  MatrixToolbox::destroy(SIGvkf);
  MatrixToolbox::destroy(SIGaoakf);
  MatrixToolbox::destroy(SIGqkf);
  MatrixToolbox::destroy(SIGomegakf);

  // perform parametric analysis of lqr weights  
  // print header to screen
  cout << endl << endl << "Executing:  LQRWeights" << endl;
  diary << endl << endl << "Executing:  LQRWeights" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  MATRIX_DOUBLE *Q = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Q, "Q", 0, sysGs->zdim, 1);

  // set inputs
  Q->real[0][0] = LQRDAT->real[0][0];
  Q->real[1][0] = LQRDAT->real[1][0];
  Q->real[2][0] = LQRDAT->real[2][0];
  Q->real[3][0] = LQRDAT->real[3][0];
  Q->real[4][0] = LQRDAT->real[4][0];
  Q->real[5][0] = LQRDAT->real[5][0];
  long double ThrustMaxBW = LQRDAT->real[6][0];
  long double ElevatorMaxBW = LQRDAT->real[7][0];
  long double PropulsionMaxBW = LQRDAT->real[8][0];

  LQRWeights(sysGs, W, Q, ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW);
  MatrixToolbox::display(Q);
  MatrixToolbox::display(Q, diary);

  // design lqr controller  
  // print header to screen
  cout << endl << endl << "Executing:  LQRDesign" << endl;
  diary << endl << endl << "Executing:  LQRDesign" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  MATRIX_DOUBLE *F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(F, "F", 0, m2, n);
  MATRIX_DOUBLE *lqrBW = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(lqrBW, "lqrBW", 0, 1, m2);
  MATRIX_DOUBLE *X = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(X, "X", 0, n, n);
  MATRIX_DOUBLE *Z = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Z, "Z", 0, r1, r1);
  long double performancedt;
  long double performancede;
  long double performancePGen;

  // set inputs
  Q->real[0][0] = LQRDAT->real[0][0];
  Q->real[1][0] = LQRDAT->real[1][0];
  Q->real[2][0] = LQRDAT->real[2][0];
  Q->real[3][0] = LQRDAT->real[3][0];
  Q->real[4][0] = LQRDAT->real[4][0];
  Q->real[5][0] = LQRDAT->real[5][0];

  LQRDesign(sysGs, W, Q, F, lqrBW, X, Z, &performancev, &performanceaoa, &performanceomega, &performancedt, &performancede, &performancePGen);

  cout << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) <<
    "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) <<
    "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  cout << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) <<
    "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) <<
    "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  cout << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) <<
    "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) <<
    "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl << endl;
  MatrixToolbox::display(F);

  diary << setw(20) << "SIGv/v0:" << setw(20) << scientific << setprecision(8) << performancev << setw(20) <<
    "SIGaoa/aoa0:" << setw(20) << scientific << setprecision(8) << performanceaoa << setw(20) <<
    "SIGomega/omega0:" << setw(20) << scientific << setprecision(8) << performanceomega << endl;
  diary << setw(20) << "SIGdt:" << setw(20) << scientific << setprecision(8) << performancedt << setw(20) <<
    "SIGde:" << setw(20) << scientific << setprecision(8) << performancede << setw(20) <<
    "SIGPGen/PGen0:" << setw(20) << scientific << setprecision(8) << performancePGen << endl;
  diary << setw(20) << "Thrust BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][0]/2/acos(-1) << setw(20) <<
    "Elevator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][1]/2/acos(-1) << setw(20) <<
    "Generator BW [Hz]:" << setw(20) << scientific << setprecision(8) << lqrBW->real[0][2]/2/acos(-1) << endl << endl;
  MatrixToolbox::display(F, diary);

  // free allocated space
  MatrixToolbox::destroy(lqrBW);
  MatrixToolbox::destroy(X);
  MatrixToolbox::destroy(Z);

  // perform frequency domain analysis of lqr controller   
  // print header to screen
  cout << endl << endl << "Executing:  LQRFrequencyResponse" << endl;
  diary << endl << endl << "Executing:  LQRFrequencyResponse" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // set up system model for the lqr

  // allocate space
  SYSTEM_MODEL *sysLQR = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysLQR, "sysLQR", n, m1, 0, r1, r2);
  MATRIX_DOUBLE *B2F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B2F, "B2F", 0, n, n);
  MATRIX_DOUBLE *D12F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D12F, "D12F", 0, r1, n);
  MATRIX_DOUBLE *C1cl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C1cl, "C1cl", 0, r1, n);
  MATRIX_DOUBLE *D22F = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D22F, "D22F", 0, r2, n);
  MATRIX_DOUBLE *C2cl = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(C2cl, "C2cl", 0, r2, n);
  MATRIX_DOUBLE *svLQRmin = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svLQRmin, "svLQRmin", 0, fNpts, 1);
  MATRIX_DOUBLE *svLQRmax = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svLQRmax, "svLQRmax", 0, fNpts, 1);

  // compute closed-loop lqr system model
  MatrixToolbox::multiply(sysGs->B2, F, B2F);
  MatrixToolbox::add(sysGs->A, B2F, Acl);
  MatrixToolbox::multiply(sysGs->D12, F, D12F);
  MatrixToolbox::add(sysGs->C1, D12F, C1cl);
  MatrixToolbox::multiply(sysGs->D22, F, D22F);
  MatrixToolbox::add(sysGs->C2, D22F, C2cl);
  ControlToolbox::ss2sys(sysLQR, Acl, sysGs->B1, null, C1cl, C2cl, sysGs->D11, null, sysGs->D21, null);
  ControlToolbox::SystemAnalysis(sysLQR);
  ControlToolbox::display(sysLQR);
  ControlToolbox::display(sysLQR, diary);

  // compute frequency response
  ControlToolbox::freqrsp(frequency, sysLQR, svLQRmin, svLQRmax);

  cout << setw(19) << "frequency [r/s]" << setw(16) << "LQR sv-minimum" << setw(20) << "LQR sv-maximum" << endl;
  diary << setw(19) << "frequency [r/s]" << setw(16) << "LQR sv-minimum" << setw(20) << "LQR sv-maximum" << endl;
  freqrsp << setw(19) << "frequency [r/s]" << setw(16) << "LQR sv-minimum" << setw(20) << "LQR sv-maximum" << endl;
  for (int i = 0; i < fNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svLQRmin->real[i][0] << setw(20) << scientific << setprecision(8) << svLQRmax->real[i][0] << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svLQRmin->real[i][0] << setw(20) << scientific << setprecision(8) << svLQRmax->real[i][0] << endl;
    freqrsp << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svLQRmin->real[i][0] << setw(20) << scientific << setprecision(8) << svLQRmax->real[i][0] << endl;
  }

  // free allocated space
  ControlToolbox::destroy(sysLQR);
  MatrixToolbox::destroy(B2F);
  MatrixToolbox::destroy(D12F);
  MatrixToolbox::destroy(C1cl);
  MatrixToolbox::destroy(D22F);
  MatrixToolbox::destroy(C2cl);
  MatrixToolbox::destroy(svLQRmin);
  MatrixToolbox::destroy(svLQRmax);

  // compute time-domain analysis of lqr controller  
  // print header to screen
  cout << endl << endl << "Executing:  DynamicLQR" << endl;
  diary << endl << endl << "Executing:  DynamicLQR" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  MATRIX_DOUBLE *vlqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(vlqr, "vlqr", 0, tNpts, 1);
  MATRIX_DOUBLE *aoalqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(aoalqr, "aoalqr", 0, tNpts, 1);
  MATRIX_DOUBLE *qlqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(qlqr, "qlqr", 0, tNpts, 1);
  MATRIX_DOUBLE *omegalqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(omegalqr, "omegalqr", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGvlqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGvlqr, "SIGvlqr", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGaoalqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGaoalqr, "SIGaoalqr", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGqlqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGqlqr, "SIGqlqr", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGomegalqr = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGomegalqr, "SIGomegalqr", 0, tNpts, 1);

  DynamicLQR(sysGs, F, K, W, disturbance, pulseStart, pulseLength, stime, vlqr, aoalqr, qlqr, omegalqr, 
    SIGvlqr, SIGaoalqr, SIGqlqr, SIGomegalqr);

  cout << setw(19) << "time [s]" << setw(16) << "vlqr" << setw(20) << "aoalqr" << setw(22) << "qlqr" << setw(20) << "omegalqr" << setw(20) <<
    "SIGvlqr" << setw(20) << "SIGaoalqr" << setw(20) << "SIGqlqr" << setw(20) <<  "SIGomegalqr" << endl;
  diary << setw(19) << "time [s]" << setw(16) << "vlqr" << setw(20) << "aoalqr" << setw(22) << "qlqr" << setw(20) << "omegalqr" << setw(20) <<
    "SIGvlqr" << setw(20) << "SIGaoalqr" << setw(20) << "SIGqlqr" << setw(20) <<  "SIGomegalqr" << endl;
  timersp << setw(19) << "time [s]" << setw(16) << "vlqr" << setw(20) << "aoalqr" << setw(22) << "qlqr" << setw(20) << "omegalqr" << setw(20) <<
    "SIGvlqr" << setw(20) << "SIGaoalqr" << setw(20) << "SIGqlqr" << setw(20) <<  "SIGomegalqr" << endl;
  for (int i = 0; i < tNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) << stime->real[i][0] << setw(20) << scientific << setprecision(8) <<
      vlqr->real[i][0] << setw(20) << scientific << setprecision(8) << aoalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      qlqr->real[i][0] << setw(20) << scientific << setprecision(8) << omegalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvlqr->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqlqr->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegalqr->real[i][0] << setw(20) << scientific << setprecision(8) << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) << stime->real[i][0] << setw(20) << scientific << setprecision(8) <<
      vlqr->real[i][0] << setw(20) << scientific << setprecision(8) << aoalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      qlqr->real[i][0] << setw(20) << scientific << setprecision(8) << omegalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvlqr->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqlqr->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegalqr->real[i][0] << setw(20) << scientific << setprecision(8) << endl;
    timersp << "   " << setw(15) << scientific << setprecision(8) << stime->real[i][0] << setw(20) << scientific << setprecision(8) <<
      vlqr->real[i][0] << setw(20) << scientific << setprecision(8) << aoalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      qlqr->real[i][0] << setw(20) << scientific << setprecision(8) << omegalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGvlqr->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoalqr->real[i][0] << setw(20) << scientific << setprecision(8) <<
      SIGqlqr->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegalqr->real[i][0] << setw(20) << scientific << setprecision(8) << endl;
  }

  // free allocated space
  MatrixToolbox::destroy(F);
  MatrixToolbox::destroy(vlqr);
  MatrixToolbox::destroy(aoalqr);
  MatrixToolbox::destroy(qlqr);
  MatrixToolbox::destroy(omegalqr);
  MatrixToolbox::destroy(SIGvlqr);
  MatrixToolbox::destroy(SIGaoalqr);
  MatrixToolbox::destroy(SIGqlqr);
  MatrixToolbox::destroy(SIGomegalqr);

  // design h2 controller  
  // print header to screen
  cout << endl << endl << "Executing:  H2Synthesis" << endl;
  diary << endl << endl << "Executing:  H2Synthesis" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  SYSTEM_MODEL *sysG = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysG, "sysG", n, m1, m2, r1, r2);
  MATRIX_DOUBLE *sqrtW = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(sqrtW, "sqrtW", 0, m1, m1);
  MATRIX_DOUBLE *B1 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(B1, "B1", 0, n, m1);
  MATRIX_DOUBLE *D21 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(D21, "D21", 0, r2, m1);
  SYSTEM_MODEL *sysK = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysK, "sysK", n, r2, 0, m2, 0);

  // modify model to include weights Q
  MatrixToolbox::equate(sysGs->A, sysG->A);
  MatrixToolbox::equate(sysGs->B1, sysG->B1);
  MatrixToolbox::equate(sysGs->B2, sysG->B2);
  MatrixToolbox::equate(sysGs->C1, sysG->C1);
  MatrixToolbox::equate(sysGs->C2, sysG->C2);
  MatrixToolbox::equate(sysGs->D11, sysG->D11);
  MatrixToolbox::equate(sysGs->D12, sysG->D12);
  MatrixToolbox::equate(sysGs->D21, sysG->D21);
  MatrixToolbox::equate(sysGs->D22, sysG->D22);
  sysG->C1->real[0][0] = sysGs->C1->real[0][0]*Q->real[0][0];
  sysG->C1->real[1][1] = sysGs->C1->real[1][1]*Q->real[1][0];
  sysG->C1->real[4][4] = sysGs->C1->real[4][4]*Q->real[2][0];
  sysG->D12->real[2][0] = sysGs->D12->real[2][0]*Q->real[3][0];
  sysG->D12->real[3][1] = sysGs->D12->real[3][1]*Q->real[4][0];
  sysG->D12->real[5][2] = sysGs->D12->real[5][2]*Q->real[5][0];

  // modify model to include process noise W
  MatrixToolbox::sqrtm(W, sqrtW);
  MatrixToolbox::multiply(sysGs->B1, sqrtW, B1);
  MatrixToolbox::multiply(sysGs->D21, sqrtW, D21);
  MatrixToolbox::equate(B1, sysG->B1);
  MatrixToolbox::equate(D21, sysG->D21);
  
  // design h2 controller 
  ControlToolbox::h2syn(sysG, sysK);
  ControlToolbox::SystemAnalysis(sysK);
  ControlToolbox::display(sysK);
  ControlToolbox::display(sysK, diary);

  // free allocated space
  ControlToolbox::destroy(sysG);
  MatrixToolbox::destroy(sqrtW);
  MatrixToolbox::destroy(B1);
  MatrixToolbox::destroy(D21);

  // perform frequency domain analysis of h2 controller   
  // print header to screen
  cout << endl << endl << "Executing:  H2FrequencyResponse" << endl;
  diary << endl << endl << "Executing:  H2FrequencyResponse" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // compute closed-loop h2 control system

  // allocate space
  SYSTEM_MODEL *sysCLwz = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysCLwz, "sysCLwz", 2*n, m1, 0, r1, 0);
  SYSTEM_MODEL *sysCLwy = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysCLwy, "sysCLwy", 2*n, m1, 0, r2, 0);
  SYSTEM_MODEL *sysCLwu = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysCLwu, "sysCLwu", 2*n, m1, 0, m2, 0);
  MATRIX_DOUBLE *svH2zmin = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svH2zmin, "svH2zmin", 0, fNpts, 1);
  MATRIX_DOUBLE *svH2zmax = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svH2zmax, "svH2zmax", 0, fNpts, 1);
  MATRIX_DOUBLE *svH2umin = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svH2umin, "svH2umin", 0, fNpts, 1);
  MATRIX_DOUBLE *svH2umax = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(svH2umax, "svH2umax", 0, fNpts, 1);

  // compute closed-loop transfer functions
  ControlToolbox::lft(sysGs, sysK, sysCLwz, sysCLwy, sysCLwu);
  ControlToolbox::SystemAnalysis(sysCLwz);
  ControlToolbox::SystemAnalysis(sysCLwy);
  ControlToolbox::SystemAnalysis(sysCLwu);
  ControlToolbox::display(sysCLwu);
  ControlToolbox::display(sysCLwu, diary);

  // compute frequency response from w to z
  ControlToolbox::freqrsp(frequency, sysCLwz, svH2zmin, svH2zmax);

  // compute frequency response from w to u
  ControlToolbox::freqrsp(frequency, sysCLwu, svH2umin, svH2umax);

  cout << setw(19) << "frequency [r/s]" << setw(16) << "H2 CLwz sv-minimum" << setw(20) << "H2 CLwz sv-maximum" << setw(16) << 
    "H2 CLwu sv-minimum" << setw(20) << "H2 CLwu sv-maximum" << endl;
  diary << setw(19) << "frequency [r/s]" << setw(16) << "H2 CLwz sv-minimum" << setw(20) << "H2 CLwz sv-maximum" << setw(16) << 
    "H2 CLwu sv-minimum" << setw(20) << "H2 CLwu sv-maximum" << endl;
  freqrsp << setw(19) << "frequency [r/s]" << setw(16) << "H2 CLwz sv-minimum" << setw(20) << "H2 CLwz sv-maximum" << setw(16) << 
    "H2 CLwu sv-minimum" << setw(20) << "H2 CLwu sv-maximum" << endl;
  for (int i = 0; i < fNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svH2zmin->real[i][0] << setw(20) << scientific << setprecision(8) << svH2zmax->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svH2umin->real[i][0] << setw(20) << scientific << setprecision(8) << svH2umax->real[i][0] << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svH2zmin->real[i][0] << setw(20) << scientific << setprecision(8) << svH2zmax->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svH2umin->real[i][0] << setw(20) << scientific << setprecision(8) << svH2umax->real[i][0] << endl;
    freqrsp << "   " << setw(15) << scientific << setprecision(8) << frequency->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svH2zmin->real[i][0] << setw(20) << scientific << setprecision(8) << svH2zmax->real[i][0] << setw(20) << scientific << setprecision(8) << 
      svH2umin->real[i][0] << setw(20) << scientific << setprecision(8) << svH2umax->real[i][0] << endl;
  }

  // free allocates space
  ControlToolbox::destroy(sysCLwy);
  ControlToolbox::destroy(sysCLwu);
  MatrixToolbox::destroy(svH2zmin);
  MatrixToolbox::destroy(svH2zmax);
  MatrixToolbox::destroy(svH2umin);
  MatrixToolbox::destroy(svH2umax);

  // compute time-domain analysis of h2 controller  
  // print header to screen
  cout << endl << endl << "Executing:  DynamicH2" << endl;
  diary << endl << endl << "Executing:  DynamicH2" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  cout << "... computing ..." << endl << endl;

  // allocate space
  MATRIX_DOUBLE *wvec = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(wvec, "wvec", 0, m1, stime->rows);
  MATRIX_DOUBLE *Wvec = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(Wvec, "Wvec", 0, m1, stime->rows);
  MATRIX_DOUBLE *xH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(xH2, "xH2", 0, 2*n, stime->rows);
  MATRIX_DOUBLE *XH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(XH2, "XH2", 0, 2*n, stime->rows);
  MATRIX_DOUBLE *vH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(vH2, "vH2", 0, tNpts, 1);
  MATRIX_DOUBLE *aoaH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(aoaH2, "aoaH2", 0, tNpts, 1);
  MATRIX_DOUBLE *qH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(qH2, "qH2", 0, tNpts, 1);
  MATRIX_DOUBLE *omegaH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(omegaH2, "omegaH2", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGvH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGvH2, "SIGvH2", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGaoaH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGaoaH2, "SIGaoaH2", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGqH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGqH2, "SIGqH2", 0, tNpts, 1);
  MATRIX_DOUBLE *SIGomegaH2 = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(SIGomegaH2, "SIGomegaH2", 0, tNpts, 1);

  // set up simulation variables
  for (int k = 0; k < stime->rows; k++)
  {
    if ((stime->real[k][0] >= pulseStart) && (stime->real[k][0] <= (pulseStart + pulseLength)))
    {
      for (int i = 0; i < m1; i++)
      {
        wvec->real[i][k] = disturbance->real[i][0];
      }
      for (int i = 0; i < m1; i++)
      {
        Wvec->real[i][k] = W->real[i][i];
      }
    }
  }
  // compute mean and variance of state vector
  ControlToolbox::SystemSimulation(sysCLwz, stime, wvec, Wvec, xH2, XH2);

  // unwind variables into output vectors
  for (int i = 0; i < stime->rows; i++)
  {
    vH2->real[i][0] = xH2->real[0][i];
    aoaH2->real[i][0] = xH2->real[1][i];
    qH2->real[i][0] = xH2->real[3][i];
    omegaH2->real[i][0] = xH2->real[4][i];
    SIGvH2->real[i][0] = sqrt(XH2->real[0][i]);
    SIGaoaH2->real[i][0] = sqrt(XH2->real[1][i]);
    SIGqH2->real[i][0] = sqrt(XH2->real[3][i]);
    SIGomegaH2->real[i][0] = sqrt(XH2->real[4][i]);
  }

  cout << setw(19) << "time [s]" << setw(16) << "vH2" << setw(20) << "aoaH2" << setw(22) << "qH2" << setw(20) << "omegaH2" << setw(20) << 
    "SIGvH2" << setw(20) << "SIGaoaH2" << setw(20) << "SIGqH2" << setw(20) <<  "SIGomegaH2" << endl;
  diary << setw(19) << "time [s]" << setw(16) << "vH2" << setw(20) << "aoaH2" << setw(22) << "qH2" << setw(20) << "omegaH2" << setw(20) << 
    "SIGvH2" << setw(20) << "SIGaoaH2" << setw(20) << "SIGqH2" << setw(20) <<  "SIGomegaH2" << endl;
  timersp << setw(19) << "time [s]" << setw(16) << "vH2" << setw(20) << "aoaH2" << setw(22) << "qH2" << setw(20) << "omegaH2" << setw(20) << 
    "SIGvH2" << setw(20) << "SIGaoaH2" << setw(20) << "SIGqH2" << setw(20) <<  "SIGomegaH2" << endl;
  for (int i = 0; i < tNpts; i++)
  {
    cout << "   " << setw(15) << scientific << setprecision(8) << stime->real[i][0] << setw(20) << scientific << setprecision(8) << 
      vH2->real[i][0] << setw(20) << scientific << setprecision(8) << aoaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      qH2->real[i][0] << setw(20) << scientific << setprecision(8) << omegaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      SIGvH2->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      SIGqH2->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegaH2->real[i][0] << endl;
    diary << "   " << setw(15) << scientific << setprecision(8) << stime->real[i][0] << setw(20) << scientific << setprecision(8) << 
      vH2->real[i][0] << setw(20) << scientific << setprecision(8) << aoaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      qH2->real[i][0] << setw(20) << scientific << setprecision(8) << omegaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      SIGvH2->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      SIGqH2->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegaH2->real[i][0] << endl;
    timersp << "   " << setw(15) << scientific << setprecision(8) << stime->real[i][0] << setw(20) << scientific << setprecision(8) << 
      vH2->real[i][0] << setw(20) << scientific << setprecision(8) << aoaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      qH2->real[i][0] << setw(20) << scientific << setprecision(8) << omegaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      SIGvH2->real[i][0] << setw(20) << scientific << setprecision(8) << SIGaoaH2->real[i][0] << setw(20) << scientific << setprecision(8) << 
      SIGqH2->real[i][0] << setw(20) << scientific << setprecision(8) << SIGomegaH2->real[i][0] << endl;
  }

  // free allocated space
  MatrixToolbox::destroy(wvec);
  MatrixToolbox::destroy(Wvec);
  MatrixToolbox::destroy(xH2);
  MatrixToolbox::destroy(XH2);
  MatrixToolbox::destroy(vH2);
  MatrixToolbox::destroy(aoaH2);
  MatrixToolbox::destroy(qH2);
  MatrixToolbox::destroy(omegaH2);
  MatrixToolbox::destroy(SIGvH2);
  MatrixToolbox::destroy(SIGaoaH2);
  MatrixToolbox::destroy(SIGqH2);
  MatrixToolbox::destroy(SIGomegaH2);

  // free allocated space
  MatrixToolbox::destroy(aircraftDAT);
  MatrixToolbox::destroy(analysisDAT);
  MatrixToolbox::destroy(propulsionDAT);
  MatrixToolbox::destroy(simulationDAT);
  MatrixToolbox::destroy(LQRDAT);
  ControlToolbox::destroy(sysGv);
  ControlToolbox::destroy(sysGp);
  ControlToolbox::destroy(sysGs);
  MatrixToolbox::destroy(frequency);
  MatrixToolbox::destroy(W);
  MatrixToolbox::destroy(disturbance);
  MatrixToolbox::destroy(stime);
  MatrixToolbox::destroy(Q);
  MatrixToolbox::destroy(null);
  ControlToolbox::destroy(sysK);
  ControlToolbox::destroy(sysCLwz);

 return 0;
}

//=====================================================================================
// end of method 'main'
//=====================================================================================

//=====================================================================================
// method 'ReadData'
//
// description:
//  This method sets key variables used in the analysis that follows.
//
// J R Dowdle
// 14-Aug-2017
//=====================================================================================

// beginning of method 'ReadData'

void ReadData(
  string inFile,                                // input file name
  string dataFolder,                            // data folder name
  MATRIX_DOUBLE *aircraftDAT,                   // array of aircraft data inputs
  MATRIX_DOUBLE *analysisDAT,                   // array of analysis data inputs
  MATRIX_DOUBLE *propulsionDAT,                 // array of propulsion data inputs
  MATRIX_DOUBLE *simulationDAT,                 // array of simulation data inputs
  MATRIX_DOUBLE *LQRDAT)                        // array of LQR data inputs

{
  // executable code

  // set input data file name and output path name
  string TempFileName = "temp.dat";

  // open input stream
  ifstream input(inFile);

  // open temporary output stream
  string tmpFile = dataFolder + "\\" + TempFileName;
  ofstream tmp(tmpFile);

  // strip comments from file and write data to temp.dat 
  while (!input.eof())
  {
    char    line[80] = "";
    input.getline(line, 80);
    if (line[0] != '/')
    {
      tmp << line << endl;
    }
  }
  input.close();
  tmp.close();

  // reopen temp.dat as an input stream
  input.open(tmpFile);

  // get analysisDAT inputs

  // frequency-domain analysis inputs
  int fDecMin, fDecMax, fNpts;                  
  {
    input >> fDecMin >> fDecMax >>fNpts;        
  }
  // time-domain analysis inputs
  long double tStart, tEnd, timeStep;           
  {
    input >> tStart >> tEnd >> timeStep;        
  }

  // load analysisDAT array
  analysisDAT->real[0][0] = fDecMin;
  analysisDAT->real[1][0] = fDecMax;
  analysisDAT->real[2][0] = fNpts;
  analysisDAT->real[3][0] = tStart;
  analysisDAT->real[4][0] = tEnd;
  analysisDAT->real[5][0] = timeStep;

  // get simulationDAT inputs

  // disturbance levels
  long double pulseStart, pulseLength;
  MATRIX_DOUBLE *disturbance = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(disturbance, "disturbance", 0, 7, 1);
  // disturbance time domain characteristics
  {
    input >> pulseStart >> pulseLength;         
  }
  for (int i = 0; i < 7; i++)
  {
    input >> disturbance->real[i][0];           
  }

  // load simulationDAT array
  simulationDAT->real[0][0] = pulseStart;
  simulationDAT->real[1][0] = pulseLength;
  simulationDAT->real[2][0] = disturbance->real[0][0];
  simulationDAT->real[3][0] = disturbance->real[1][0];
  simulationDAT->real[4][0] = disturbance->real[2][0];
  simulationDAT->real[5][0] = disturbance->real[3][0];
  simulationDAT->real[6][0] = disturbance->real[4][0];
  simulationDAT->real[7][0] = disturbance->real[5][0];
  simulationDAT->real[8][0] = disturbance->real[6][0];

  // get aircraftDAT inputs
  // aircraft model parameters
  long double S, CBAR, MASS, IYY, TSTAT;
  long double DTDV, ZE, CDCLS, CLA, CMA;
  long double CMDE, CMQ, CMADOT, CLADOT, RTOD;
  long double GD, CLO_CLEAN, CLO_LAND, CDO_CLEAN, CDO_LAND;
  long double CMO_CLEAN, CMO_LAND, DCDG_CLEAN, DCDG_LAND;
  long double DCMG_CLEAN, DCMG_LAND;
  {
    input >> S >> CBAR >> MASS >> IYY >> TSTAT >> DTDV >> ZE;
    input >> CDCLS >> CLA >> CMA >> CMDE >> CMQ >> CMADOT >> CLADOT >> RTOD >> GD;
    input >> CLO_CLEAN >> CLO_LAND >> CDO_CLEAN >> CDO_LAND >> CMO_CLEAN >> CMO_LAND >> DCDG_CLEAN >> DCDG_LAND;
    input >> DCMG_CLEAN >> DCMG_LAND;
  }

  // load aircraftDAT array
  aircraftDAT->real[0][0] = S;
  aircraftDAT->real[1][0] = CBAR;
  aircraftDAT->real[2][0] = MASS;
  aircraftDAT->real[3][0] = IYY;
  aircraftDAT->real[4][0] = TSTAT;
  aircraftDAT->real[5][0] = DTDV;
  aircraftDAT->real[6][0] = ZE;
  aircraftDAT->real[7][0] = CDCLS;
  aircraftDAT->real[8][0] = CLA;
  aircraftDAT->real[9][0] = CMA;
  aircraftDAT->real[10][0] = CMDE;
  aircraftDAT->real[11][0] = CMQ;
  aircraftDAT->real[12][0] = CMADOT;
  aircraftDAT->real[13][0] = CLADOT;
  aircraftDAT->real[14][0] = RTOD;
  aircraftDAT->real[15][0] = GD;
  aircraftDAT->real[16][0] = CLO_CLEAN;
  aircraftDAT->real[17][0] = CLO_LAND;
  aircraftDAT->real[18][0] = CDO_CLEAN;
  aircraftDAT->real[19][0] = CDO_LAND;
  aircraftDAT->real[20][0] = CMO_CLEAN;
  aircraftDAT->real[21][0] = CMO_LAND;
  aircraftDAT->real[22][0] = DCDG_CLEAN;
  aircraftDAT->real[23][0] = DCDG_LAND;
  aircraftDAT->real[24][0] = DCMG_CLEAN;
  aircraftDAT->real[25][0] = DCMG_LAND;

  // get propulsionDAT inputs
  // aircraft model parameters
  long double eta, powerDensity, fPropeller, JPropeller, omega0;
  {
    input >> eta >> powerDensity >> fPropeller >> JPropeller >> omega0;
  }

  // load propulsionDAT array
  propulsionDAT->real[0][0] = eta;
  propulsionDAT->real[1][0] = powerDensity;
  propulsionDAT->real[2][0] = fPropeller;
  propulsionDAT->real[3][0] = JPropeller;
  propulsionDAT->real[4][0] = omega0;

  // get LQRDat inputs
  // LQR design parameters
  long double Q0WGT0, Q1WGT0, Q2WGT0, Q3WGT0, Q4WGT0, Q5WGT0;
  long double ThrustMaxBW, ElevatorMaxBW, PropulsionMaxBW;
  {
    input >> Q0WGT0 >> Q1WGT0 >> Q2WGT0 >> Q3WGT0 >> Q4WGT0 >> Q5WGT0;
    input >> ThrustMaxBW >> ElevatorMaxBW >> PropulsionMaxBW;
  }

  // load propulsionDAT array
  LQRDAT->real[0][0] = Q0WGT0;
  LQRDAT->real[1][0] = Q1WGT0;
  LQRDAT->real[2][0] = Q2WGT0;
  LQRDAT->real[3][0] = Q3WGT0;
  LQRDAT->real[4][0] = Q4WGT0;
  LQRDAT->real[5][0] = Q5WGT0;
  LQRDAT->real[6][0] = ThrustMaxBW;
  LQRDAT->real[7][0] = ElevatorMaxBW;
  LQRDAT->real[8][0] = PropulsionMaxBW;

  // close files
  input.close();

  // free allocated space
  MatrixToolbox::destroy(disturbance);
}

//=====================================================================================
// end of method 'ReadData'
//=====================================================================================

//=====================================================================================
// end of module 'DesignTrades'
//=====================================================================================