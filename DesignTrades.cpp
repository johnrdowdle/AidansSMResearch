//=====================================================================================
// method 'main'
//
// description:
//  This method runs the full set of scripts that perform design trades for the 
//  turboelectric aircraft.
//
// J R Dowdle
// 0.0.0.0
// 14-Aug-2017
//=====================================================================================

// include headers, macros, defined constants, procedure prototypes
#include "stdafx.h"
#include "DesignTrades.h"
#include "../../MatrixSolutions/ControlSystemLibrary/ControlSystemLibrary.h"
#include "../../MatrixSolutions/MatrixLibrary/MatrixLibrary.h"

// included namespaces
using namespace std;
using namespace MatrixLibrary;
using namespace ControlSystemLibrary;

// global variable declarations
int trim;

// beginning of method 'main'

int main(int argc, char *argv[], char *envp[])
{

  // executable code

  // allocate space
  MATRIX_DOUBLE *aircraftDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(aircraftDAT, "aircraftDAT", 0, 26, 1);
  MATRIX_DOUBLE *analysisDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(analysisDAT, "analysisDAT", 0, 6, 1);
  MATRIX_DOUBLE *propulsionDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(propulsionDAT, "propulsionDAT", 0, 5, 1);
  MATRIX_DOUBLE *simulationDAT = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(simulationDAT, "simulationDAT", 0, 9, 1);
  MATRIX_DOUBLE *W = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(W, "W", 0, 7, 7);
  SYSTEM_MODEL *sysP = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));

  // set file names from input arguments
  string inFile, dataFolder;
  if (argc >= 3)
  {
    inFile = argv[1];                      // input file name
    dataFolder = argv[2];                  // output data folder name
  }
  else
  {
    inFile = "U:\\John\\Documents\\Software Projects\\AidansSMResearch\\CramerRaoBound\\CRB\\input.dat";
    dataFolder = "U:\\John\\Documents\\Software Projects\\AidansSMResearch\\CramerRaoBound\\CRB";
  }

  // set desired trim condition (1, 2 or 3)
  do
  {
    cout << "Enter trim condition (1, 2, 3): " << endl;
    cin >> trim;
  } while ((trim != 1) && (trim != 2) && (trim != 3));

  // set ouotput folder based upon trim condition
  // TODO: check that folders exist and opening of file was successful
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

  // open diary and results files
  ofstream diary(diaryFile);
  ofstream results(resultsFile);
  ofstream freqrsp(freqrspFile);

  // initialize data 
  // print header to screen
  cout << endl << endl << "Executing:  ReadData" << endl;
  diary << endl << endl << "Executing:  ReadData" << endl;
  time_t rawtime;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;

  ReadData(inFile, dataFolder, aircraftDAT, analysisDAT, propulsionDAT, simulationDAT);
  
  // set up the model
  SYSTEM_MODEL *sysGv = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysGv, "sysGv", 4, 6, 2, 4, 3);
  SYSTEM_MODEL *sysGp = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysGp, "sysGp", 1, 1, 1, 2, 1);
  SYSTEM_MODEL *sysGs = (SYSTEM_MODEL *)malloc(sizeof(SYSTEM_MODEL));
  ControlToolbox::create(sysGs, "sysGs", 5, 7, 3, 6, 4);

  // print header to screen
  cout << endl << endl << "Executing:  SystemModel" << endl;
  diary << endl << endl << "Executing:  SystemModel" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  SystemModel(aircraftDAT, propulsionDAT, sysP, trim, sysGv, sysGp, sysGs);
  ControlToolbox::display(sysGv);
  ControlToolbox::display(sysGv, diary);
  ControlToolbox::display(sysGp);
  ControlToolbox::display(sysGp, diary);
  ControlToolbox::display(sysGs);
  ControlToolbox::display(sysGs, diary);

  // compute open-loop frequency responses of integrated model

  // print header to screen
  cout << endl << endl << "Executing:  OpenLoopFrequencyResponse" << endl;
  diary << endl << endl << "Executing:  OpenLoopFrequencyResponse" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  // set frequency vector 
  int fDecMin = analysisDAT->real[0][0];
  int fDecMax = analysisDAT->real[1][0];
  int fNpts = analysisDAT->real[2][0];
  MATRIX_DOUBLE *frequency = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(frequency, "frequency", 0, fNpts, 1);
  MatrixToolbox::logspace(fDecMin, fDecMax, frequency);
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

  cout << "computing ..." << endl << endl;

  ControlToolbox::freqrsp(frequency, sysGs, svGsmin, svGsmax, svGs11min, svGs11max, svGs12min, svGs12max,
    svGs21min, svGs21max, svGs22min, svGs22max);

  diary << "// frequency [r/s]     Gs-minimum         Gs-maximum         Gs11-minimum       Gs11-maximum       Gs12-minimum       Gs12-maximum       Gs21-minimum       Gs21-maximum       Gs22-minimum       Gs22-maximum" << endl;
  freqrsp << "// frequency [r/s]     Gs-minimum         Gs-maximum         Gs11-minimum       Gs11-maximum       Gs12-minimum       Gs12-maximum       Gs21-minimum       Gs21-maximum       Gs22-minimum       Gs22-maximum" << endl;
  for (int i = 0; i < fNpts; i++)
  {
    diary << "   " << frequency->real[i][0] << "     " << svGsmin->real[i][0] << "     " << svGsmax->real[i][0] << "     " <<
      svGs11min->real[i][0] << "     " << svGs11max->real[i][0] << "     " <<
      svGs12min->real[i][0] << "     " << svGs12max->real[i][0] << "     " <<
      svGs21min->real[i][0] << "     " << svGs21max->real[i][0] << "     " <<
      svGs22min->real[i][0] << "     " << svGs22max->real[i][0] << endl;
    freqrsp << "   " << frequency->real[i][0] << "     " << svGsmin->real[i][0] << "     " << svGsmax->real[i][0] << "     " <<
      svGs11min->real[i][0] << "     " << svGs11max->real[i][0] << "     " <<
      svGs12min->real[i][0] << "     " << svGs12max->real[i][0] << "     " <<
      svGs21min->real[i][0] << "     " << svGs21max->real[i][0] << "     " <<
      svGs22min->real[i][0] << "     " << svGs22max->real[i][0] << endl;
  }
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
   
  // compute the disturbance intensities	
  // print header to screen
  cout << endl << endl << "Executing:  DisturbanceIntensities" << endl;
  diary << endl << endl << "Executing:  DisturbanceIntensities" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  DisturbanceIntensities(sysGs, sysGs->xref->real[0][0], sysGs->xref->real[1][0], sysGs->xref->real[4][0], W);
  MatrixToolbox::display(W);
  MatrixToolbox::display(W, diary);

  //  run the parametrics	on disturbances
  // set time vector
  long double tStart = analysisDAT->real[3][0];
  long double tEnd = analysisDAT->real[4][0];
  long double timeStep = analysisDAT->real[5][0];
  long double tNpts = (tEnd - tStart)/timeStep + 1;
  MATRIX_DOUBLE *stime = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(stime, "stime", 0, tNpts, 1);
  MatrixToolbox::linspace(0.0, tEnd, stime);

  StaticCRB();

  //  compute time-domain responses	
  // print header to screen
  cout << endl << endl << "Executing:  DynamicAnalysis" << endl;
  diary << endl << endl << "Executing:  DynamicAnalysis" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  DynamicAnalysis();

  //	run parametrics on LQR weights	
  // print header to screen
  cout << endl << endl << "Executing:  LQRWeights" << endl;
  diary << endl << endl << "Executing:  LQRWeights" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  LQRWeights();

  //	design the LQR controller	
  // print header to screen
  cout << endl << endl << "Executing:  LQRDesign" << endl;
  diary << endl << endl << "Executing:  LQRDesign" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  LQRDesign();

  //	design the H2 controller`	
  // print header to screen
  cout << endl << endl << "Executing:  ControlDesign" << endl;
  diary << endl << endl << "Executing:  ControlDesign" << endl;
  time(&rawtime);
  ctime(&rawtime);
  cout << endl <<  ctime(&rawtime) << endl;
  diary << endl <<  ctime(&rawtime) << endl;

  ControlDesign();

  // free allocated space

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

void ReadData(string inFile, 
  string dataFolder, 
  MATRIX_DOUBLE *aircraftDAT,
  MATRIX_DOUBLE *analysisDAT, 
  MATRIX_DOUBLE *propulsionDAT, 
  MATRIX_DOUBLE *simulationDAT)
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
  int fDecMin, fDecMax, fNpts;                      // min / max decades and number of points for frequency vector
  {
    input >> fDecMin >> fDecMax >>fNpts;            // read data
  }
  // time-domain analysis inputs
  long double tStart, tEnd, timeStep;               // simulation start time, end time and time-step
  {
    input >> tStart >> tEnd >> timeStep;            // read data
  }

  // load analysisDAT array
  analysisDAT->real[0][0] = fDecMin;
  analysisDAT->real[1][0] = fDecMax;
  analysisDAT->real[2][0] = fNpts;
  analysisDAT->real[3][0] = tStart;
  analysisDAT->real[4][0] = tEnd;
  analysisDAT->real[5][0] = timeStep;

  // get simulationDAT inputs

  // disturbance time domain characteristics
  long double pulseStart, pulseLength;                // pulse start time, duration of pulse
  {
    input >> pulseStart >> pulseLength;               // read data
  }

  // disturbance levels
  MATRIX_DOUBLE *disturbance = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(disturbance, "disturbance", 0, 7, 1);
  for (int i = 0; i < 7; i++)
  {
    input >> disturbance->real[i][0];                       // read data
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
  
  // close files
  input.close();
}

//=====================================================================================
// end of method 'ReadData'
//=====================================================================================

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
//  ztilda = [vtilda; aoatilda; dttilda; detilda; omegatilda; PGentilda/PGen0] 
//
//
//  there are three trim conditions of interest:
//  Trim condition 1
//  Ts = 40000;				                          // thrust, lbf
//  v0 = 0.4*995;         	                    // fps
//  h0 = 30000;           	                    // feet
//  gamma = 0;            	                    // degrees
// 
//  Trim condition 2	
//	Ts = 40000;   			                        // thrust, lbf
//  v0 = 0.5*995;                               // fps
//  h0 = 30000;                                 // feet
//  gamma = 0;                                  // degrees
//
//  Trim condition 3	
//	Ts = 40000;    			                        // thrust, lbf
//  v0 = 0.6*995;                               // fps
//  h0 = 30000;                                 // feet
//  gamma = 0;                                  // degrees
//
// J R Dowdle
// 08-Aug-2017
//=====================================================================================

// beginning of method 'SystemModel'

void SystemModel(MATRIX_DOUBLE *aircraftDAT, 
  MATRIX_DOUBLE *propulsionDAT, 
  SYSTEM_MODEL *sysP, 
  int trim,
  SYSTEM_MODEL *sysGv,
  SYSTEM_MODEL *sysGp,
  SYSTEM_MODEL *sysGs)
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
    T0 = 40000;                           // thrust, lbf
    v0 = 0.4*995;                         // fps
    h0 = 30000;                           // feet
    gamma = 0;                            // degrees
  }
  else if (trim == 2)
  {
    aircraftDAT->real[4][0] = 40000;      // thrust, lbf
    T0 = 40000;                           // thrust, lbf
    v0 = 0.5*995;                         // fps
    h0 = 30000;                           // feet
    gamma = 0;                            // degrees
  }
  else if (trim == 3)
  {
    aircraftDAT->real[4][0] = 40000;      // thrust, lbf
    T0 = 40000;                           // thrust, lbf
    v0 = 0.6*995;                         // fps
    h0 = 30000;                           // feet
    gamma = 0;                            // degrees
  }

  // compute the trim conditions    
  trimComputation(v0, h0, gamma, xv0, uv0);

  // set up the vehicle state-space model
  transp(aircraftDAT, propulsionDAT, 0.0, xv0, uv0, h0, xvdot, Av, B1v, B2v, C1v, D11v, D12v, C2v, D21v, D22v);

  // set the variables for trim condition
  long double Ts, theta0, aoa0, q0, dt0, de0;
  Ts = T0;                                // thrust (N)
  v0 = xv0->real[0][0];                   // trim velocity (fps)
  aoa0 = xv0->real[1][0];                 // angle-of-attack (rad)
  theta0 = xv0->real[2][0];               // pitch angle (rad)
  q0 = xv0->real[3][0];                   // pitch rate (rps)
  dt0 = uv0->real[0][0];                  // throttle (unitless)
  de0 = uv0->real[1][0];                  // elevator (deg)

  // set the nominal measurement
  yv0->real[0][0] = v0;                   // trim velocity (fps)
  yv0->real[1][0] = aoa0;                 // angle-of-attack (rad)
  yv0->real[2][0] = q0;                   // pitch rate (rps)

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

void trimComputation(long double v, long double h, long double gamma, MATRIX_DOUBLE *xv, MATRIX_DOUBLE *uv)
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
  MATRIX_DOUBLE *s,
  MATRIX_DOUBLE *x,
  long double gamma,
  long double h,
  MATRIX_DOUBLE *aircraftDAT,
  MATRIX_DOUBLE *propulsionDAT,
  MATRIX_DOUBLE *Av,
  MATRIX_DOUBLE *B1v,
  MATRIX_DOUBLE *B2v,
  MATRIX_DOUBLE *C1v,
  MATRIX_DOUBLE *D11v,
  MATRIX_DOUBLE *D12v,
  MATRIX_DOUBLE *C2v,
  MATRIX_DOUBLE *D21v,
  MATRIX_DOUBLE *D22v)
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
  u->real[0][0] = s->real[0][0];              // throttle
  u->real[1][0] = s->real[1][0];              // elevator
  y->real[0][0] = x->real[0][0];              // speed
  y->real[1][0] = s->real[2][0];              // angle-of-attack
  y->real[2][0] = y->real[1][0] + gamma;      // pitch angle
  y->real[3][0] = x->real[3][0];              // pitch rate

  // evaluate nonlinear simulation
  transp(aircraftDAT, propulsionDAT, 0.0, y, u, h, yd, Av, B1v, B2v, C1v, D11v, D12v, C2v, D21v, D22v);

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
// method 'transp'
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

// beginning of method 'transp'

void transp(MATRIX_DOUBLE *aircraftDAT, 
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
  MATRIX_DOUBLE *D22v)
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
  THTL = u->real[0][0];                       // throttle
  ELEV = u->real[1][0];                       // elevator
  v = x->real[0][0];                          // wind speed in fps 
  ALPHA = 90/asin(1.0)*x->real[1][0];         // angle-of-attack (deg)
  THETA = x->real[2][0];                      // pitch angle 
  Q = x->real[3][0];                          // pitch rate
  H = h;                                      // altitude 

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
  ADC(v, h, &RHO, &QBAR, &MACH);
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
  xd->real[1][0] =(-THR*SALP - QS*CL + MASS*(v*Q + GD*CGAM))/Momentum;   //  alphadot
  xd->real[2][0] = Q;                                                     // thetadot
  long double D = 0.5*CBAR*(CMQ*Q + CMADOT*xd->real[1][0])/v;            // pitch damping 
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
// end of method 'transp'
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
  MATRIX_DOUBLE *H2p)
{
  // executable code

  // set constants
  long double eta = propulsionDAT->real[0][0];             //  subsystem efficiency
  long double PDen = propulsionDAT->real[1][0];            //  power density
  long double fprop = propulsionDAT->real[2][0];           //  propulsion damping factor
  long double Jprop = propulsionDAT->real[3][0];           //  propulsion inertia
  long double omega0 = propulsionDAT->real[4][0];          //  nominal shaft rotational rate

  // compute generator nominal power at trim
  *PGen0 = T0*v0/eta;

  // compute generator nominal weight at trim
  long double mGenkg = (*PGen0)/PDen;            // generator mass (kg)
  *mGen0 = mGenkg*2.205/32.2;      // generator mass (slugs)

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
  D12p->real[1][0] = 1/(*PGen0);

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
// method 'ADC'
//
// description:
//  This method computes Mach number and aerodynamic pressure for input speed and 
//  altitude.
//
// J R Dowdle
// 14-Aug-2017
//=====================================================================================

// beginning of method 'ADC'

void ADC(
  long double v, 
  long double h, 
  long double *rho, 
  long double *qbar, 
  long double *mach)
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
// end of method 'ADC'
//=====================================================================================

//=====================================================================================
// method 'DisturbanceIntensities'
//
// description:
//  This method computes the process and measurement noises that result in 10% open-
//  variations in the key states, and 1% estimation error when a Kalman filter is 
//  implemented.
//
// J R Dowdle
// 24-Aug-2017
//=====================================================================================

// beginning of method 'DisturbanceIntensities'

void DisturbanceIntensities(
  SYSTEM_MODEL *sysP, 
  long double v0, 
  long double aoa0, 
  long double omega0, 
  MATRIX_DOUBLE *W)
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
  MATRIX_DOUBLE *sqW = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(sqW, "sqW", 0, m1, m1);
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
  MATRIX_DOUBLE *K = (MATRIX_DOUBLE *)malloc(sizeof(MATRIX_DOUBLE));
  MatrixToolbox::create(K, "K", 0, n, r2);

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
  cout << "W[0][0] = " << W->real[0][0] << ";        W[1][1] = " << W->real[1][1] << endl;
  cout << "W[2][2] = " << W->real[2][2] << ";        W[3][3] = " << W->real[3][3] << endl;
  cout << "W[4][4] = " << W->real[4][4] << ";        W[5][5] = " << W->real[5][5] << endl;
  cout << "W[6][6] = " << W->real[6][6] << endl << endl;
  long double performancev = sqrt(X->real[0][0])/v0;
  long double performanceaoa = sqrt(X->real[1][1])/aoa0;
  long double performanceomega = sqrt(X->real[3][3])/omega0;
  cout << "SIGV/V0 = " << performancev << ";        SIGaoa/aoa0 = " << performanceaoa << endl;
  cout << "SIGomega/omega0 = " << performanceomega << endl << endl;
  
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
    W->real[3][3] = INFINITY;                        // v sensor noise
    W->real[4][4] = INFINITY;                        // aoa sensor noise
    W->real[5][5] = INFINITY;                        // q sensor noise
    W->real[6][6] = INFINITY;                        // omega sensor noise

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
      for (int i = 0; i < m1; i++)
        sqW->real[i][i] = sqrt(W->real[i][i]);
      MatrixToolbox::multiply(sysPb->B1, sqW, B1);
      MatrixToolbox::multiply(sysPb->D21, sqW, D21);
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
  W->real[0][0] = W00;                        // v disturbance
  W->real[1][1] = W11;                        // aoa disturbance
  W->real[2][2] = W22;                        // q disturbance
  W->real[3][3] = W33;                        // v sensor noise
  W->real[4][4] = W44;                        // aoa sensor noise
  W->real[5][5] = W55;                        // q sensor noise
  W->real[6][6] = W66;                        // omega sensor noise

  // compute the performance with all disturbances and sensor noises active
  for (int i = 0; i < m1; i++)
    sqW->real[i][i] = sqrt(W->real[i][i]);
  MatrixToolbox::multiply(sysPb->B1, sqW, B1);
  MatrixToolbox::multiply(sysPb->D21, sqW, D21);
  MatrixToolbox::equate(B1, sysG->B1);
  MatrixToolbox::equate(D21, sysG->D21);
  ControlToolbox::kf(sysG, K, Y);
  MatrixToolbox::transpose(sysG->C2, C2t);
  MatrixToolbox::multiply(sysG->C2, Y, C2Y);
  MatrixToolbox::multiply(C2Y, C2t, X);

  // display the results
  cout << "closed-loop disturbance response:" << endl;
  cout << "W[0][0] = " << W->real[0][0] << ";        W[1][1] = " << W->real[1][1] << endl;
  cout << "W[2][2] = " << W->real[2][2] << ";        W[3][3] = " << W->real[3][3] << endl;
  cout << "W[4][4] = " << W->real[4][4] << ";        W[5][5] = " << W->real[5][5] << endl;
  cout << "W[6][6] = " << W->real[6][6] << endl << endl;
  performancev = sqrt(X->real[0][0])/v0;
  performanceaoa = sqrt(X->real[1][1])/aoa0;
  performanceomega = sqrt(X->real[3][3])/omega0;
  cout << "SIGV/V0 = " << performancev << ";        SIGaoa/aoa0 = " << performanceaoa << endl;
  cout << "SIGomega/omega0 = " << performanceomega << endl << endl;

  // TODO:  write code to perform sensitivity analyses around the above nominal W

  // free allocated space
  MatrixToolbox::destroy(B1W);
  MatrixToolbox::destroy(B1t);
  MatrixToolbox::destroy(B1WB1t);
  MatrixToolbox::destroy(sqW);
  MatrixToolbox::destroy(B1);
  MatrixToolbox::destroy(D21);
  MatrixToolbox::destroy(Y);
  MatrixToolbox::destroy(K);
  MatrixToolbox::destroy(C2t);
  MatrixToolbox::destroy(C2Y);
  MatrixToolbox::destroy(X);
  ControlToolbox::destroy(sysPb);
  ControlToolbox::destroy(sysG);
}

//=====================================================================================
// end of method 'DisturbanceIntensities'
//=====================================================================================



void StaticCRB()
{

}

void DynamicAnalysis()
{

}

void LQRWeights()
{

}

void LQRDesign()
{

}

void ControlDesign()
{

}
