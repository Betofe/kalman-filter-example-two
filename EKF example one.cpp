//============================================================================
// Name        : Kalman.cpp
// Author      : Imami Betofe
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


// initial state
MatrixXd x0(3,1);

MatrixXd Fj(3,3);

// initial uncertainty
MatrixXd P0(3,3);

// process noise
MatrixXd Q(3,3);

// measurement matrix
MatrixXd H(2,3);

// sensor noise
MatrixXd R(2,2);

// measurement received
MatrixXd z1(2,1);


int main()
{    
     //known inputs
     double v=3;
     double w=0.2;
     double deltaT=1;
     // initial state
     x0 << 2,
          1,
          0.5;

     //current estimate 
     double x=2;
     double y=1;
     double thetha=0.5;
     
     double xnew = x+v*cos(thetha)*deltaT;
     double ynew = y+v*sin(thetha)*deltaT;
     double thethaNew = thetha + w*deltaT;

     MatrixXd x1(3,1);
     x1<<xnew,
         ynew,
         thethaNew;

    // state transition matrix
    Fj << 1, 0, -v*sin(thetha)*deltaT,
         0, 1, v*cos(thetha)*deltaT,
         0,0,1;

    // initial uncertainty
    P0 << 1, 0, 0,
          0, 1, 0,
          0, 0, 0.05;

    // process noise
    Q << 0.05, 0, 0,
         0, 0.05, 0,
         0, 0, 0.01;

    // measurement matrix
    H << 1, 0, 0,
         0, 1, 0;

     //measurement noise
     R <<0.36, 0,
         0, 0.36;
     
     //measurement received
     z1 <<4.7,
          2.9;
     

     //predict covariance
     
     MatrixXd p1 = Fj*P0*Fj.transpose()+Q;

     //========================================================
     // UPDATE
     //========================================================
     MatrixXd yvec = z1-H*x1;
     
     cout << "ynew is " << y << endl;

     // innovation covariance

     MatrixXd S=H*p1*H.transpose()+R;
     
     //========================================================
     // KALMAN GAIN
     //========================================================

     MatrixXd K = p1*H.transpose()*S.inverse();

     cout << "Kalman gain is:\n"
             << K << endl;

     //========================================================
     // STATE UPDATE
     //========================================================

     MatrixXd X = x1 + K*yvec;

     cout << "X updated is:\n"
             << X << endl;
     
     //========================================================
     // COVARIANCE UPDATE
     //========================================================

     MatrixXd I = MatrixXd::Identity(3,3);

     MatrixXd P = (I-K*H)*p1;

     cout << "P updated is:\n"
             << P << endl;


     cout << "----------------------------------" << endl;

    return 0;
}