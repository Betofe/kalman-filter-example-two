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
MatrixXd x0(2,1);

MatrixXd F(2,2);

// initial uncertainty
MatrixXd P0(2,2);

// process noise
MatrixXd Q(2,2);

// measurement matrix
MatrixXd H(1,2);

// sensor measurement position
double R = 4;

// measurement sequence
double z[10] = {
    6.0, 9.5, 16.0, 19.0, 26.5,
    29.0, 36.5, 39.0, 46.0, 49.5
};


int main()
{
    // initial state
    x0 << 0,
           5;

    // state transition matrix
    F << 1, 1,
         0, 1;

    // initial uncertainty
    P0 << 100, 0,
          0, 25;

    // process noise
    Q << 0.1, 0.1,
         0.1, 0.2;

    // measurement matrix
    H << 1, 0;


    // current state and covariance
    MatrixXd x = x0;
    MatrixXd P = P0;


    for(int i = 0; i < 10; i++)
    {
        //========================================================
        // PREDICT
        //========================================================

        x = F * x;

        P = F * P * F.transpose() + Q;

        cout << "x" << i+1 << " predicted is:\n"
             << x << endl;

        cout << "P" << i+1 << " predicted is:\n"
             << P << endl;


        //========================================================
        // UPDATE
        //========================================================

        // innovation / residual
        //`(0,0)` pulls the single number out of the 1×1 matrix that `H*x` produces, since Eigen keeps it as a matrix object rather than a plain `double` until you explicitly index into it.
        
        double y = z[i] - (H * x)(0,0);

        // innovation covariance
        double S = (H * P * H.transpose())(0,0) + R;

        cout << "y" << i+1 << " is " << y << endl;


        //========================================================
        // KALMAN GAIN
        //========================================================

        MatrixXd K = P * H.transpose() / S;

        cout << "K" << i+1 << " is:\n"
             << K << endl;


        //========================================================
        // STATE UPDATE
        //========================================================

        x = x + K * y;

        cout << "x" << i+1 << " updated is:\n"
             << x << endl;


        //========================================================
        // COVARIANCE UPDATE
        //========================================================

        MatrixXd I = MatrixXd::Identity(2,2);

        P = (I - K * H) * P;

        cout << "P" << i+1 << " updated is:\n"
             << P << endl;


        cout << "----------------------------------" << endl;
    }


    return 0;
}