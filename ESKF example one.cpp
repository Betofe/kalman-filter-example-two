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


// nominal orientation
MatrixXd qn(4,1);

//initial error state
MatrixXd thetha(3,1);

// initial error covariance
MatrixXd P0(3,3);

// process noise
MatrixXd Q(3,3);

// measurement noise
MatrixXd R(3,3);

// measurement received
MatrixXd z1(3,1);


int main()
{    

     // Nominal orientation
     qn << 0.9962,
          0.0872,
          0,
          0;

     //Initial error state
     thetha <<0,
              0,
              0;
     //
    MatrixXd Fthetha = MatrixXd::Identity(3,3);
    
    //Measurement matrix
    MatrixXd H = MatrixXd::Identity(3,3);

    // initial error covariance
    P0 << 0.01, 0, 0,
          0, 0.01, 0,
          0, 0, 0.01;

    // process noise
    Q << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001;

     //measurement noise
     R <<0.02, 0,0,
         0, 0.02,0,
         0, 0, 0.02;
     
     //measurement received
     z1 <<0.02,
          0.01,
          -0.015;
     

     //Error state predict

     MatrixXd thetha1 = Fthetha*thetha;

     MatrixXd P1 = Fthetha*P0*Fthetha.transpose()+Q;

     //========================================================
     // kalman gain
     //========================================================
     MatrixXd K=P1*H.transpose()*(H*P1*H.transpose()+R).inverse();
     cout << "K:\n" << K << endl;

     //========================================================
     // error state update
     //========================================================
     MatrixXd innovation =z1-H*thetha1;

     MatrixXd thetha2=thetha1+ K*innovation;

     MatrixXd I = MatrixXd::Identity(3,3);

     MatrixXd P2 = (I-K*H)*P1;
     cout << "thetha2 (corrected error state):\n" << thetha2 << endl;
     cout << "P2:\n" << P2 << endl;

     //========================================================
     // injection
     //========================================================
     MatrixXd qthetha(4,1);
              qthetha<<1,
                       thetha2(0,0)/2,
                       thetha2(1,0)/2,
                       thetha2(2,0)/2;

     // quaternion multiplication: q_true = qn (x) qthetha
     double w1 = qn(0,0), x1 = qn(1,0), y1 = qn(2,0), z1_q = qn(3,0);
     double w2 = qthetha(0,0), x2 = qthetha(1,0), y2 = qthetha(2,0), z2 = qthetha(3,0);

     double w = w1*w2 - x1*x2 - y1*y2 - z1_q*z2;
     double x = w1*x2 + x1*w2 + y1*z2 - z1_q*y2;
     double y = w1*y2 - x1*z2 + y1*w2 + z1_q*x2;
     double z = w1*z2 + x1*y2 - y1*x2 + z1_q*w2;

     MatrixXd q_raw(4,1);
     q_raw << w, x, y, z;

     // normalize
     double norm = sqrt(w*w + x*x + y*y + z*z);

     MatrixXd qthethafinal(4,1);
     qthethafinal << w/norm,
                      x/norm,
                      y/norm,
                      z/norm;
     cout << "qthetha (correction quaternion):\n" << qthetha.transpose() << endl;
     cout << "q_raw (before normalize):\n" << q_raw.transpose() << endl;
     cout << "norm: " << norm << endl;
     cout << "qthethafinal (injected + normalized):\n" << qthethafinal.transpose() << endl;
     
     //========================================================
     // reset
     //========================================================

     thetha<< 0,
              0,
              0;

     cout << "thetha after reset:\n" << thetha.transpose() << endl;
     
     cout << "----------------------------------" << endl;

    return 0;
}