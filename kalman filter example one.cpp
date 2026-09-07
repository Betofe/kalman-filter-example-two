//============================================================================
// Name        : Kalman.cpp
// Author      : Imami Betofe
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <iostream>
using namespace std;

int x0 = 0; //Initial estimate
int p0= 1; //Initial uncertainty
double q=0.0001; //Process noise
double r= 0.01; //Measurement noise
double a=1.25; //true voltage
double z[10] = {1.30, 1.18, 1.27, 1.22, 1.31, 1.19, 1.24, 1.28, 1.21, 1.26}; //measurement sequence

int main() { 
    double x;
    double p;
    double k;

    x=x0;
    p=p0;
    
    for(int i=0; i<10; i++){
    // predict
    x=x;
    p=p+q;
    
    cout << "x"<<i+1 <<" predicted is " << x <<endl; 
    cout << "p"<<i+1 <<" predicted is " << p <<endl; 

    // update
    k=p/(p+r);
    x=x+k*(z[i]-x);
    p=(1-k)*p;
    
    cout << "k"<<i+1 <<" updated is " << k <<endl;
    cout <<"x"<<i+1 <<" updated is " << x <<endl; 
    cout << "p"<<i+1 <<" updated is " << p <<endl; 
    }
    return 0;
}
