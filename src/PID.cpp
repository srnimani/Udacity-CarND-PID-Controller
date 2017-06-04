#include "PID.h"
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    this->p_error = 0.0 ;
    this->i_error = 0.0 ;
    this->d_error = 0.0 ;
}

void PID::UpdateError(double cte)
{
    d_error = cte - p_error; // p_error is previous cte
    p_error = cte;
    i_error += cte;
    //cout << "p_error = \t" << p_error <<"\ti_error = \t" << i_error << "\td_error = \t" << d_error << endl;
}

double PID::TotalError()
{
    double total_error = Kp * p_error + Kd * d_error + Ki * i_error ;
    //cout << "Total Error : " << total_error << endl;
    return total_error;
}
