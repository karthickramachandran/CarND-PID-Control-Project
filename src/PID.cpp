#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    past_cte = 0;
}

void PID::UpdateError(double cte)
{
    double diff_cte = cte - past_cte;
    p_error = cte;    
    i_error += cte;
    d_error = diff_cte;
    past_cte = cte;
}

double PID::TotalError()
{
    // calculating  -tau_p * CTE - tau_i * int_CTE - tau_d * diff_CTE
    double total_error;
    total_error =   -Kp * p_error
                    -Ki * i_error
                    -Kd * d_error ;

    return total_error;
}

