#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    dp = {0.1 * Kp, 0.1 * Ki, 0.1 * Kd};
    p = {Kp,Ki,Kd};

    PID::p_error = 0.0;
    PID::i_error = 0.0;
    PID::d_error = 0.0;

    curr_step = 1;

    n_steps = 100;

    index = 2;

    TWIDDLE = false;

    err = 0.0;

    /* Initialize the best error with a big number*/
    best_err = 100000000.0;

    tried_add = false;
    tried_subtract = false;

}

void PID::UpdateError(double cte) {

    if (curr_step == 1)
    {
        p_error = cte;
    }

    d_error = cte - p_error;

    p_error = cte;

    i_error += cte;

    /* Update the error when the step has passed the n*/
    if (curr_step % (n_steps+2*n_steps) > n_steps)
    {
        err += pow(cte,2);
    }

    /*twiddle after calculating the average error*/
    if (TWIDDLE == true && ((curr_step % (n_steps+2*n_steps)) == 0))
    {

        cout << "curr_step: " << curr_step << endl;
        cout << "curr_error: " << err << endl;
        cout << "best_error: " << best_err << endl;

        if (err < best_err)
        {
            cout<< "******************** error decreased *********************" << endl;
            best_err = err;

            if (curr_step != (n_steps+2*n_steps))
            {
                dp[index] *= 1.1;

            }

            //move to next parameter
            index = (index+1)%3;

            tried_add = tried_subtract = false;

        }

        if(!tried_add && !tried_subtract)
        {
            p[index] += dp[index];

            tried_add = true;

        }
        else if(tried_add && !tried_subtract)
        {
            p[index] -= 2*dp[index];

            tried_subtract = true;
        }
        else
        {

            p[index] += dp[index];

            dp[index] *=0.9;

            index = (index + 1) % 3;
            tried_add = tried_subtract = false;

        }

        err = 0.0;

        //Update the gains after tuning
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];

        cout << "new gains" << endl;
        cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << endl;
        cout<< endl;
    }

    curr_step++;

}

double PID::TotalError() {

    double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;

    return total_error;
}

