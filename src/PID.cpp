#include "PID.h"
#include <algorithm>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  d_error = 0;
  p_error = 0;
  i_error = 0;

  threshold = 0.0000001;
  best_error = 2.0;
  err = 0.0;

  variable_counter = 0;
  rounds = 2000;
  i = 0;

  caliberated = false;

  P.push_back(Kp);
  P.push_back(Ki);
  P.push_back(Kd);

  best_P.push_back({Kp,Ki, Kd});
  best_err.push_back(best_error);

  dp.push_back(0.001);
  dp.push_back(0.000001);
  dp.push_back(0.01);

  best_dp.push_back({0.001, 0.000001, 0.01});

  INIT = true;
  first_check = false;
  updated = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
   double t_err = -(P[0] * p_error + P[2] * d_error + P[1] * i_error);
   std::cout<<"Total Error : "<<t_err<<" ";
  return t_err < 1.0 ? (t_err > -1.0 ? t_err : -1.0 ) : 1.0 ;  // TODO: Add your total error calc here!
}

void PID::Twiddle(double cte) {

    if (dp[0] + dp[1] + dp[2] > threshold && i < rounds) {

        err += (cte * cte);
        err = err / rounds;
        //err = cte;

        if (updated == true){
            variable_counter = (variable_counter + 1) % 3;
            updated = false;
            first_check = false;
            INIT =true;
        }


        if (INIT == true){
            P[variable_counter] += dp[variable_counter];
            INIT = false;
            return;
        }
        /**

        if(first_check == false){

            if ( err < best_err ){
                best_err = err;
                dp[variable_counter] *= 1.1;
                updated = true;
                first_check = true;
            }
            else{
                P[variable_counter] -= 2 * dp[variable_counter];
                best_err = err;
                dp[variable_counter] *= 1.1;
                updated = true;
                first_check = true;
            }

        }
        else{
            P[variable_counter] += dp[variable_counter];
            dp[variable_counter] *= 0.9;
            updated = true;
            first_check = true;
        }

         */

        if (first_check == false){
            if(err < best_error ){
                best_error = err;
                dp[variable_counter] *= 1.1;
                updated = true;
                first_check = true;
                best_P.push_back({P[0],P[1], P[2]});
                best_err.push_back(best_error);
            }else{
                P[variable_counter] -= 2 * dp[variable_counter];
                first_check = true;
            }
        }
        else{
            if(err < best_error){
                best_error = err;
                dp[variable_counter] *= 1.1;
                updated = true;
                best_P.push_back({P[0],P[1], P[2]});
                best_err.push_back(best_error);
            }else{
                P[variable_counter] += dp[variable_counter];
                dp[variable_counter] *= 0.9;
                updated = true;
            }
        }
        std::cout<<" P: "<<P[0]<<" I: "<<P[1]<<" D: "<<P[2]<<" dp[0]: "<<dp[0]<<" dp[1]: "<<dp[1]<<" dp[2]: "<<dp[2]<<" "<< "Error: "<<err<<" Sum: "<<(dp[0] + dp[1] + dp[2])<<" ";

    }else{
        caliberated = true;
        std::cout<<"\n\nCaliberation Done "<<std::endl;
        int best_idx = std::distance(best_err.begin(), std::min_element(best_err.begin(), best_err.end()));
        std::vector <double> v = best_P[best_idx];
        std::cout<<" P I D :"<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<std::endl;

    }



}