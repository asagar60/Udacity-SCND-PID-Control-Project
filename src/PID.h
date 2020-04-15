#ifndef PID_H
#define PID_H

#include <vector>
class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void Twiddle(double cte);
  int i;
  bool caliberated;
  std::vector <std::vector <double>> best_dp;
  std::vector <std::vector <double>> best_P;
  std::vector <double> best_err;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  double threshold ;
  double best_error ;
  double err;

  int variable_counter;

  bool updated ;
  bool first_check;
  bool INIT;

  int rounds;



  std::vector<double> dp;
  std::vector <double> P;
};

#endif  // PID_H