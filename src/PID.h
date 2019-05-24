#include <uWS/uWS.h>

#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Variables for Twiddle
   */
  double dp[3];
  int state;
  int index;

  double total_error;

  double best_err;

  int counter_iterations;

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
  double UpdateError(double cte, uWS::WebSocket<uWS::SERVER> ws);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  void TotalError(double cte);
  
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
  
  double twiddle();

 
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  
  

  /**
   * PID Coefficients
   */ 
  double p[3];
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H