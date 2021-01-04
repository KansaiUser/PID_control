#include "PID.h"

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

  previous_cte=0;  //perhaps change this
  //sum_cte=0;


}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  //mycte= cte;
  p_error=cte;
  d_error=cte- previous_cte ;  //supposed to be divided by dt but do we have dt??
  i_error += cte;
  previous_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}

double PID::GetResult(){

  double res= -Kp * p_error -Kd * d_error -Ki* i_error;
  return  res;
}