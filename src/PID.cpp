#include "PID.h"

using namespace std;

#define DEFAULT_TWIDDLE 0.002

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle_on) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  i_error = 0.0;
  update_index = 0;
  
  for (int i = 0; i < 3; i++) {
    p[i] = 0.0;
    dp[i] = 1.0;
  }

  first_run = true;
  PID::twiddle_on = twiddle_on;
  ch_param = false;
  state = 0;
  err_sum = 0.0;
  err_average = 0.0;
  count = 0;
  reset_trig = false;
  update_k = true;
  sample_cte = false;
  sample_cte_done = false;
  twiddle_done = false;
  twiddle_condition = DEFAULT_TWIDDLE;
}

void PID::UpdateError(double cte) {
  if (first_run) {
    p_error = cte;
    best_err = cte*cte;
    first_run = false;
  }
  
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
 
}

/*
* The TWIDDLE state machine
*/
void PID::Twiddle(double condition) {
  twiddle_condition = condition;
  if (twiddle_on) {
    switch (state) {
      case 0:
        if (!sample_cte) {
          sample_cte = true;
        }

        if (sample_cte_done) {
          update_k = true;
          sample_cte_done = false;
          best_err = err_average;
          state = 1;
          reset_trig = true;
        }
        break;
        
      case 1:
        if (update_k) {
          p[update_index] += dp[update_index];
          update_k = false;
          sample_cte = true;
          sample_cte_done = false;
          state = 2;
        }
        break;

      case 2:
        if (sample_cte_done) {
          sample_cte_done = false;
          if (err_average < best_err) {
            best_err = err_average;
            dp[update_index] *= 1.1;
            state = 1;
            update_k = true;
            update_k = true;
            ch_param = true;
          } else {
            p[update_index] -= 2*dp[update_index];
            state = 3;
            sample_cte = true;            
            ch_param = false;
          }
          reset_trig = true;
        }
        break;

      case 3:
        if (sample_cte_done) {
          sample_cte_done = false;
          if (err_average < best_err) {
            best_err = err_average;
            dp[update_index] *= 1.1;
          } else {
            p[update_index] += dp[update_index];
            dp[update_index] *= 0.9;
          }
          state = 1;
          update_k = true;
          ch_param = true;
          reset_trig = true;
        }
        break;

        default:
        break;
    }

    if (ch_param) {
      ch_param = false;
      int index = update_index + 1;
      update_index = index%3;
    }

    double sum_dp = dp[0] + dp[1] + dp[2];
    if (sum_dp < twiddle_condition) {
      twiddle_on = false;
      Kp = p[0];
      Kd = p[1];
      Ki = p[2];
      twiddle_done = true;
    }
  }
}

void PID::Goodness(double cte, int loop_num) {
  if (sample_cte) { 
    if (count == loop_num - 1) {
      err_average = err_sum/loop_num;
      err_sum = cte*cte;
      count = 0;
      sample_cte = false;
      sample_cte_done = true;
    } else {
      err_sum += cte*cte;
      count++;
    }
  }
}

double PID::TotalError() {
  //std::cout << "p_error is " << p_error << std::endl;
  double control;
  if (twiddle_on) {
    control = -p[0]*p_error - p[1]*d_error - p[2]*i_error;
  } else {
    control = -Kp*p_error - Kd*d_error - Ki*i_error;
  }
  return control;
}

