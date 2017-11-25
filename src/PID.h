#ifndef PID_H
#define PID_H

class PID {

public:
  /*
  * Flag for first run UpdateError function
  */
  bool first_run;

  /*
  * The best error when using TWIDDLE alg
  */
  double best_err;

  /*
  * Index to which K is being tuned during TWIDDLE
  */
  int update_index;

  /*
  * The tuning Ks and stepping amount during TWIDDLE
  */
  double p[3], dp[3];

  /*
  * The flag to switch on the TWIDDLE function
  */
  bool twiddle_on;
 
  /*
  * The state during TWIDDLE
  */
  int state;

  /*
  * The flag which indicates to change another K to be tuned
  */
  bool ch_param;

  /*
  * The counting of probing the error
  */
  int count;

  /*
  * The error sum when probing the error
  */
  double err_sum;

  /*
  * The error average after an epoch of error probing
  */
  double err_average;

  /*
  * The condition for terminate the TWIDDLE cycles
  */
  double twiddle_condition;

  /*
  * The flag which indicate the reset of simulator
  */
  bool reset_trig;

  /*
  * The flag which is used in the TWIDDLE state machine
  * To change the state
  */ 
  bool update_k;

  /*
  * The flag which is used in the TWIDDLE state machine
  * To change the state
  */
  bool sample_cte;

  /*
  * The flag which is used in the TWIDDLE state machine
  * To change the state
  */
  bool sample_cte_done;

  /*
  * The flag to show that TWIDDLE is done
  */
  bool twiddle_done;


  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool twiddle_on);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle algorithm to optimize the K values
  */
  void Twiddle(double condition);

  /*
  * Calculate the goodness
  */
  void Goodness(double cte, int loop_num);

};

#endif /* PID_H */
