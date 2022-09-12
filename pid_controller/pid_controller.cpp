/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->cte = 100000;
  this->diff_cte = 0;
  this->int_cte = 0;
  this->best_error = 0;

  this->P = Kpi;
  this->I = Kii;
  this->D = Kdi;

  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;
  this->delta_time = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  if (this->delta_time == 0){
    this->diff_cte = 0;
    }
  else{
  this->diff_cte = (cte - this->cte)/this->delta_time;}
  this->int_cte += cte;
  this->cte = cte;
    
}

void PID::UpdatePID(double P, double I, double D) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  this->P = P;
  this->I = I;
  this->D = D;
    
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
  //double control;
  double control = this->P*this->cte + this->D*this->diff_cte + this->I*this->int_cte;
  control = max(control, this->output_lim_min);
  control = min(control, this->output_lim_max);
  return control;
}

double PID::Twiddle(double inp, double tune){
  if (this->cte < this->best_error){
    this->best_error = this->cte;
    tune *= 1.1;
  }
  else{
    inp -= 1.1*tune;
  }
  return inp, tune;

                //else:
                  //  p[i] += dp[i]
                   // dp[i] *= 0.9
                     
  
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  this->delta_time = new_delta_time;
  return this->delta_time;
}