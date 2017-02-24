#include <PID_CONTROLLER.hpp>
#include <iostream>
/**
 * @file   PID_CONTROLLER.cpp
 * @date   Feb 20, 2017
 * @author Lamar Simpson
 *
 *@brief Proportional Integral Derivative (PID) Controller
 *@details implementation of PID_CONTROLLER class.
 */
//using namespace std::cout;
//using namespace std::cin;
//using namespace std::endl;


PID_CONTROLLER::PID_CONTROLLER(double Kp, double Ki, double Kd, double dt)
	:Kp(Kp),Ki(Ki),Kd(Kd),dt(dt),error(0),integral(0) {
	
  if ( Kp < 0 ) { PID_CONTROLLER::setKp(0); } 
  if ( Ki < 0 ) { PID_CONTROLLER::setKi(0); } 
  if ( Kd < 0 ) { PID_CONTROLLER::setKd(0); } 
}



/**
   * @brief Computes the new input.
   * @return result double
   */

double PID_CONTROLLER::compute(double setPoint, double actualVelocity) {
    
    double currentError = actualVelocity - setPoint; 
    
    double result = currentError * Kp + (currentError - error) / dt * Kd + integral * Ki;

	return currentError;
}



/**
   * @brief Return value of Kp.
   * @return Kp double.
   */
  double PID_CONTROLLER::getKp(){ return this->Kp; }

  /**
   * @brief Return the value of Ki.
   * @return Ki double.
   */


  double PID_CONTROLLER::getKi() {return this->Ki;}

  /**
   * @brief Return the value of Kd
   * @return Kd double.
   */
 double PID_CONTROLLER::getKd(){return this->Kd;}

  /**
   * @brief Change the value of Kp class member variable
   * @param Kp double.
   */

  void PID_CONTROLLER::setKp(double Kp){this->Kp = Kp;}

  /**
   * @brief Change the value Ki  class member variable.
   * @param Ki double.
   */

  void PID_CONTROLLER::setKi(double Ki){this->Ki = Ki;}

  /**
   * @brief Change the value of Kd class member variable
   * @param Kd double.
   */

  void PID_CONTROLLER::setKd(double Kd){this->Kd = Kd;}
