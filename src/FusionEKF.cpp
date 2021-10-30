#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 10, 0, 0, 0,
             0, 10, 0, 0,
             0, 0, 100, 0,
             0, 0, 0, 100;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
  /**
   * Initialization
   */
  if (!is_initialized_) 
  {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_[0]; // range = the magnitude of ρ
  	  double phi = measurement_pack.raw_measurements_[1]; // bearing = atan(py/px) 
  	  double rho_dot = measurement_pack.raw_measurements_[2]; // Velocity = The projection of the velocity, v, onto the line L
      
      double x = rho * cos(phi);
      std::cout << "x for Radar is " << x << std::endl;
      
      if ( x < 0.0001 ) 
      {
        x = 0.0001;
      }
      
      double y = rho * sin(phi); 
      std::cout << "y for Radar is " << y << std::endl;
      if ( y < 0.0001 ) 
      {
        y = 0.0001;
      }
      
      double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);
      
      ekf_.x_ << x, y, vx , vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      //Initialize state.
      //Velocity is zero.
      
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }

    // done initializing, no need to predict or update
    // Saving first timestamp in second
    previous_timestamp_ = measurement_pack.timestamp_ ;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  

  
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
    
  
  // State transition matrix update
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Noise covariance matrix computation
  double noise_ax = 8.95;
  double noise_ay = 8.95;
  
  
  // Pre-calculations for variables in matrix
  double dt_2 = pow(dt,2); // (delta time)^2
  double dt_3 = pow(dt,3); // (delta time)^3
  double dt_4 = pow(dt,4); // (delta time)^4
  double dt_4_4 = dt_4 / 4;// ((delta time)^4)/4
  double dt_3_2 = dt_3 / 2; //((delta time)^3)/2
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
	         0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
	         dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
 	         0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  
  {
    // TODO: Radar updates
    
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
  
  {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
