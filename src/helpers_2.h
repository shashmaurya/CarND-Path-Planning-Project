#ifndef HELPERS_2_H
#define HELPERS_2_H

#include <math.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"

// for convenience
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Inverse;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.



MatrixXd get_inverse(MatrixXd mat){
   
  float determinant = 0;
  MatrixXd inv = MatrixXd(3,3);
  // Get determinant
  for(int i = 0; i < 3; i++){
		determinant = determinant + (mat(0, i) * (mat(1, (i+1)%3) * mat(2, (i+2)%3) - mat(1, (i+2)%3) * mat(2, (i+1)%3)));
  }
  
  // Get inverse
  for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			//cout<<((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))/ determinant<<"\t";
          inv(i, j) = ((mat((j+1)%3,(i+1)%3) * mat((j+2)%3, (i+2)%3)) - (mat((j+1)%3, (i+2)%3) * mat((j+2)%3, (i+1)%3)))/ determinant;
        }
	}
  
  return inv;

}



vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   
    double a_0 = start[0];
    double a_1 = start[1];
    double a_2 = start[2]/2.0;
    
    MatrixXd t_mat = MatrixXd(3,3);
    
    t_mat<<pow(T, 3), pow(T, 4), pow(T, 5),
                    3.0*pow(T, 2), 4.0*pow(T, 3), 5.0*pow(T, 4),
                    6.0*pow(T, 1), 12.0*pow(T, 2), 20.0*pow(T, 3);
    VectorXd alpha = VectorXd(3);
    VectorXd rhs = VectorXd(3);
    //MatrixXd I = identity(3); 
    
    rhs[0] = end[0] - (start[0] + (start[1]*T)+ (0.5*start[2]* pow(T, 2)));
    rhs[1] = end[1] - (start[1] + (start[2]*T));
    rhs[2] = end[2] - (start[2]);
    
    MatrixXd t_mat_i = t_mat.inverse();
    //MatrixXd t_mat_i = get_inverse(t_mat);
    alpha = t_mat_i*rhs;
    //alpha = rhs*t_mat_t;
    
  //return {1,2,3,4,5,6};
  return {a_0, a_1, a_2, alpha[0], alpha[1], alpha[2]};
}








#endif  // HELPERS_2_H