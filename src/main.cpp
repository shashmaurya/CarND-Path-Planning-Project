#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "helpers_2.h"
#include "spline.h"     // Added for generating splines for path 

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  
  
  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  // Added for initializing values
  int lane_num = 1;            // Vehicle's current lane number (of 0, 1, 2) 
  double lane_width = 4.0;     // Define lane width
  double target_speed = 0.0;   // Intialize target speed to zero to prvent sudden accelation

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane_num, &lane_width, & target_speed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //____________________________________________________________________________

          /**
           * Define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          double pos_x;
          double pos_y;
          double angle;
          double pos_yaw;
          vector<double> range_x;    // Range vectors to be used for spline
          vector<double> range_y;                              
          int path_size = previous_path_x.size(); // Number of point in current path 
          double speed_limit = 48.0;              // Define speed limit just under 50
          
          pos_x = car_x;
          pos_y = car_y;
          pos_yaw = deg2rad(car_yaw);
                   
          // Intialize car s to the end of last path
          if(path_size>0){          
            car_s = end_path_s;
          }
          
          
          // SECTION 1: Decision making for driving changes
          bool too_close = false;   // True if a vehicle in front is close
          double safe_distance_front = 30.0;   // Defined safe distance for above value
                   
          for(int i=0; i<sensor_fusion.size(); ++i){
            
            // Check if it's in same lane
            float d = sensor_fusion[i][6];           

            if((d<(((lane_num*lane_width)+ 2) + 2)) && (d>(((lane_num*lane_width)+ 2) - 2))){
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += ((double)path_size*0.02*check_speed);  // Use car position one second later
              
              // If the car is ahead, and distance is less than safe limit
              if((check_car_s > car_s) && ((check_car_s-car_s)<safe_distance_front)){                
                too_close = true;
              }              
            }            
          }
          
          // If a vehicle is too close ahead, slow down
          if(too_close){
            target_speed -= 0.224;
          }
          else if(target_speed<speed_limit){
            target_speed += 0.224;          
          }         
                    
          
          // SECTION 1.1: Lane Change options
          if (too_close && target_speed < 40.0){            
            
            vector<int> check_lanes;   // Vector to hold possible lane change options            
            // Check left lane
            if(lane_num == 0){   // Left Lane
              // Possible lane: 1
              check_lanes.push_back(1);
            } else if (lane_num == 1){   // Middle Lane
              // Possible lanes: 2 and 0
              check_lanes.push_back(0);
              check_lanes.push_back(2);
            } else if (lane_num ==2){
              // Possible lane: 1 
              check_lanes.push_back(1);
            }
            
            
            // Check all cars for lane clearance
            double safe_distance_lc = 20.0;    // Safe clearance for lane change
            bool lc_safe = true;
            int safe_lane = -1;
            
            for(int j=0; j<check_lanes.size(); ++j){
              int pot_lane = check_lanes[j];   // Potential lane
              
              for(int i=0; i<sensor_fusion.size(); ++i){
                
                float d = sensor_fusion[i][6];   // Get d value from sensor data
                
                // Check if the vehicle is in the potential lane
                if((d<(((pot_lane*lane_width)+ 2) + 2)) && (d>(((pot_lane*lane_width)+ 2) - 2))){

                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
                  double check_car_s = sensor_fusion[i][5];

                  check_car_s += ((double)path_size*0.02*check_speed);  // For vehicle position one second later
              
                  if(fabs(check_car_s-car_s) < safe_distance_lc){       // check magnitude of difference for safe distance         
                    lc_safe = false;
                    break;
                  }              
                }                
              }
              
              if (lc_safe){
                  lane_num = pot_lane;    // If lane change is safe, set new lane number
                  cout<<"Changing lane safe. Moving to: "<< lane_num<<endl;
                  break;
              }              
            }
            
            
          } // End: Lane change options
          
                   
          
          
          // SECTION 2: Calculation of the next points in the path
          
          // SECTION 2.1: Build x and y vectors for spline generation
          if(path_size<2){  // To initialize the path if less than two elements
            
            // Calculate previous car position based on yaw
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            
            // Add the above values to the x and y range vector
            range_x.push_back(prev_car_x);
            range_x.push_back(car_x);
            
            range_y.push_back(prev_car_y);
            range_y.push_back(car_y);
            
          } else{
            
            // Get last two points to maintain continuity
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            
            double prev_pos_x = previous_path_x[path_size-2];
            double prev_pos_y = previous_path_y[path_size-2];
            pos_yaw = atan2(pos_y-prev_pos_y, pos_x-prev_pos_x); // Calculate angle
            
            // Add the points to x and y range vectors for spline
            range_x.push_back(prev_pos_x);
            range_x.push_back(pos_x);
            
            range_y.push_back(prev_pos_y);
            range_y.push_back(pos_y);
          }
          
          double range_inc = 30.0;   // Incremental distance for points to be used in spline
          
          // Get x an y coordinates for three points in incremental range
          vector<double> next_wp0 = getXY(car_s+(1.0*range_inc), (lane_num+0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+(2.0*range_inc), (lane_num+0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+(3.0*range_inc), (lane_num+0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // Add the x and y values to range vectors for spline
          range_x.push_back(next_wp0[0]);
          range_x.push_back(next_wp1[0]);
          range_x.push_back(next_wp2[0]);

          range_y.push_back(next_wp0[1]);
          range_y.push_back(next_wp1[1]);
          range_y.push_back(next_wp2[1]);
          
          // Shift correction for vehicle position
          for(int i =0; i< range_x.size(); ++i){
            double shift_x = range_x[i] - pos_x;
            double shift_y = range_y[i] - pos_y;
            
            range_x[i] = (shift_x*cos(0.0-pos_yaw)) - (shift_y*sin(0.0-pos_yaw));
            range_y[i] = (shift_x*sin(0.0-pos_yaw)) + (shift_y*cos(0.0-pos_yaw));            
          }
          
          //Set spline and use the range  vector set up above
          tk::spline sp;          
          sp.set_points(range_x, range_y);         
          
          
          // SECTION 2.2: Build the path for next one second with 50 points
          //Intialize the vector for next point with previous points
          for(int i=0; i< previous_path_x.size(); ++i){            
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);            
          }
                    
          double target_x = 30.0;            // Target x increment of 30
          double target_y = sp(target_x);    // Get corresponding y from spline
          double target_dist = sqrt(pow(target_x, 2)+ pow(target_y,2));   // Calculate target distance
          
          double x_add_on = 0.0;   // Addon value to start x point from
          
          // Loop to add however many points were used from the previous path
          for(int i=0; i<= 50-previous_path_x.size(); ++i){
            double N= (target_dist/(0.02* target_speed/2.24));   // Number of points based on time and speed
            double x_point = x_add_on+ (target_x/N);         // Increment x point 
            double y_point = sp(x_point);                    // Get corresponding y from spline
            
            x_add_on = x_point;   // Save value for next iteration

            // Convert back the coordinates
            double x_ref = x_point;
            double y_ref = y_point;            
            x_point = (x_ref*cos(pos_yaw)) - (y_ref*sin(pos_yaw));
            y_point = (x_ref*sin(pos_yaw)) + (y_ref*cos(pos_yaw));
            
            x_point += pos_x;
            y_point += pos_y;
            
            // Add values to the next path array
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);  
                    
          }
          
           
          
          //____________________________________________________________________________

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}