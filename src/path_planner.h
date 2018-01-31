/*
 * path_planner.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Steve Thomas
 *
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <chrono>
#include <map>
#include <string>

using namespace std;

struct Vehicle {
  int lane;
  double x;
  double y;
  double s;
  double s_dot;
  double s_dot_dot;
  double d;
  double prev_x;
  double prev_y;
  string state;
  double T;
};



class PathPlanner {
public:

	vector<double> map_x;
  vector<double> map_y;
  vector<double> map_s;
  
  bool predict_init = false;
  map<int, vector<Vehicle>> predict_cur;
  map<int, vector<double>> predict_past_v;
  map<int, vector<double>> predict_past_acc;
  
  chrono::high_resolution_clock::time_point prev_t;
  vector<double> lane_speed;
  
  int prev_path_size;
  vector<double> prev_path_x;
  vector<double> prev_path_y;
  
  Vehicle ego;
  double cur_s;
  double yaw;
  double v;
  
  string prev_state = "KL";
  int prev_goal_lane = 1;
  
  vector<double> traj_gaps;
  vector<double> traj_goal_dist;

	// Constructor
  PathPlanner();
  PathPlanner(vector<double> map_way_x, vector<double> map_way_y, vector<double> map_way_s);

	// Destructor
  virtual ~PathPlanner();

  void update_predictions(vector<vector<double>> sensor_fusion,  double ego_x, double ego_y, double ego_s);

  void set_ego(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
	
  vector<vector<double>> generate_best_trajectory();
  
  vector<vector<vector<double>>> generate_lane_trajectory(double new_d);
  
  vector<vector<double>> generate_trajectory(double goal_dist, vector<double> goals_s, vector<double> goals_d, bool FOLLOW);
  
  int find_lane(double car_d);
  
};



#endif /* PATH_PLANNER_H_ */
