//
//  cost.h
//  path_planning
//
//  Created by Steven Thomas on 1/26/18.
//

#ifndef cost_h
#define cost_h

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <thread>
#include "path_planner.h"
#include "helper_functions.h"

using namespace std;


double inefficiency_cost(vector<vector<double>> vals, vector<int> intended_lanes, vector<double> lane_speed, double TARGET_V) {
  int fast_lane = 0;
  double fast_v = lane_speed[0];
  
  for (int i=1; i<lane_speed.size(); ++i) {
    if (lane_speed[i] > fast_v) {
      fast_lane = i;
      fast_v = lane_speed[i];
    }
  }
  // encourage double lane change if the fast lane is two over
  if ((intended_lanes[0] + 2 == fast_lane) || (intended_lanes[0] - 2 == fast_lane)) {
    lane_speed[1] = fast_v;
  }
  double cost = 2 - (lane_speed[intended_lanes[0]] / fast_v) - (lane_speed[intended_lanes[1]] / fast_v);
  return cost;
}


double d_distance_cost(vector<vector<double>> vals, vector<int> intended_lanes, double LANE_WIDTH, vector<double> map_x, vector<double> map_y) {
  double d_cost = 0;
  const double MAX_D_DIST = 1.0;
  int path_size = vals[0].size();
  double goal_d = intended_lanes[1]*LANE_WIDTH + 2;
  
  for (int i=0; i<path_size-1; ++i) {
    double temp_yaw = atan2(vals[1][i+1] - vals[1][i], vals[0][i+1] - vals[0][i]);
    vector<double> temp_frenet = getFrenet(vals[0][i+1], vals[1][i+1], temp_yaw, map_x, map_y);
    double temp_d_diff = fabs(temp_frenet[1] - goal_d);
    if (temp_d_diff > MAX_D_DIST) {
      d_cost += 1;
    }
  }
  d_cost /= (path_size-1);
  return d_cost;
}


double s_distance_cost(vector<vector<double>> vals, vector<int> intended_lanes, double cur_s, int HORIZON, double DT, double TARGET_V, double MAX_S, vector<double> map_x, vector<double> map_y) {
  const double MAX_DIST = (HORIZON+15)*DT*TARGET_V;
  int path_size = vals[0].size();
  double temp_yaw = atan2(vals[1][path_size-1] - vals[1][path_size-2], vals[0][path_size-1] - vals[0][path_size-2]);
  vector<double> temp_frenet = getFrenet(vals[0][path_size-1], vals[1][path_size-1], temp_yaw, map_x, map_y);
  double end_s = temp_frenet[0];
  if (end_s < cur_s) {end_s += MAX_S;}
  double dist = end_s - cur_s;
  double cost = (MAX_DIST - dist) / MAX_DIST;
  return cost;
}


double collision_cost(vector<vector<double>> vals, vector<double> BUFFER_DIST, vector<double> map_x, vector<double> map_y, map<int, vector<Vehicle>> predict_cur) {
  double cost = 0;
  for (int i=0; i<predict_cur.size(); ++i) {
    if (predict_cur[i].size() > 0) {
      for (int j=0; j<150; ++j) {
        double temp_ego_x0 = vals[0][j];
        double temp_ego_y0 = vals[1][j];
        double temp_ego_x1 = vals[0][j+1];
        double temp_ego_y1 = vals[1][j+1];
        
        double temp_ego_yaw = atan2(temp_ego_y1 - temp_ego_y0, temp_ego_x1 - temp_ego_x0);
        vector<double> temp_frenet = getFrenet(temp_ego_x1, temp_ego_y1, temp_ego_yaw, map_x, map_y);
        
        double temp_car_x1 = predict_cur[i][j+1].x;
        double temp_car_y1 = predict_cur[i][j+1].y;
        double temp_car_d = predict_cur[i][j+1].d;
        
        double temp_dist = distance(temp_ego_x1, temp_ego_y1, temp_car_x1, temp_car_y1);
        double temp_d_dist = fabs(temp_car_d - temp_frenet[1]);
        if ((temp_d_dist < BUFFER_DIST[1]) && (temp_dist < BUFFER_DIST[0])) {
          cost += 1;
        }
      }
    }
  }
  return cost;
}
  

double total_cost(vector<vector<double>> vals, vector<int> intended_lanes, vector<double> lane_speed, double cur_s, vector<double> BUFFER_DIST, double LANE_WIDTH, int HORIZON, double TARGET_V, double MAX_S, double MAX_A, double DT, vector<double> map_x, vector<double> map_y, map<int, vector<Vehicle>> predict_cur) {
  
  //Cost coeffs have different orders of magnitude because they operate as logic gates
  const double Ki = 10.0;
  const double Kdd = 1.0;
  const double Ksd = 1.0;
  
  // BOOLEAN COSTS
  const double Kc = 100.0;
  
  double Ci = inefficiency_cost(vals, intended_lanes, lane_speed, TARGET_V);
  double Cdd = d_distance_cost(vals, intended_lanes, LANE_WIDTH, map_x, map_y);
  double Csd = s_distance_cost(vals, intended_lanes, cur_s, HORIZON, DT, TARGET_V, MAX_S, map_x, map_y);
  double Cc = collision_cost(vals, BUFFER_DIST, map_x, map_y, predict_cur);

  return Ki*Ci + Kdd*Cdd + Ksd*Csd + Kc*Cc; // + Kacc*Cacc + Kjerk*Cjerk;
}

#endif /* cost_h */
