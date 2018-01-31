/*
 * path_planner.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: Steve Thomas
 *
 */

#include <algorithm>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <iterator>
#include <vector>
#include <chrono>
#include <random>
#include "spline.h"
#include "path_planner.h"
#include "helper_functions.h"
#include "cost.h"


using namespace std;


const double MAX_S = 6945.554;
const int N_LANES = 3;
const double LANE_WIDTH = 4;
const double DT = 0.02;
const int HORIZON = 250;
const int N_PREV_PATH = 30;
const double TARGET_V = mph2mps(49.0);
const double MAX_A = 5.0;
const vector<double> BUFFER_DIST = {10.0, 2.75};

PathPlanner::PathPlanner() {}
PathPlanner::PathPlanner(vector<double> map_way_x, vector<double> map_way_y, vector<double> map_way_s) {
  map_x = map_way_x;
  map_y = map_way_y;
  map_s = map_way_s;
}

PathPlanner::~PathPlanner() {}

void PathPlanner::update_predictions(vector<vector<double>> sensor_fusion, double ego_x, double ego_y, double ego_s) {
  vector<double> lane_n = {0, 0, 0};
  vector<double> lane_v = {0, 0, 0};
  
  chrono::high_resolution_clock::time_point cur_t = chrono::high_resolution_clock::now();
  double temp_dt;
  if (predict_init) {
    temp_dt = chrono::duration_cast<chrono::duration<double>>(cur_t - prev_t).count();
  }
  else {
    prev_t = chrono::high_resolution_clock::now();
    temp_dt = chrono::duration_cast<chrono::duration<double>>(cur_t - prev_t).count();
    predict_init = true;
  }
  
  for (int i=0; i<sensor_fusion.size(); ++i) {
    int car_id = sensor_fusion[i][0];
    double car_x = sensor_fusion[i][1];
    double car_y = sensor_fusion[i][2];
    double car_vx = sensor_fusion[i][3];
    double car_vy = sensor_fusion[i][4];
    double car_s_dot = sqrt(car_vx*car_vx + car_vy*car_vy);
    double car_s = sensor_fusion[i][5];
    double car_d = sensor_fusion[i][6];
    int car_lane = find_lane(sensor_fusion[i][6]);
    
    double dist_diff = distance(ego_x, ego_y, car_x, car_y);
    if ((car_lane >= 0) && (car_s_dot > 0)){
      //Track current speed of every car by lane if it's ahead of car with 5m buffer behind
      double max_dist = HORIZON*DT*TARGET_V*0.25;
      if ((car_lane==0) && (car_s > ego_s - 1) && (dist_diff < max_dist)) {
        lane_n[0] += 1;
        lane_v[0] += car_s_dot;
      }
      else if ((car_lane==1) && (car_s > ego_s - 1) && (dist_diff < max_dist)) {
        lane_n[1] += 1;
        lane_v[1] += car_s_dot;
      }
      else if ((car_lane==2) && (car_s > ego_s - 1) && (dist_diff < max_dist)) {
        lane_n[2] += 1;
        lane_v[2] += car_s_dot;
      }
      
      double car_s_dot_dot = 0;
      double avg_car_s_dot_dot = 0;
      if (predict_past_v[car_id].size() > 0) {
        car_s_dot_dot = (car_s_dot - predict_past_v[car_id][predict_past_v[car_id].size()-1]) / temp_dt;
      }
      predict_past_acc[car_id].push_back(car_s_dot_dot);
      predict_past_v[car_id].push_back(car_s_dot);
      
      for (int j=0; j<predict_past_acc[car_id].size(); ++j) {
        avg_car_s_dot_dot += predict_past_acc[car_id][j];
        }
      avg_car_s_dot_dot /= (double)predict_past_acc[car_id].size();
      
      prev_t = cur_t;
      
      if (predict_past_acc[car_id].size() > 10) {
        predict_past_acc[car_id].erase(predict_past_acc[car_id].begin());
        predict_past_v[car_id].erase(predict_past_v[car_id].begin());
      }
      
      predict_cur[car_id].clear();
      for (int k=0; k<HORIZON; ++k) {
        double temp_s_dot = car_s_dot + avg_car_s_dot_dot*DT*k;
        double temp_s = car_s + car_s_dot*DT*k + 0.5*avg_car_s_dot_dot*DT*k*DT*k;
        while (temp_s > MAX_S) {temp_s -= MAX_S;}

        vector<double> temp_xy = getXY(temp_s, car_d, map_s, map_x, map_y);
        Vehicle temp_pred = {car_lane, temp_xy[0], temp_xy[1], temp_s, temp_s_dot, car_s_dot_dot, car_d, 0.0, 0.0, "KL", temp_dt};
        predict_cur[car_id].push_back(temp_pred);
      }
    }
    else {
      predict_past_acc[car_id].clear();
      predict_past_v[car_id].clear();
      predict_cur[car_id].clear();
    }
  }
  
  double temp_v0;
  double temp_v1;
  double temp_v2;
  if (lane_n[0] > 0) {
    temp_v0 = lane_v[0] / lane_n[0];
  }
  else {
    temp_v0 = TARGET_V;
  }
  if (lane_n[1] > 0) {
    temp_v1 = lane_v[1] / lane_n[1];
  }
  else {
    temp_v1 = TARGET_V;
  }
  if (lane_n[2] > 0) {
    temp_v2= lane_v[2] / lane_n[2];
  }
  else {
    temp_v2 = TARGET_V;
  }
  lane_speed = {temp_v0, temp_v1, temp_v2};
  
}


void PathPlanner::set_ego(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d) {
  prev_path_x = previous_path_x;
  prev_path_y = previous_path_y;
  cur_s = car_s;
  
  
  if (prev_path_x.size() > N_PREV_PATH) {
    prev_path_size = N_PREV_PATH;
  }
  else {
    prev_path_size = (prev_path_x.size());
  }
  
  if (prev_path_size < 3) {
    int temp_lane = find_lane(car_d);
    double temp_x = car_x;
    double temp_y = car_y;
    double temp_s = car_s;
    double temp_d = car_d;
    yaw = deg2rad(car_yaw);
    v = mph2mps(car_speed);
    
    double prev_x = temp_x - cos(yaw);
    double prev_y = temp_y - sin(yaw);
    
    ego = {temp_lane, temp_x, temp_y, temp_s, v, 0.0, temp_d, prev_x, prev_y, "KL", (double)prev_path_size};
  }
  
  else {
    double temp_x = prev_path_x[prev_path_size-1];
    double temp_y = prev_path_y[prev_path_size-1];
    double prev_x = prev_path_x[prev_path_size-2];
    double prev_y = prev_path_y[prev_path_size-2];
    yaw = atan2(temp_y - prev_y, temp_x - prev_x);
    v = distance(temp_x, temp_y, prev_x, prev_y) / DT;
    
    vector<double> temp_frenet = getFrenet(temp_x, temp_y, yaw, map_x, map_y);
    int temp_lane = find_lane(car_d);
    
    double prev_prev_x = prev_path_x[prev_path_size-3];
    double prev_prev_y = prev_path_y[prev_path_size-3];
    double prev_v = distance(prev_x, prev_y, prev_prev_x, prev_prev_y) / DT;
    double a = (v - prev_v) / DT;
    
    ego = {temp_lane, temp_x, temp_y, temp_frenet[0], v, a, temp_frenet[1], prev_x, prev_y, prev_state, (double)prev_path_size};
  }
}

vector<vector<double>> PathPlanner::generate_best_trajectory() {
  int n_paths_generated = 0;
  int cur_best_goal_lane;
  string cur_best_prev_state;
  double goal_d = ego.lane*LANE_WIDTH + 2;
  int temp_goal_lane = ego.lane;
  string temp_goal_state = "KL";
  
  vector<vector<vector<double>>> KL_trajectories = generate_lane_trajectory(goal_d);
  n_paths_generated += KL_trajectories.size();

  double best_cost = total_cost(KL_trajectories[0], {ego.lane, ego.lane}, lane_speed, cur_s, BUFFER_DIST, LANE_WIDTH, HORIZON, TARGET_V, MAX_S, MAX_A, DT, map_x, map_y, predict_cur);
  
  // if previous goal state was Change Lanes and have successfully changed lanes, encourage staying in the lane for one pass
  if ((temp_goal_state != prev_state) && (temp_goal_lane == prev_goal_lane)) {
    best_cost *= 0.01;
  }
  
  double best_gap = traj_gaps[0];
  double best_gd = traj_goal_dist[0];
  vector<vector<double>> best_vals = KL_trajectories[0];
  cur_best_prev_state = temp_goal_state;
  cur_best_goal_lane = temp_goal_lane;
  
  for (int i=1; i<KL_trajectories.size(); ++i) {
    double temp_cost = total_cost(KL_trajectories[i], {ego.lane, ego.lane}, lane_speed, cur_s, BUFFER_DIST, LANE_WIDTH, HORIZON, TARGET_V, MAX_S, MAX_A, DT, map_x, map_y, predict_cur);
    
    // if previous goal state was Change Lanes and have yet to successfully change lanes, encourage to keep changing
    if ((temp_goal_state != prev_state) && (temp_goal_lane == prev_goal_lane)) {
      temp_cost *= 0.01;
    }
    if (temp_cost < best_cost) {
      best_cost = temp_cost;
      best_vals = KL_trajectories[i];
      cur_best_goal_lane = ego.lane;
      best_gd = traj_goal_dist[i];
      best_gap = traj_gaps[i];
    }
  }
  
  if ((ego.lane == 1) || (ego.lane == 2)) {
    double temp_goal_d = goal_d - 4;
    temp_goal_lane = ego.lane - 1;
    temp_goal_state = "TL";
    vector<vector<vector<double>>> TL_trajectories = generate_lane_trajectory(temp_goal_d);
    n_paths_generated += TL_trajectories.size();
    
    for (int k=0; k<TL_trajectories.size(); ++k) {
      double temp_cost = total_cost(TL_trajectories[k], {ego.lane, temp_goal_lane}, lane_speed, cur_s, BUFFER_DIST, LANE_WIDTH, HORIZON, TARGET_V, MAX_S, MAX_A, DT, map_x, map_y, predict_cur);
      if ((temp_goal_state == prev_state) && (temp_goal_lane == prev_goal_lane)) {
        temp_cost *= 0.01;
      }
      if (temp_cost < best_cost) {
        best_cost = temp_cost;
        best_vals = TL_trajectories[k];
        cur_best_prev_state = temp_goal_state;
        cur_best_goal_lane = temp_goal_lane;
        best_gd = traj_goal_dist[k];
        best_gap = traj_gaps[k];
      }
    }
  }
  
  if ((ego.lane == 0) || (ego.lane == 1)){
    double temp_goal_d = goal_d + 4;
    temp_goal_lane = ego.lane + 1;
    temp_goal_state = "TR";
    vector<vector<vector<double>>> TR_trajectories = generate_lane_trajectory(temp_goal_d);
    n_paths_generated += TR_trajectories.size();
    
    for (int j=0; j<TR_trajectories.size(); ++j) {
      double temp_cost = total_cost(TR_trajectories[j], {ego.lane, temp_goal_lane}, lane_speed, cur_s, BUFFER_DIST, LANE_WIDTH, HORIZON, TARGET_V, MAX_S, MAX_A, DT, map_x, map_y, predict_cur);

      // if previous goal state was Change Lanes and have yet to successfully change lanes, encourage to keep changing
      if ((temp_goal_state == prev_state) && (temp_goal_lane == prev_goal_lane)) {
        temp_cost *= 0.01;
     }
      if (temp_cost < best_cost) {
        best_cost = temp_cost;
        best_vals = TR_trajectories[j];
        cur_best_prev_state = temp_goal_state;
        cur_best_goal_lane = temp_goal_lane;
        best_gd = traj_goal_dist[j];
        best_gap = traj_gaps[j];
      }
    }
  }
  
  prev_goal_lane = cur_best_goal_lane;
  prev_state = cur_best_prev_state;
  
  return best_vals;
}


vector<vector<vector<double>>> PathPlanner::generate_lane_trajectory(double new_d) {
  traj_gaps.clear();
  traj_goal_dist.clear();
  
  vector<vector<vector<double>>> possible_trajectories;
  bool FOLLOW = false;
  int new_lane = find_lane(new_d);
  double goal_diff = 100;
  int goal_idx;
  double end_t = HORIZON - prev_path_size;
  for (int i=0; i<predict_cur.size(); ++i) {
    if (predict_cur[i].size() > 0) {
    double dist_diff = distance(ego.x, ego.y, predict_cur[i][ego.T].x, predict_cur[i][ego.T].y);
      if ((new_lane == predict_cur[i][0].lane) && (predict_cur[i][0].s > (cur_s - 2)) && (dist_diff < goal_diff)) {
        FOLLOW = true;
        goal_idx = i;
        goal_diff = dist_diff;
      }
    }
  }
  vector<double> all_goals_dist;
  double goal_dist;
  vector<double> temp_gds;
  if (FOLLOW) {
    //cout << "FOLLOWING..." << endl;
    double goal_x;
    double goal_y;
    for (int i=0; i<3; ++i) {
      double gd = 45 + 20*(2-i);
      goal_x = predict_cur[goal_idx][ego.T + end_t - gd].x;
      goal_y = predict_cur[goal_idx][ego.T + end_t - gd].y;
      goal_dist = distance(ego.x, ego.y, goal_x, goal_y);
      all_goals_dist.push_back(goal_dist);
      temp_gds.push_back(gd);
    }
  }
  if (!FOLLOW) {
    goal_dist = min(TARGET_V*end_t*DT, v*end_t*DT + 0.5*MAX_A*end_t*DT*end_t*DT) + 20;
    for (int i=0; i<3; ++i) {
      double temp_goal_dist = goal_dist - 10*(2-i)  ;
      all_goals_dist.push_back(temp_goal_dist);
      temp_gds.push_back(999+(2-i));
    }
  }
  
  vector<vector<double>> all_goals_s;
  vector<double> temp_gaps;
  
  for (int i=0; i<3; ++i) {
    double gap = 35+10*i;

    double s0 = ego.s + gap;
    while (s0 > MAX_S) {s0 -= MAX_S;}

    double s1 = ego.s + 2*gap;
    while (s1 > MAX_S) {s1 -= MAX_S;}

    double s2 = ego.s + 3*gap;
    while (s2 > MAX_S) {s2 -= MAX_S;}

    double s3 = ego.s + 4*gap;
    while (s3 > MAX_S) {s3 -= MAX_S;}

    all_goals_s.push_back({s0, s1, s2, s3});
    temp_gaps.push_back(gap);
  }
 
  vector<double> goals_d;
  goals_d = {new_d, new_d, new_d, new_d, new_d};
 
  for (int k=0; k<all_goals_s.size(); ++k) {
    vector<double> temp_goals_s = all_goals_s[k];
    double temp_gap = temp_gaps[k];
    
    for (int l=0; l<all_goals_dist.size(); ++l) {
      double temp_goal_dist = all_goals_dist[l];
      double temp_gd = temp_gds[l];
      
      possible_trajectories.push_back(generate_trajectory(temp_goal_dist, temp_goals_s, goals_d, FOLLOW));
      traj_gaps.push_back(temp_gap);
      traj_goal_dist.push_back(temp_gd);
    }
  }
  return possible_trajectories;
}


vector<vector<double>> PathPlanner::generate_trajectory(double goal_dist, vector<double> goals_s, vector<double> goals_d, bool FOLLOW) {
  vector<double> next_wp1 = getXY(goals_s[0], goals_d[0], map_s, map_x, map_y);
  vector<double> next_wp2 = getXY(goals_s[1], goals_d[1], map_s, map_x, map_y);
  vector<double> next_wp3 = getXY(goals_s[2], goals_d[2], map_s, map_x, map_y);
  vector<double> next_wp4 = getXY(goals_s[3], goals_d[3], map_s, map_x, map_y);

  vector<double> pts_x;
  vector<double> pts_y;
  // start with current ego and previous ego
  pts_x.push_back(ego.prev_x);
  pts_x.push_back(ego.x);
  
  pts_y.push_back(ego.prev_y);
  pts_y.push_back(ego.y);
  
  pts_x.push_back(next_wp1[0]);
  pts_x.push_back(next_wp2[0]);
  pts_x.push_back(next_wp3[0]);
  pts_x.push_back(next_wp4[0]);
  
  pts_y.push_back(next_wp1[1]);
  pts_y.push_back(next_wp2[1]);
  pts_y.push_back(next_wp3[1]);
  pts_y.push_back(next_wp4[1]);
  
  vector<double> shift_pts_x;
  vector<double> shift_pts_y;
  double shift_x;
  double shift_y;
  // shift car ref angle to be 0 and position to [0,0]
  double prev_temp_x = -999;
  double temp_x;
  for (int i=0; i<pts_x.size(); ++i) {
    
    shift_x = pts_x[i] - ego.x;
    shift_y = pts_y[i] - ego.y;
    temp_x = (shift_x*cos(0-yaw) - shift_y*sin(0-yaw));
    if (temp_x > prev_temp_x) {
      shift_pts_x.push_back(temp_x);
      shift_pts_y.push_back(shift_x*sin(0-yaw) + shift_y*cos(0-yaw));
      prev_temp_x = temp_x;
    }
  }

  // Create and set (x,y) points to spline:
  tk::spline s;
  s.set_points(shift_pts_x, shift_pts_y);
  
  // Define points to send to planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  // Start with previous path points
  for (int i=0; i<prev_path_size; ++i) {
    next_x_vals.push_back(prev_path_x[i]);
    next_y_vals.push_back(prev_path_y[i]);
  }
  
  double x_add_on = 0;
  double ref_car_v = v;
  double cur_ref_v;
  
  double target_x = 50;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  
  double end_t = HORIZON - prev_path_size;
  double cur_a;
  
  double temp_a = 2*(goal_dist - v*end_t*DT) / (end_t*DT*end_t*DT);
  if (temp_a > MAX_A) {
    cur_a = MAX_A;
  }
  else if (temp_a < -1*MAX_A) {
    cur_a = -1*MAX_A;
  }
  else {
    cur_a = temp_a;
  }
  
  for (int k=0; k<end_t; ++k) {
    cur_ref_v = ref_car_v + min(cur_a*DT, TARGET_V - ref_car_v);
    if (cur_ref_v > TARGET_V) {
      ref_car_v = TARGET_V;
    }
    else {
      ref_car_v = cur_ref_v;
    }
    
    double N = target_dist/DT/cur_ref_v;
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
      
    x_add_on = x_point;
      
    double x_ref = x_point;
    double y_ref = y_point;
      
    // rotate back to global coordinates
    x_point = x_ref*cos(yaw) - y_ref*sin(yaw) + ego.x;
    y_point = x_ref*sin(yaw) + y_ref*cos(yaw) + ego.y;
      
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
     
    }
  return {next_x_vals, next_y_vals};
}

int PathPlanner::find_lane(double car_d) {
  int car_lane;
  for (int i=0; i<N_LANES; ++i) {
    if (car_d >= i*LANE_WIDTH && car_d <= (i+1)*LANE_WIDTH) {
      car_lane = i;
    }
  }
  return car_lane;
}
