#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <map>

// for convenience
using std::string;
using std::vector;


/* Add a type for actions */
typedef enum {
  STAY_IN_LANE = 0,
  CHANGE_LANE_LEFT,
  CHANGE_LANE_RIGHT
}ACTION;

/* Params */
constexpr int INVALID_LANE = 100000;
constexpr int LANE_WIDTH = 4;
constexpr double MAX_DIST_REAR = -25.0;
constexpr double MAX_DIST_FRONT = 75.0;
constexpr double MPS_TO_MPH = 2.23694;
constexpr double MPH_TO_MPS = 0.44704;
constexpr double TOP_MPH = 49.5;
constexpr int LANE_CHANGE_NEEDED = 500;
constexpr int CAR_IN_LANE = 1000;
constexpr int MINIMUM_CLEARANCE_FRONT =  35;
constexpr int MINIMUM_CLEARANCE_REAR  = -15;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

/* Process the date and return an action to be performed */
ACTION processData(double my_car_s, double my_future_s, int lane, std::vector<std::vector<double>> sensor_fusion, int prev_size, double &speed){
  ACTION result = STAY_IN_LANE;
  
  /* the speed of the car in front */
  double car_ahead_speed = 0;
  
  /* lane costs */
  std::map<int,int> lane_cost;
  lane_cost[-1] = INVALID_LANE;
  lane_cost[0] = 0;
  lane_cost[1] = 0;
  lane_cost[2] = 0;
  lane_cost[3] = INVALID_LANE;
  
  /* Track closest cars in each lane */
  std::vector<double> closest_distance_in_lane(3, 500.0);
  
  for (unsigned  int i = 0; i < sensor_fusion.size(); i++){
    double other_car_vx = sensor_fusion[i][3];
    double other_car_vy = sensor_fusion[i][4];
    double other_car_s  = sensor_fusion[i][5];
    double other_car_d  = sensor_fusion[i][6];
    
    /* use other car velocity to calculate the s in the future */
    double other_car_mps = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy );
    double other_car_future_s = other_car_s + static_cast<double>(prev_size) * 0.02 * other_car_mps;
    
    /* determine the lane on which the car is */
    int other_car_lane = floor(other_car_d / LANE_WIDTH);
    
    /* get the distance to the other car */
    double other_car_distance = other_car_s - my_car_s;
    
    /* only consider cars that are near */
    if ((other_car_distance < MAX_DIST_REAR) || (other_car_distance > MAX_DIST_FRONT)){
      continue;
    }
    
    /* keep track of the closest distance in each lane */
    double future_distance = other_car_future_s - my_future_s;
    
    if ((future_distance > 0) && (future_distance < closest_distance_in_lane[other_car_lane])){
      closest_distance_in_lane[other_car_lane] = future_distance;
    }
    
    /* only consider cars in front*/
    if (((other_car_distance > MINIMUM_CLEARANCE_REAR) && (other_car_distance < MINIMUM_CLEARANCE_FRONT)) ||
            ((future_distance > MINIMUM_CLEARANCE_REAR) && (future_distance < MINIMUM_CLEARANCE_FRONT))){
      lane_cost[other_car_lane] += CAR_IN_LANE;
      if (lane == other_car_lane){
        car_ahead_speed = other_car_mps * MPS_TO_MPH;
      }
    }
  }
  
  /* if cost of current lane is high try to change the lane */
  if (lane_cost[lane] > LANE_CHANGE_NEEDED){
    /* get the cost for other lanes */
    if ((lane_cost[lane - 1] >= CAR_IN_LANE) && (lane_cost[lane + 1] < CAR_IN_LANE)){
      /* we choose to stay on current lane */
      result = STAY_IN_LANE;
      speed = car_ahead_speed - 1.0f;
    }
    else if ((lane_cost[lane - 1] < CAR_IN_LANE) && (lane_cost[lane + 1] < CAR_IN_LANE)){
      /* we can go either way*/
      result = (closest_distance_in_lane[lane - 1] < closest_distance_in_lane[lane + 1]) ? CHANGE_LANE_RIGHT : CHANGE_LANE_LEFT;
      speed = TOP_MPH;
    }
    else if (lane_cost[lane - 1] < CAR_IN_LANE){
      /* change to left lane*/
      result = CHANGE_LANE_LEFT;
      speed = TOP_MPH;
    }
    else if (lane_cost[lane + 1] < CAR_IN_LANE){
      /* change to right lane*/
      result = CHANGE_LANE_RIGHT;
      speed = TOP_MPH;
    }
  }
  else {
    /* no need to change the lane
    attempt to go to the middle lane*/
    if ((lane == 0) && (lane_cost[1] < CAR_IN_LANE)){
      result = CHANGE_LANE_RIGHT;
      speed = TOP_MPH;
    }
    else if ((lane == 2) && (lane_cost[1] < CAR_IN_LANE)){
      result = CHANGE_LANE_LEFT;
      speed = TOP_MPH;
    }
    else {
      result = STAY_IN_LANE;
      speed = TOP_MPH;
    }
  }
  return result;
}

#endif  // HELPERS_H