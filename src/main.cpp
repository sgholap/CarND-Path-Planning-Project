#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "Spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Enum for lanes
enum LANE_NUMBER {
	LANE0,
	LANE1,
	LANE2,
	INVALID_LANE
};

// Enum for states
enum STATE {
	KEEP_LANE,
	CHANGE_LEFT,
	CHANGE_RIGHT,
	PREPARE_CHANGE_LEFT,
	PREPARE_CHANGE_RIGHT
};

// Paramters to tune
const double target_max_speed = 49.5;
const double target_max_acceleration = 0.223;

const double target_max_dist_for_lane_change_look = 60;

const double target_min_turn_front_dist = 20;
const double target_min_turn_back_dist = 15;

const double target_min_deaccer_front_dist = 30;

const double target_critical_front_dist = 2;

// Global Variable
STATE prev_state = KEEP_LANE;
LANE_NUMBER intendedLaneChange = INVALID_LANE;


// Get the lane number based on d
LANE_NUMBER getLaneNumber(double d) {
	if (d >= 0 && d <= 4)
	{
		return LANE0;
	}
	else if (d >= 4 && d <= 8)
	{
		return LANE1;
	}
	else if (d >= 8 && d <= 12)
	{
		return LANE2;
	}
	else
	{
		return INVALID_LANE;
		std::cout << "++++++ Invalid lane ++++++" << std::endl;
	}
}

// Find the closes vechicle in front or back.
// parameters are current vechicle s and d, future Distance, sensor fustion data, front or back.
vector<double> closestVehicle(double ref_s, double ref_d, int lane, double futureDistance, vector < vector<double>> sensor_fusion, bool isFront)
{
	double speed = target_max_speed;
	double distance = std::numeric_limits<double>::max();
	double d_l = std::numeric_limits<double>::max();
	for (int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++)
	{
		float d = sensor_fusion[vehicle][6];
		double vx = sensor_fusion[vehicle][3];
		double vy = sensor_fusion[vehicle][4];
		double other_car_s = sensor_fusion[vehicle][5];
		double other_car_speed = sqrt(vx*vx + vy * vy);
		other_car_s += futureDistance * 0.02 * other_car_speed;
		if (getLaneNumber(d)  != INVALID_LANE && getLaneNumber(d) == lane)
		{
			if (isFront)
			{
				// Find closest in front;
				if (other_car_s >= ref_s) {
					if (distance > other_car_s - ref_s) {
						distance = other_car_s - ref_s;
						speed = other_car_speed;
						d_l = std::abs(ref_d - d);
					}
				}
			}
			else
			{
				// Find closest in back
				if (ref_s > other_car_s) {
					if (distance > ref_s - other_car_s)
					{
						distance = ref_s - other_car_s;
						speed = other_car_speed;
						d_l = std::abs(ref_d - d);
					}
				}
			}
		}
	}

	std::cout << "Close vehicle " << (isFront ? "Front: " : "Back: ") << lane << " " << distance << " " << speed << " " << d_l << std::endl;
	return { distance, speed, d_l};
}

// Check if state can be keep lane.
bool checkKeepLane(const vector<vector<vector<double>>> parameters,const int bestLane, const int curLane)
{
	// Check if see any car in our lane.
	// We can compare by lane number,
	// But we may miss if front car is in changing lane and it may collide.
	bool keeplane = false;
	for (int t = 0; t < 3; t++)
	{
		if (parameters[t][0][0] < target_min_turn_front_dist &&
			parameters[t][0][2] < 2.5)
		{
			keeplane = true;
			break;
		}
	}
	// Change lane only if require.
	// keep in lane if:
	// we don't want to change lane as enough distance is available
	// or we are close to target,
	// or we are already in best lane.
	// or if we have best lane at two lane distance but car in adjacent lane is very close.
	if (parameters[curLane][0][0] > target_max_dist_for_lane_change_look ||
		keeplane ||
		bestLane == curLane ||
		(abs(bestLane - curLane) > 1 && (parameters[1][0][0] < target_min_turn_front_dist)))
	{
		return true;
	}
	return false;
}

// Action if state need to be in keep lane.
// return target speed and distance.
void actionKeepLane(const vector<vector<vector<double>>> &parameters, STATE &newstate,const int &curLane, double &target_speed, double &front_distance, int &target_lane)
{
	bool keeplane = false;
	double speed = target_max_speed;
	double distance = 1000; // Big number


	// Check if see any car in our lane.
	// We can compare by lane number,
	// But we may miss if front car is in changing lane and it may collide.
	for (int t = 0; t < 3; t++)
	{
		if (parameters[t][0][0] < target_min_deaccer_front_dist &&
			parameters[t][0][2] < 2)
		{
			keeplane = true;
			speed = parameters[t][0][1];
			distance = parameters[t][0][0];
			break;
		}
	}
	// Check if same lane;
	newstate = KEEP_LANE;
	if (keeplane)
	{
		// If car is near the threshold for min dist, set speed = front car speed.
		target_speed = speed;
		front_distance = distance;
	}
	else {
		target_speed = target_max_speed;
		front_distance = 1000;
	}
	target_lane = curLane;
	intendedLaneChange = INVALID_LANE;
	std::cout << "Keep in same Lane " << std::endl;
}

// Action to preapare lane.
void actionPrepareLane(const vector<vector<vector<double>>> &parameters, STATE &newstate, const int &curLane, const int &bestLane,  double &target_speed, double &front_distance, int &target_lane)
{
	// Better lane available. We can prepare.
	int newLaneDir;
	if (bestLane > curLane)
	{
		newstate = PREPARE_CHANGE_RIGHT;
		newLaneDir = 1;
	}
	else
	{
		newstate = PREPARE_CHANGE_LEFT;
		newLaneDir = -1;
	}

	if (parameters[curLane + newLaneDir][0][0] < target_min_deaccer_front_dist)
	{
		// if front car is close set the speed equal to front car.
		target_speed = parameters[curLane + newLaneDir][0][1];
	}
	else
	{
		// Else send max speed
		target_speed = target_max_speed;
	}
	front_distance = parameters[curLane][0][0];
	target_lane = curLane;
	std::cout << "Prepare for next lane to " << (newstate == PREPARE_CHANGE_LEFT ? "LEFT" : "RIGHT") << std::endl;
}

// Check if we can change the lane.
bool canLaneChange(const vector<vector<vector<double>>> parameters, const int bestLane, const int curLane, const vector<double> costLane, const double speed)
{
	int newLaneDir;
	if (prev_state == CHANGE_RIGHT || prev_state == PREPARE_CHANGE_RIGHT)
	{
		newLaneDir = 1;
	}
	else if (prev_state == CHANGE_LEFT || prev_state == PREPARE_CHANGE_LEFT)
	{
		newLaneDir = -1;
	}

	// Change the lane if:
	// We are yet to finish previous lane change (intended lane != curLane), and
	// If front car is far away or front car speed is greater our car speed, and
	// if back car is far away or back car speed is less than or car speed.
	if (intendedLaneChange != curLane &&
		costLane[bestLane] < costLane[curLane] - 0.2 &&

		((parameters[curLane + newLaneDir][0][0] > target_min_turn_front_dist) ||
		(parameters[curLane + newLaneDir][0][0] > target_min_turn_front_dist / 2 &&
			speed < parameters[curLane + newLaneDir][0][1])) &&

			((parameters[curLane + newLaneDir][1][0] > target_min_turn_back_dist) ||
		(parameters[curLane + newLaneDir][1][0] > target_min_turn_back_dist / 2 &&
			speed > parameters[curLane + newLaneDir][1][1]))
		)
	{
		return true;
	}
	return false;
}

// Change the Lane
void actionLaneChange(const vector<vector<vector<double>>> &parameters, STATE &newstate, const int &curLane, const int &bestLane, double &target_speed, double &front_distance, int &target_lane)
{
	int newLaneDir;
	if (prev_state == PREPARE_CHANGE_RIGHT || prev_state == CHANGE_RIGHT)
	{
		newstate = CHANGE_RIGHT;
		newLaneDir = 1;
	}
	else
	{
		newLaneDir = -1;
		newstate = CHANGE_LEFT;
	}

	// Set the target speed to max if car is far otherwise set to car in front.
	if (parameters[curLane + newLaneDir][0][0] < target_min_deaccer_front_dist)
	{
		target_speed = parameters[curLane + newLaneDir][0][1];
		front_distance = parameters[curLane + newLaneDir][0][0];
	}
	else {
		target_speed = target_max_speed;
		front_distance = 1000;
	}
	target_lane = curLane + newLaneDir;
	intendedLaneChange = (LANE_NUMBER)target_lane;
	std::cout << "Change the lane to " << (newstate == CHANGE_LEFT ? "LEFT" : "RIGHT") << std::endl;
}

// Plan the next behavior or speed and lane.
// State transition from one state to other, and action of lane change and speed.
vector<double> behaviorPlanning(double ref_s, double ref_d, int curLane, double current_speed, double futureDistance, vector < vector<double>> sensor_fusion)
{
	double target_speed = target_max_speed;
	double front_distance = 1000;
	int target_lane = curLane;

	// Calculate cost of each lane. This help us to select best lane with least cost.
	// Cost will find for closest vehicles in front and less from our car.
	// Set cost to zero if no vehicle ahead or back.
	vector<double> costLane = { 0, 0, 0 };
	vector<vector<vector<double>>> parameters;
	for (int lane = 0; lane < 3; lane++)
	{
		vector<double> closeFront = closestVehicle(ref_s, ref_d, lane, futureDistance, sensor_fusion, true);
		vector<double> closeBack = closestVehicle(ref_s, ref_d, lane, futureDistance, sensor_fusion, false);
		parameters.push_back({ closeFront, closeBack });
		costLane[lane] += ((target_max_speed - closeFront[1]) / target_max_speed);
		costLane[lane] += (1 - exp(-(target_min_deaccer_front_dist / closeFront[0])));
	}

	std::cout << costLane[0] << " " << costLane[1] << " " << costLane[2] << std::endl;

	// State transition.
	STATE state = KEEP_LANE;

	// Find best lane based on cost, however, it may not feasible to change due to invalid states or other cars too close to it.
	int bestLane = std::min_element(costLane.begin(), costLane.end()) - costLane.begin();

	// If previously car was in keep lane state.
	if (prev_state == KEEP_LANE) {
		// Change lane only if require.
		if (checkKeepLane(parameters, bestLane, curLane))
		{
			// Get best parameters.
			actionKeepLane(parameters, state, curLane, target_speed, front_distance, target_lane);
		}
		else
		{
			// Better lane available. Let's prepare by setting target lane.
			actionPrepareLane(parameters, state, curLane, bestLane, target_speed, front_distance, target_lane);
		}
	}
	// If we were in prepare state.
	else if (prev_state == PREPARE_CHANGE_LEFT || prev_state == PREPARE_CHANGE_RIGHT)
	{
		// If we don't want to change the lane now, as we are too close or far from front car.
		// or, we are best lane now.
		if (checkKeepLane(parameters, bestLane, curLane)) 
		{
			actionKeepLane(parameters, state, curLane, target_speed, front_distance, target_lane);
		}
		// Check if we can change the lane now.
		else if (canLaneChange(parameters, bestLane, curLane, costLane, current_speed))
		{
			actionLaneChange(parameters, state, curLane, bestLane, target_speed, front_distance, target_lane);
		}
		else
		{
			// Wait in prepare state. // Match the target speed in adjacent lane.
			actionPrepareLane(parameters, state, curLane, bestLane, target_speed, front_distance, target_lane);
		}
	}
	// if we are changing the lane.
	else {
		// Continue to check if we have finsihed the lane change.
		if (canLaneChange(parameters, bestLane, curLane, costLane, current_speed))
		{
			actionLaneChange(parameters, state, curLane, bestLane, target_speed, front_distance, target_lane);
		}
		else
		{
			// Change state to keep lane once lane change is finish.
			actionKeepLane(parameters, state, curLane, target_speed, front_distance, target_lane);
		}
	}
	prev_state = state;
	std::cout << "State: " << prev_state << " -> " << state << std::endl;
	std::cout << "Target Lane " << target_lane << " target speed" << target_speed << " front_distance " << front_distance << std::endl;
	return { (double)target_lane, target_speed, front_distance};
}

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
  double ref_v = 0.0;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_v]
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

			
			double prev_size = previous_path_x.size();
			vector<double> next_x_vals;
			vector<double> next_y_vals;

			// Project code start here 
			// Sensor Fusion to avoid collision.
			if (prev_size > 0)
			{
				car_s = end_path_s;
				//car_d = end_path_d;
			}

			// GEt the best lane and target speed.
			int lane = getLaneNumber(car_d);
			if (lane == INVALID_LANE) lane = 1;
			vector<double> behaviorResult = behaviorPlanning(car_s, car_d, lane, ref_v, prev_size, sensor_fusion);
			lane = (int)behaviorResult[0];
			
			// Set the velocity based on target speed and acceleration.
			// TODO: Try to smooth it in future by varying accleration.
			if (ref_v < behaviorResult[1])
			{
				ref_v += target_max_acceleration;
			}
			else if (ref_v > behaviorResult[1])
			{
				ref_v -= target_max_acceleration;
			}
			if (ref_v > target_max_speed) ref_v = target_max_speed;
			if (ref_v < 0.5 || behaviorResult[2] < target_critical_front_dist) ref_v = 0.5;
			std::cout << "Current Lane: " << lane << " speed: " << ref_v<<std::endl;
			std::cout<< "**********************************" << std::endl;
			
			/* Generate trajectories */ 
			vector<double> ptsx;
			vector<double> ptsy;
			
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			
			if  (prev_size < 2)
			{
				// we don't have enough previous point. Use current car position and make path tangent to car;
				double prev_car_x = car_x  - cos (car_yaw);
				double prev_car_y = car_y  - sin (car_yaw);
				ptsx.push_back(prev_car_x);
				ptsy.push_back(prev_car_y);		
				ptsx.push_back(car_x);
				ptsy.push_back(car_y);
			}
			else
			{
				// We have previous path. Take couple of points from previous path. This help in smooth trajectories and avoid jerk.
				// Update reference point as well.
				ref_x  = previous_path_x[prev_size-1];
				ref_y  = previous_path_y[prev_size-1];
				
				double ref_x_prev =  previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);
				ptsx.push_back(ref_x_prev);
				ptsy.push_back(ref_y_prev);
				
				ptsx.push_back(ref_x);
				ptsy.push_back(ref_y);
			}

			// Get the waypoint at 30, 60 and 90 mt.  This are dest points.
			vector<double> wp1 = getXY(car_s+30, (2+lane * 4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> wp2 = getXY(car_s+60, (2+lane * 4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> wp3 = getXY(car_s+90, (2+lane * 4), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(wp1[0]);
			ptsy.push_back(wp1[1]);
			ptsx.push_back(wp2[0]);
			ptsy.push_back(wp2[1]);
			ptsx.push_back(wp3[0]);
			ptsy.push_back(wp3[1]);
			
			// Transfer to car coordinate
			for (int i =0; i < 	ptsx.size(); i++)
			{
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;
				
				ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
				ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
				
			}
			tk::spline s;
			// set the set of anchor points in spine
			s.set_points(ptsx, ptsy);
			
			// Add previous left over point. This avoid sudden jerk due to previous and next trajectories.
			for (int i=0; i < previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}


			// Find remaining new points.
			double target_x = 30;
			double target_y = s(target_x);
			double target_dist = sqrt(target_x*target_x + target_y*target_y);
			
			double x_acc = 0.0;

			for (int i=0; i < (50-previous_path_x.size()); i++)
			{
                // Calculate points along new path
                double N = (target_dist/(.02*ref_v/2.24));
                double x_point = x_acc+(target_x)/N;
                double y_point = s(x_point);
				x_acc = x_point;
				double x_temp = x_point;
				double y_temp = y_point;
				// Rotate and shift back to normal
				x_point = (x_temp*cos(ref_yaw)-y_temp*sin(ref_yaw));
				y_point = (x_temp*sin(ref_yaw)+y_temp*cos(ref_yaw));
				
				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}
			// Project code end here 
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