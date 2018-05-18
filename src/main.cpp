#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
/*
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;
 */

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
            // The below waypoints are in the map's co-ordinate system
            // They need to be transformed to the vehicle's co-ordinate system
            // based on the current position and orientation of the vehicle
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double previous_a = j[1]["throttle"];
          double previous_delta = j[1]["steering_angle"];
            
            //Display the waypoints/reference line
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            // Create an Eigen matrix to store the x and y waypoints
            cout << "pts x size = " << ptsx.size() << endl;
            auto vehicle_waypoints = Eigen::MatrixXd(2, ptsx.size());
            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Yellow line
            for(size_t i = 0; i < ptsx.size(); ++i){
                // Convert pts from map co-ordinate system to vehicle co-ordinate system
                vehicle_waypoints(0, i) = (ptsx[i] - px)*cos(psi) + (ptsy[i] - py)*sin(psi);
                vehicle_waypoints(1, i) = (ptsy[i] - py)*cos(psi) - (ptsx[i] - px)*sin(psi);
                next_x_vals.push_back(vehicle_waypoints(0, i));
                next_y_vals.push_back(vehicle_waypoints(1, i));
                // Great visualization of the transformation between map and vehicle co-ordinate frames
                // at this link : https://discourse-cdn-sjc3.com/udacity/uploads/default/original/4X/3/0/f/30f3d149c4365d9c395ed6103ecf993038b3d318.png
                
            }

          // Calculate the coefficient based on the order
            int order = 3;
            
            Eigen::VectorXd pts_vecx = vehicle_waypoints.row(0);
            Eigen::VectorXd pts_vecy = vehicle_waypoints.row(1);
            auto coeffs = polyfit(pts_vecx, pts_vecy, order);
            // y = ax^2 + bx + c;
         
         // Calculate the cross track and orientation errors
            double cte = polyeval(coeffs, 0);
            std::cout << "CTE = " << cte << std::endl;
            double epsi = -atan(coeffs[1]);
            std::cout << "EPSI = " << epsi << std::endl;
          
        // Incorporate latency, by predicting what the state will be after the delay
            // We will provide that as the initial state to the mpc solver.
            // In the frame of the vehicle px, py and psi are 0
            // So use those values for incorporating latency;
            double x, y, psi_n;
            x = 0; y = 0; psi_n = 0;
            // Incorporate latency
            double dt = 0.1;
            const double Lf = 2.67;
            x += v * cos(psi_n) * dt;
            y += v * sin(psi_n) * dt;
            psi_n -= v * previous_delta / Lf * dt;
            v = v + previous_a * dt;
            cte += (v * sin(epsi) * dt);
            epsi -= v * previous_delta / Lf * dt;

        // Initialize the state to send to the MPC solver
            Eigen::VectorXd state(6);
            state << x, y, psi_n, v, cte, epsi;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          auto vars = mpc.Solve(state, coeffs);
          double steer_value;
          double throttle_value;
          steer_value = vars[0];
          throttle_value = vars[1];
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
            steer_value = steer_value / deg2rad(25);
          msgJson["steering_angle"] = -1 * steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          //vector<double> mpc_x_vals;
          //vector<double> mpc_y_vals;
          /*size_t N = 25;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
            for(size_t i = 1; i < N; ++i){
                double mpc_x = vars[i+1];
                double mpc_y = vars[i+N+1];
                mpc_x_vals.push_back(mpc_x);
                mpc_y_vals.push_back(mpc_y);
            }
           */
          

          msgJson["mpc_x"] = mpc.mpc_x_vals;
          msgJson["mpc_y"] = mpc.mpc_y_vals;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
