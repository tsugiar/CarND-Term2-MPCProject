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

  // Initialize main function variables here
  double px;
  double py;
  double psi;
  double v;

  double prev_delta = 0;
  double prev_accel = 0;
  double prev_psi = 0;
  double est_roc = 0;
  double yaw_rate_est = 0;
  double feed_fwd_steer = 0;


  double dx, dy;  // Distance from way-point to host vehicle

  double cte;
  double epsi;
  double steer_value;
  double throttle_value;
  Eigen::VectorXd state(6);
  Eigen::VectorXd old_coeffs(4);


  old_coeffs <<0 , 0, 0, 0;

  // Initialize waypoint, assume that it consists
  // of 6 elements
  size_t  waypoint_vector_size = 6;
  Eigen::VectorXd ptsx_eigen(waypoint_vector_size);
  Eigen::VectorXd ptsy_eigen(waypoint_vector_size);


  vector<double> ptsx;
  vector<double> ptsy;

  // Reserver space for ptsx and ptsy, initially set it to 20
  ptsx.reserve(20);
  ptsy.reserve(20);

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &px, &py, &psi, &v, &cte,
              &epsi, &ptsx, &ptsy, &ptsx_eigen, &ptsy_eigen,
              &waypoint_vector_size,
              &steer_value, &throttle_value, &state, &dx, &dy,
              &prev_delta, &prev_accel, &est_roc, &yaw_rate_est, &feed_fwd_steer, &prev_psi]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
  //  cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {


          // Obtain host-position, heading, and speed
          px = j[1]["x"];
          py = j[1]["y"];
          psi = j[1]["psi"];
          v =  static_cast<double>(j[1]["speed"]) *  0.44704;   // Obtain velocity and convert it from mph to meters/sec


          // Obtain ptsx and ptsy vector
          ptsx.clear();
          ptsy.clear();
          ptsx = j[1]["ptsx"].get<std::vector<double>>();
          ptsy = j[1]["ptsy"].get<std::vector<double>>();

          // Change size of ptsx_eigen and ptsy_eigen if it isn't the
          // same as previous value
          if (waypoint_vector_size != ptsx.size())
          {
              waypoint_vector_size = ptsx.size();
                ptsx_eigen.resize(waypoint_vector_size);
                ptsy_eigen.resize(waypoint_vector_size);
          }

          // Looping statement to file out ptsx_eigen and ptsy_eigen with data
          // from ptsx and ptsy
          for (unsigned int iter_ind = 0; iter_ind < ptsx.size(); ++iter_ind)
          {
              // Transformation of way-point vector to vehicle coord system
              dx = ptsx[iter_ind] - px;
              dy = ptsy[iter_ind] - py;

              ptsx_eigen[iter_ind] =  dx * cos(psi) + dy * sin(psi);
              ptsy_eigen[iter_ind] = -dx * sin(psi) + dy * cos(psi);
          }


          std::cout << std::endl;
          std::cout << "Corresponding (hostx,hosty) value is =("
                    << px << "," << py << ")" << std::endl;
          std::cout << "At speed : " << v << std::endl;
          std::cout << std::endl;

          //==================  END OF DEBUG PRINTOUT ==============================
          //========================================================================

          // Polynomial fit (third order)
          auto coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);

          // Estimate roc from lane-poly
          est_roc = 1/(2*coeffs[2]);

          cte = polyeval(coeffs,0);     // Compute lateral distance error, at 0 point from host
                                        // coordinate system
          epsi = -atan(coeffs[1]);           // Compute heading angle error

          // Estimate yaw-rate

          if (v > 0.1)
          {
          yaw_rate_est = (psi - prev_psi)/100e-3;   // Estimate yaw-rate
          feed_fwd_steer = asin(yaw_rate_est/est_roc / v);
           }
          else
          {
              feed_fwd_steer = 0;
          }


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Update state values
          state[0] = px * 0 ;
          state[1] = py * 0 ;
          state[2] = psi * 0 ;
          state[3] = v ;
          state[4] = cte;
          state[5] = epsi;

          // Compute mpt
          auto solution_vec = mpc.Solve(state, coeffs, prev_delta, prev_accel);
          steer_value       = solution_vec[0] + feed_fwd_steer;
          throttle_value    = solution_vec[1];


          std::cout << "Steering value is " << steer_value/M_PI * 180 << "\t" << "Cte value is : " << cte << std::endl;
          std::cout << "Feed forward steering value : " << feed_fwd_steer/M_PI * 180 << std::endl;
          std::cout << "Throttle value is " << throttle_value << "\t" << "Speed value is " << v  << std::endl;
          std::cout << "CTE val " << cte << std::endl;
          std::cout << "epsi val : " << "(" << epsi/M_PI*180 << std::endl;
          std::cout << std::endl;


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Store previous steering value
          prev_delta = steer_value;
          prev_accel = throttle_value;
          prev_psi = psi;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // Determine size of x,y values from solution vector
          size_t soln_vec_xysize = (solution_vec.size()-2);
          for (unsigned int iter_idx = 0 ; iter_idx < soln_vec_xysize; iter_idx=iter_idx+2)
          {
              mpc_x_vals.push_back(solution_vec[2+iter_idx]);
              mpc_y_vals.push_back(solution_vec[2+iter_idx+1]);


          }


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (unsigned int i=0 ; i < 100; ++i)
          {
              next_x_vals.push_back(static_cast<double>(i));
              next_y_vals.push_back(polyeval(coeffs,i));

          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
     //     std::cout << msg << std::endl;
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
