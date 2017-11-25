#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

#define LOOP_NUM 100
#define SPEED_TARGET 40.0
#define THROTTLE_LIMIT 10000.0
#define PID_CONDITION 0.002
#define PID_T_CONDITION 0.002
#define SLOWDOWN_FOR_SAFE

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  // It is suggested that not TWIDDLE two controller at the same time
  // If TWIDDLE is desired, set the last argument as "true"
  //pid.Init(0.05, 0.00001, 2.0, true);
  /*
  * The Kp, Kd are the result of TWIDDLE
  * Ki is setting manually
  */
  pid.Init(0.109, 0.0001, 0.522959, false);
  //pid.Init(0.109, 0.0001, 50.0, false);

  PID pid_t;
  //pid_t.Init(1.0, 0.0, 0.0, true);
   /*
  * The Kp, Kd are the result of TWIDDLE
  * Ki is setting manually
  */
  pid_t.Init(1.0, 0.0, -1.23978, false);
  //pid_t.Init(0.1, 0.0, 0.0, false);

  //const int loop_num = 300;
  //pid.count = 0;
  //const int speed_target = 40;


  h.onMessage([&pid, &pid_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // The following code is to twiddle controller for handling first,
          // and then twiddle the controller for throttle.
          // But such method has bug, and needs to be modified.
          if (pid.twiddle_done) {
            pid_t.twiddle_on = true;
            pid.twiddle_done = false;
          }
          ///////////////////////////////////////////////////////////////////

          /*
          * The following are for handling controller TWIDDLE
          */
          pid.UpdateError(cte);
          pid.Twiddle(PID_CONDITION);
          pid.Goodness(cte, LOOP_NUM);
          
          /*
          * For speeding down when cte getting larger
          * That's only experiment, to prevent lossing control
          */
          #ifdef SLOWDOWN_FOR_SAFE
            double ratio = 2.0*(1/(1 + exp(-cte*cte)) - 0.5);
            double speed_t = SPEED_TARGET*(1.0 - ratio);
            double speed_cte = speed - speed_t;
            std::cout << "Ratio = " << ratio << std::endl;
            std::cout << "Speed Target = " << speed_t << std::endl;
          #else
            double speed_cte = speed - SPEED_TARGET;
          #endif
          /*
          * The following are for throttle controller TWIDDLE
          */         
          pid_t.UpdateError(speed_cte);
          pid_t.Twiddle(PID_T_CONDITION);
          pid_t.Goodness(speed_cte, LOOP_NUM);
          double throttle = pid_t.TotalError();
          if (throttle > THROTTLE_LIMIT) {
            throttle = THROTTLE_LIMIT;
          } else if (throttle < 0.0) {
            throttle = 0.0;
          }
          
          std::cout << "H:p[0] = " << pid.p[0] << " H:dp[0] = " << pid.dp[0] << std::endl;
          std::cout << "H:p[1] = " << pid.p[1] << " H:dp[1] = " << pid.dp[1] << std::endl;
          std::cout << "H:p[2] = " << pid.p[2] << " H:dp[2] = " << pid.dp[2] << std::endl;
          std::cout << "H:index = " << pid.update_index << std::endl;
          std::cout << "H:state = " << pid.state << std::endl;

          std::cout << "T:p[0] = " << pid_t.p[0] << " T:dp[0] = " << pid_t.dp[0] << std::endl;
          std::cout << "T:p[1] = " << pid_t.p[1] << " T:dp[1] = " << pid_t.dp[1] << std::endl;
          std::cout << "T:p[2] = " << pid_t.p[2] << " T:dp[2] = " << pid_t.dp[2] << std::endl;
          std::cout << "T:index = " << pid_t.update_index << std::endl;
          std::cout << "T:state = " << pid_t.state << std::endl;
          steer_value = pid.TotalError();
          if (steer_value > 1.0) {
            steer_value = 1.0;
          } else if (steer_value < -1.0) {
            steer_value = -1.0;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Speed_CTE: " << speed_cte << " Throttle: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (pid.reset_trig || pid_t.reset_trig) {
            pid.reset_trig = false;
            pid_t.reset_trig = false;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
