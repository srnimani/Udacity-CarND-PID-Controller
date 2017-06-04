#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Additional variables for twiddle
double p[3]                 = {0.0, 0.0, 0.0} ; // Array containing tau_p, tau_i, tau_d
char   twid                 = 'n'; // User input to decide whether to use twiddle or not
double current_steer        = 0.0;
int    twiddle_run          = 0 ; //first time twiddle
double dp[3]                = {1.0, 1.0, 1.0} ;
double tw_err               = 0.0 ; // Initial value
double tol                  = 0.0001 ;
double best_err             = 1000000 ;
double sum                  = 0.0 ;


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


void twiddle(PID pid, double cte)
// Twiddle algorithm
{
    //if (twiddle_run < 100) // twiddle loop run for a few runs before deciding on the hyper params
    {
        p[0] = 0.3    ; // Kp Initializing to 0 does not help
        p[1] = 0.003  ; // Ki
        p[2] = 4.0    ; // Kd

        pid.Init(p[0], p[1], p[2]);
        
        dp[0]   = 1.0   ;
        dp[1]   = 1.0   ;
        dp[2]   = 1.0   ;
        
        best_err = 10000000 ; // Initialized to a large value
     
        //twiddle_run += 1;
    }
    
    sum      = dp[0] + dp[1] + dp[2] ;
    
    //int iter = 0;

    while (sum > tol)
    {
        for (int i = 0; i < 3; i++)
        {
            // Twiddle Up
            p[i] += dp[i];
            pid.Kp = p[0];
            pid.Ki = p[1];
            pid.Kd = p[2];
            tw_err = abs(pid.TotalError()); // Take the absolute value of error
            
            if (tw_err < best_err)
            {
                best_err = tw_err;
                dp[i]    *= 1.1;
            }
            else
            {
                // Twiddle Down
                p[i]   -= 2*dp[i];
                pid.Kp = p[0];
                pid.Ki = p[1];
                pid.Kd = p[2];
                tw_err = abs(pid.TotalError()); // Take the absolute value of error
                
                if (tw_err < best_err)
                {
                    best_err = tw_err;
                    dp[i]    *= 1.1;
                }
                else
                {
                    p[i]   +=dp[i];
                    pid.Kp = p[0];
                    pid.Ki = p[1];
                    pid.Kd = p[2];
                    dp[i]  *= 0.9;
                }
                //cout << "i :\t" << i << "\ttw_err: \t" << tw_err << "\tbest_err: \t" << best_err << endl;
            }
        }
        sum = dp[0] + dp[1] + dp[2] ;
        //iter += 1;
    }
}


// Discrete speed control function to modulate the speed based on the streeing value

double discrete_speed_control(double steering_angle)
{
    double throttle_value = 0.1;
    
    if (abs(steering_angle) >= 0.9)
    {
        throttle_value = 0.1; // Slow down as the curve is sharp
    }
    else if ((abs(steering_angle) <= 0.9) && (abs(steering_angle) > 0.8))
    {
        throttle_value = 0.15;
    }
    else if ((abs(steering_angle) <= 0.8) && (abs(steering_angle) > 0.7))
    {
        throttle_value = 0.2;
    }
    else if ((abs(steering_angle) <= 0.7) && (abs(steering_angle) > 0.6))
    {
        throttle_value = 0.25;
    }
    else if ((abs(steering_angle) <= 0.6) && (abs(steering_angle) > 0.5))
    {
        throttle_value = 0.3;
    }
    else if ((abs(steering_angle) <= 0.5) && (abs(steering_angle) > 0.4))
    {
        throttle_value = 0.35;
    }
    else if ((abs(steering_angle) <= 0.4) && (abs(steering_angle) > 0.2))
    {
        throttle_value = 0.4;
    }
    else if ((abs(steering_angle) <= 0.2) && (abs(steering_angle) > 0.1))
    {
        throttle_value = 0.45;
    }
    else
    {
        throttle_value = 0.5; // Full speed
    }
    return throttle_value ;
}


// Check for steering out of bouns.. Steering should be between +1 and -1

double steering_check(double current_steer)
{
    
    if (current_steer > 1.0)
    {
        current_steer = 1.0;
    }
    if (current_steer < -1.0)
    {
        current_steer = -1.0;
    }
    
    return current_steer ;
}


int main()
{
  uWS::Hub h;

  PID pid;
    
  // TODO: Initialize the pid variable.
    
  // Initialize Kp, Ki and Kd for non-twiddle case
    
  p[0] = 0.3    ; // Kp
  p[1] = 0.002  ; // Ki
  p[2] = 3.0    ; // Kd
    
  pid.Init(p[0], p[1], p[2]);

  twiddle_run = 0 ;
    
  // Check if twiddle needed
    
  cout << "Twiddle Hyperparams? y/n " ;
  cin >> twid;
    
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          pid.UpdateError(cte);
            
          if (twid == 'y')
          {
              twiddle(pid, cte); // Twiddle the parameters to get the best values
          }
            
          //cout << "Kp =\t" << p[0] << "\tKi =\t" << p[1] << "\tKd =\t" << p[2] << endl;

          current_steer = -pid.TotalError();
      
          steer_value = steering_check(current_steer) ; //
            
          // DEBUG
          //cout << "CTE: " << cte << " Steering Value: " << steer_value << endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
            
          // Speed control based on steering angle
            
          double throttle_value = discrete_speed_control(steer_value);
          msgJson["throttle"]   = throttle_value; //
            
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
