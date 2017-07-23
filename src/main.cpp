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

std::vector<double> p = {-0.135, 0.0 , -3.19};
std::vector<double> dp = {1,.1,1};
double best_err = 100000;
double tolerance = 0.1;

int twiddleStep = 0;
int twiddlePIndex = 0;


std::vector<double> twiddle( double calcErr){

  

  std::cout << "BEGIN TWIDDLE" << p[0] << " " << p[1] << std::endl;
    double err = calcErr;

    double newTwiddlePart = 1;
    int i = twiddlePIndex;
    if( twiddleStep == 0 ){
      std::cout << "PART 0" << std::endl;
      p[i] += dp[i];
      newTwiddlePart = 1;
      best_err = err;
    }
    else if( twiddleStep == 1){
      std::cout << "PART 1" << std::endl;
      if(err < best_err){
        std::cout << "PART 1 ERR LESS" << std::endl;
        best_err = err;
        dp[i] *= 1.1;
        if(twiddlePIndex < 2){
          twiddlePIndex += 1;
        }
        else{
          twiddlePIndex = 0;
        }
        newTwiddlePart = 0;
      }
      else{
        std::cout << "PART 1 ERR MORE SUBTRACT" << std::endl;
        p[i] -= 2 * dp[i];
        newTwiddlePart = 2;
      }
    }
    else{
      std::cout << "PART 2" << std::endl;
      if( err < best_err ){
        best_err = err;
        dp[i] *= 1.1;
      }
      else{
        p[i] += dp[i];
        dp[i] *= 0.9;
      }        
      if(twiddlePIndex < 2){
          twiddlePIndex += 1;
      }
      else{
        twiddlePIndex = 0;
      }
      newTwiddlePart = 0;
      
    }  
    

    twiddleStep = newTwiddlePart;
    
    

    
    
    // while ( (dp[0] + dp[1] + dp[2]) > tolerance) {
    //   std::cout << "HIGHER THAN TOLERANCE" << std::endl;
    //   for( int i = 0; i < p.size(); i++){
    //     p[i] += dp[i];
    //     err = twiddleRun(p);
    //     if(err < best_err){
    //       best_err = err;
    //       dp[i] *= 1.1;
    //     }
    //     else{
    //       p[i] -= 2* dp[i];
    //       err = twiddleRun(p);
    //       if( err < best_err ){
    //         best_err = err;
    //         dp[i] *= 1.1;
    //       }
    //       else{
    //         p[i] += dp[i];
    //         dp[i] *= 0.9;
    //       }
    //     }

    //   }
    // }
    std::cout << "DONE TWIDDLING" << std::endl;
    return p;
}
  



double twiddleRun(){
  uWS::Hub h;
  PID pid;
  // std::vector<double> p = {0,0,0};
  // std::vector<double> dp = {1,1,1};
  std::cout << "p in initial" << p[0] << "p1 in initial" << p[1] << p[2] << std::endl;
  pid.Init(p[0], p[1], p[2]);
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
    // std::cout << "TWIDDLE RUN in websocket" << std::endl;
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
        auto s = hasData(std::string(data).substr(0, length));
        if (s != "") {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {

            // double twiddle_it = j[1].value("twiddle_it", 0.0);
            double twiddle_it = pid.TwiddleIteration();

            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());

            // double calcErr = 0;
            // if(j.find("calcError") != j.end()){
            //   calcErr = std::stod(j[1]["calcErr"].get<std::string>());
            //   calcErr += (cte*cte);
            // }
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
            steer_value = pid.TotalError();

            double calcErr = 0;

            // calc the error between steps 100 and 200
            if( twiddle_it > 100 ){
              calcErr = pid.CalcError(cte);
            }
            if( twiddle_it > 300 ){
              std::cout << "RETURN CALCERR" << calcErr << std::endl;
              // reset simulator to tune params between iterations because often goes off too far and gets stuck
              std::string msg = "42[\"reset\", {}]";
              std::cout << msg << std::endl;
              twiddle_it = 0;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              // returnErr(calcErr);
              // return calcErr;
              std::vector<double>tp = twiddle(calcErr);
              
              // when reached tolerance level stop tuning params
              if( dp[0] + dp[1] + dp[2] > tolerance){
                std::cout << "NEW P1 " << tp[0] << "NEW P2 " << tp[1] << "NEW P3 " << tp[2] << std::endl;
                pid.Init(tp[0], tp[1], tp[2]);
              }
              else{
                std::cout << "FINAL P1 " << tp[0] << "FINAL P2 " << tp[1] << "FINAL P3 " << tp[2] << std::endl;
                // ws.close();
              }
              
            }
            
            // DEBUG
            // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            // msgJson["twiddle_it"] = twiddle_it;
            // msgJson["calcErr"] = calcErr;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    });
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

  std::cout << "RETURN FINAL" << std::endl;
  double calcErr = pid.CalcError(0);
  return calcErr;
}





int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.


  // first ran twiddle run to tune params, then ran with output params
// commented out for nontuning run
  // double err = twiddleRun();
  // std::vector<double> p= twiddle();
  pid.Init(p[0], p[1], p[2]);

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
          steer_value = pid.TotalError();
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
