#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void resetSim(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
/*void resetErrors(double &err ){

    err=0;
}
*/


int main(int argc, char *argv[]) {
  uWS::Hub h;
  bool twiddle=false;

  std::cout << "There are " << argc << " arguments:\n";
  if((argc>1)&& (std::string(argv[1])=="twiddle")  ){
      twiddle= true; //by default
  }
  

  PID pid;
//  bool twiddle = false; //true;
  double p[3] = {0.2, 0.0004, 3.0};
  double dp[3] = {.1, .0001, .1};
  bool first_time=true;
  int n=0;
  int max_n=100;
  //double elerror;
  double best_error;
  
  /**pid.Init(p[0],p[1],p[2]);
   * TODO: Initialize the pid variable.
   */
//  pid.Init(0.2,0.004,3.0);  // For starters let's begin with this does not work
//  pid.Init(0.2,0,0); 
//  pid.Init(0.2,0,3.0); 
  if(twiddle==true){
    pid.Init(p[0],p[1],p[2]);
  }
  else{
	pid.Init(0.2,0.0004,3.0); // For some reason this works to some extend but it does not improve PD  
  }
  std::cout<<"Twiddle: "<<twiddle<<std::endl;

  h.onMessage([&pid, &p, &dp ,&twiddle, &first_time,&n, &max_n, &best_error]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());  //Cross Track Error
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
		  json msgJson;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
      
	      if(twiddle){
			  
            if(first_time){  //Run the robot once before twiddle looping
               if(n==0){
                   pid.Init(p[0],p[1],p[2]);  //reset
               }
               n++;
               
               pid.UpdateError(cte);  //update the errors
               steer_value = pid.GetResult();
               if(steer_value>1) steer_value=1;
               if(steer_value<-1) steer_value=-1;
               msgJson["steering_angle"] = steer_value;
               msgJson["throttle"] = 0.3;
               auto msg = "42[\"steer\"," + msgJson.dump() + "]";
               //std::cout << msg << std::endl;
               std::cout<<n<<" ";
               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
               if(n>=max_n){  //start calculating error
                    double elerror= pid.TotalError();  //here it is acumulating the error
                    std::cout<<"el error: "<<elerror<<std::endl;
                    //elerror= elerror/(max_n);
                    if(n>=(2*max_n)){
                        //first_time=false;
                        best_error= elerror/(max_n); 
                        std::cout<<"Best Error: "<<best_error<<std::endl;
                        resetSim(ws);  //Reset the simulation
                        twiddle=false;
                    }
               }
            }//first time end


          }
		  else{  //Not twiddle
          // We have a steering angle that is the one that we have to control
          // (it is the equivalent to y_trajectory in the python)
          // We have a CTE that we want to make 0 with PID
          // that is equivalent to robot.y in the python lessons
          if(first_time){
              pid.Reset();
              first_time=false;
          }

          pid.UpdateError(cte);  //update the errors
          steer_value = pid.GetResult();
          if(steer_value>1) steer_value=1;
          if(steer_value<-1) steer_value=-1;




          // DEBUG
          std::cout << "Speed: "<< speed << " angle: "<<angle<<std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          //json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		  } //not twiddle
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
    std::cout << "Listening to port bubu " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}