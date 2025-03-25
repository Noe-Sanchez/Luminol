#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <math.h>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "httplib.h"

// Global thread, for death after ROS shutdown
pthread_t server_thread;

// Proto for server thread
void *run_server(void *args);

using namespace std::chrono_literals;

class LuminolBridge : public rclcpp::Node{
  public:
    LuminolBridge(): Node("luminol_bridge"){
      this->declare_parameter("num_drones", 12);
      num_drones = this->get_parameter("num_drones").as_int();

      this->declare_parameter("http_port", 19001);
      http_port = this->get_parameter("http_port").as_int();

      // Make 0.1s timer
      stream_timer = this->create_wall_timer(100ms, std::bind(&LuminolBridge::stream_callback, this));


      std::cout << "Luminol Bridge initialized at " << http_port << std::endl;

      for (int i = 0; i < num_drones; i++){
	std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> follower_proto = std::bind(&LuminolBridge::follower_pose_callback, this, std::placeholders::_1, i);
	follower_pose_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/estimator/pose", 10, follower_proto));

	follower_poses.push_back(geometry_msgs::msg::Pose());
      }

      server_thread = pthread_t();

      // THIS MIGHT BREAK (it did not lol)
      server.Get("/stream", [&](const httplib::Request &req, httplib::Response &res){
	  // Return poses in json
	  std::string json = "{";
	  for (int i = 0; i < num_drones; i++){
	    json += "\"tello_" + std::to_string(i) + "\": {";
	    json += "\"x\": " + std::to_string(follower_poses[i].position.x) + ",";
	    json += "\"y\": " + std::to_string(follower_poses[i].position.y) + ",";
	    json += "\"z\": " + std::to_string(follower_poses[i].position.z) + "}";
	    if (i != num_drones - 1){
	      json += ",";
	    }
	  }
	  json += "}";
	  res.set_content(json, "application/json");
      });

      pthread_create(&server_thread, NULL, run_server, &server);
      
    }
    
    void follower_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int i){
      follower_poses[i] = msg->pose;
    }

    void stream_callback(){
      std::cout << "Stream callback" << std::endl; 
    }

  private: 

    int num_drones; 
    int http_port; 
    
    httplib::Server server;

    rclcpp::TimerBase::SharedPtr stream_timer;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> follower_pose_subscriptions;
    
    std::vector<geometry_msgs::msg::Pose> follower_poses;

};

void *run_server(void *args){
  httplib::Server *server = (httplib::Server *)args;
  server->listen("0.0.0.0", 19001);
  return NULL;
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LuminolBridge>());
  rclcpp::shutdown();
  
  pthread_cancel(server_thread);

  return 0;
}
