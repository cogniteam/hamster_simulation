/*
* teleop_pr2
* Copyright (c) 2009, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the <ORGANIZATION> nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

//\author: Blaise Gassend

#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread.hpp>
#include <linux/hid.h>



///\brief Opens, reads from and publishes joystick events
class Joystick
{
private:
 ros::NodeHandle nh_;
 bool open_;
 std::string joy_dev_;
 double deadzone_;
 double autorepeat_rate_;  // in Hz.  0 for no repeat.
 double coalesce_interval_; // Defaults to 100 Hz rate limit.
 int event_count_;
 int pub_count_;
 double lastDiagTime_;

 bool allow_pub_;
 double a_scale_;
 double l_scale_;
 ros::Publisher velocity_publisher_;
 ros::Publisher twist_publisher_;
 ros::Timer velocity_publish_timer_;
 sensor_msgs::Joy current_joy_message_;
 sensor_msgs::Joy last_joy_message_;
 boost::mutex allow_pub_mutex_;

 void publish_velocity() {
   ROS_INFO("Publishing...");


   while (ros::ok()) {
     geometry_msgs::Twist vel;
     ackermann_msgs::AckermannDriveStamped ack;

     double leftTrigger = (2.0 - (current_joy_message_.axes[2] + 1)) / 2.0;
     double rightTrigger = (2.0 - (current_joy_message_.axes[5] + 1)) / 2.0;

     ack.drive.speed = l_scale_ * (-leftTrigger + rightTrigger); // [-1..+1]
     ack.drive.steering_angle = a_scale_ * (current_joy_message_.axes[3] * M_PI_4);

    //  ack.drive.steering_angle = fmax(-M_PI_4,fmin(M_PI_4, ack.drive.steering_angle));

     vel.linear.x = ack.drive.speed * 20;
     vel.angular.z = ack.drive.steering_angle * 5;
//      vel.linear.x = 2 * l_scale_ * current_joy_message_.axes[4];

     if(allow_pub_)
     {
		 allow_pub_mutex_.lock();
		 // allow_pub_ = false;
		 allow_pub_mutex_.unlock();
		 twist_publisher_.publish(vel);
		 velocity_publisher_.publish(ack);
     }


     boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0 / 30.0));

//      std::cout << current_joy_message_ << std::endl;

//      std::cout << leftTrigger << ", " << rightTrigger << std::endl;
   }

 }

 // void publish_velocity(const ros::TimerEvent& event) {
 //   ROS_INFO("Publishing...");
	 //  geometry_msgs::Twist vel;
	 //  vel.angular.z = a_scale_ * current_joy_message_.axes[3];
	 //  vel.linear.x = l_scale_ * current_joy_message_.axes[4];
	 //  velocity_publisher_.publish(vel);
 // }

public:
 Joystick() : nh_(),allow_pub_(false) {}

 ///\brief Opens joystick port, reads from port and publishes while node is active
 int main(int argc, char **argv)
 {
//    diagnostic_.add("Joystick Driver Status", this, &Joystick::diagnostics);
//    diagnostic_.setHardwareID("none");

   // Parameters
   ros::NodeHandle nh_param("~");

   current_joy_message_.axes.push_back(0);
   current_joy_message_.axes.push_back(0);
   current_joy_message_.axes.push_back(0);
   current_joy_message_.axes.push_back(0);
   current_joy_message_.axes.push_back(0);

   std::string pub_topic;
   nh_param.param<std::string>("topic", pub_topic, "joy");

   ROS_INFO("Starting joystick publisher...");

   // pub_ = nh_.advertise<sensor_msgs::Joy>(pub_topic, 1);
   velocity_publisher_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1, false);
   twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
   // velocity_publish_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Joystick::publish_velocity, this, _1));

   nh_param.param<std::string>("dev", joy_dev_, "/dev/input/js0");
   nh_param.param<double>("deadzone", deadzone_, 0.05);
   nh_param.param<double>("autorepeat_rate", autorepeat_rate_, 0);
   nh_param.param<double>("coalesce_interval", coalesce_interval_, 0.001);

   nh_param.param("scale_angular", a_scale_, 4.0);
   nh_param.param("scale_linear" , l_scale_, 2.0);

   // Checks on parameters
   if (autorepeat_rate_ > 1 / coalesce_interval_)
     ROS_WARN("joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1/coalesce_interval_);

   if (deadzone_ >= 1)
   {
     ROS_WARN("joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone by 32767, but this behavior is deprecated so you need to update your launch file.");
     deadzone_ /= 32767;
   }

   if (deadzone_ > 0.9)
   {
     ROS_WARN("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
     deadzone_ = 0.9;
   }

   if (deadzone_ < 0)
   {
     ROS_WARN("joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
     deadzone_ = 0;
   }

   if (autorepeat_rate_ < 0)
   {
     ROS_WARN("joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
     autorepeat_rate_ = 0;
   }

   if (coalesce_interval_ < 0)
   {
     ROS_WARN("joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
     coalesce_interval_ = 0;
   }

   // Parameter conversions
   double autorepeat_interval = 1 / autorepeat_rate_;
   double scale = -1. / (1. - deadzone_) / 32767.;
   double unscaled_deadzone = 32767. * deadzone_;

   js_event event;
   struct timeval tv;
   fd_set set;
   int joy_fd;
   event_count_ = 0;
   pub_count_ = 0;
   lastDiagTime_ = ros::Time::now().toSec();




   boost::thread th(boost::bind(&Joystick::publish_velocity, this));

   // Big while loop opens, publishes
   while (nh_.ok())
   {
     open_ = false;
//      diagnostic_.force_update();
     bool first_fault = true;
     while (true)
     {
       ros::spinOnce();
       if (!nh_.ok())
         goto cleanup;
       joy_fd = open(joy_dev_.c_str(), O_RDONLY);

       if (joy_fd != -1)
       {
         // There seems to be a bug in the driver or something where the
         // initial events that are to define the initial state of the
         // joystick are not the values of the joystick when it was opened
         // but rather the values of the joystick when it was last closed.
         // Opening then closing and opening again is a hack to get more
         // accurate initial state data.
         close(joy_fd);
         joy_fd = open(joy_dev_.c_str(), O_RDONLY);
       }
       if (joy_fd != -1)
         break;
       if (first_fault)
       {
         ROS_ERROR("Couldn't open joystick %s. Will retry every second.", joy_dev_.c_str());
         first_fault = false;
       }
       sleep(1.0);
//        diagnostic_.update();
     }

     ROS_INFO("Opened joystick: %s. deadzone_: %f.", joy_dev_.c_str(), deadzone_);
     open_ = true;
//      diagnostic_.force_update();

     bool tv_set = false;
     bool publication_pending = false;
     tv.tv_sec = 1;
     tv.tv_usec = 0;


//  	struct JS_DATA_TYPE js;
//  	while (1) {
//  		if (read (joy_fd, &js, JS_RETURN) != JS_RETURN) {
//  	      		/* error */
//  	   	}
//  	   	usleep (1000);
//  		std::cout << (js.x) << std::endl;
//  		std::cout << (js.y) << std::endl;
//  	}


//  	char name[128];
//  	if (ioctl(joy_fd, JSIOCGNAME(sizeof(name)), name) < 0)
//  		strncpy(name, "Unknown", sizeof(name));
//  	printf("Name: %s\n", name);
//
//
////      char number_of_axes;
////      ioctl (joy_fd, JSIOCGVERSION, &number_of_axes);
////      std::cout << "NUM AXIS = " << (int)number_of_axes << std::endl;
	#define BITS_TO_LONGS(x) \
		(((x) + 8 * sizeof (unsigned long) - 1) / (8 * sizeof (unsigned long)))
     unsigned long features[BITS_TO_LONGS(FF_CNT)];




     sensor_msgs::Joy joy_msg; // Here because we want to reset it on device close.
     while (nh_.ok())
     {
       ros::spinOnce();

       bool publish_now = false;
       bool publish_soon = false;
       FD_ZERO(&set);
       FD_SET(joy_fd, &set);

       //ROS_INFO("Select...");
       int select_out = select(joy_fd+1, &set, NULL, NULL, &tv);
       //ROS_INFO("Tick...");
       if (select_out == -1)
       {
         tv.tv_sec = 0;
         tv.tv_usec = 0;
         //ROS_INFO("Select returned negative. %i", ros::isShuttingDown());
         continue;
         //				break; // Joystick is probably closed. Not sure if this case is useful.
       }

       if (FD_ISSET(joy_fd, &set))
       {
         if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
           break; // Joystick is probably closed. Definitely occurs.

         //ROS_INFO("Read data...");
         joy_msg.header.stamp = ros::Time().now();
         event_count_++;
//          std::cout << "Event Type = " << event.type << ",Event Value = " << event.value << std::endl;
         switch(event.type)
         {
         case JS_EVENT_BUTTON:
         case JS_EVENT_BUTTON | JS_EVENT_INIT:
           if(event.number >= joy_msg.buttons.size())
           {
             int old_size = joy_msg.buttons.size();
             joy_msg.buttons.resize(event.number+1);
             for(unsigned int i=old_size;i<joy_msg.buttons.size();i++)
               joy_msg.buttons[i] = 0.0;
           }
           joy_msg.buttons[event.number] = (event.value ? 1 : 0);
           // For initial events, wait a bit before sending to try to catch
           // all the initial events.
           if (!(event.type & JS_EVENT_INIT))
             publish_now = true;
           else
             publish_soon = true;
           break;
         case JS_EVENT_AXIS:
         case JS_EVENT_AXIS | JS_EVENT_INIT:
           if(event.number >= joy_msg.axes.size())
           {
             int old_size = joy_msg.axes.size();
             joy_msg.axes.resize(event.number+1);
             for(unsigned int i=old_size;i<joy_msg.axes.size();i++)
             {
               joy_msg.axes[i] = 0.0;
             }
           }
           if (!(event.type & JS_EVENT_INIT)) // Init event.value is wrong.
           {
             double val = event.value;
             // Allows deadzone to be "smooth"
             if (val > unscaled_deadzone)
               val -= unscaled_deadzone;
             else if (val < -unscaled_deadzone)
               val += unscaled_deadzone;
             else
               val = 0;
             joy_msg.axes[event.number] = val * scale;
           }
           // Will wait a bit before sending to try to combine events.
           publish_soon = true;
           break;
         default:
           ROS_WARN("joy_node: Unknown event type. Please file a ticket. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
           break;
         }
       }
       else if (tv_set) // Assume that the timer has expired.
         publish_now = true;

       if (publish_now)
       {

         // Assume that all the JS_EVENT_INIT messages have arrived already.
         // This should be the case as the kernel sends them along as soon as
         // the device opens.
         //ROS_INFO("Publish...");

         // pub_.publish(joy_msg);
    	 last_joy_message_ = current_joy_message_;
       	 current_joy_message_ = joy_msg;
         publish_now = false;
         tv_set = false;
         publication_pending = false;
         publish_soon = false;
         pub_count_++;
         bool pub = false;
         if(last_joy_message_.axes.size() == current_joy_message_.axes.size())
        	 for(int i=0;i<current_joy_message_.axes.size();i++)
        		 if(last_joy_message_.axes[i] != current_joy_message_.axes[i])
        			 pub = true;
         if(last_joy_message_.buttons.size() == current_joy_message_.buttons.size())
        	 for(int i=0;i<current_joy_message_.buttons.size();i++)
        		 if(last_joy_message_.buttons[i] != current_joy_message_.buttons[i])
        			 pub = true;
         if(pub)
         {
			 allow_pub_mutex_.lock();
			 allow_pub_ = true;
			 allow_pub_mutex_.unlock();
         }
       }

       // If an axis event occurred, start a timer to combine with other
       // events.
       if (!publication_pending && publish_soon)
       {
//    	   std::cout << "INSIDE" << std::endl;
         tv.tv_sec = trunc(coalesce_interval_);
         tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
         publication_pending = true;
         tv_set = true;
         //ROS_INFO("Pub pending...");
       }

       // If nothing is going on, start a timer to do autorepeat.
       if (!tv_set && autorepeat_rate_ > 0)
       {
         tv.tv_sec = trunc(autorepeat_interval);
         tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
         tv_set = true;
         //ROS_INFO("Autorepeat pending... %i %i", tv.tv_sec, tv.tv_usec);
       }

       if (!tv_set)
       {
//    	   std::cout << "IN43SIDE" << std::endl;
         tv.tv_sec = 1;
         tv.tv_usec = 0;
       }

//        diagnostic_.update();
     } // End of joystick open loop.

     close(joy_fd);
     ros::spinOnce();
     if (nh_.ok())
       ROS_ERROR("Connection to joystick device lost unexpectedly. Will reopen.");
   }

 cleanup:
   ROS_INFO("joy_node shut down.");

   return 0;
 }
};

int main(int argc, char **argv)
{
 ros::init(argc, argv, "meteor_teleop_node");
 Joystick j;
 return j.main(argc, argv);
}
