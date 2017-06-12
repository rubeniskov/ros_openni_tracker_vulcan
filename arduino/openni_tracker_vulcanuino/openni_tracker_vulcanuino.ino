#ifndef TWI_FREQ
#define TWI_FREQ 400000L
#endif

#define PI 3.1415926535897932384626433832795

#include "Vulcanuino.h"
#include <ros.h>
#include <geometry_msgs/Point32.h>

Vulcanuino vulcan(3,4,2,10);

void subscriber_cb(const geometry_msgs::Point32& cmd_msg){
    vulcan.trackUser(
          180 + (atan(cmd_msg.x/cmd_msg.z) * -1) * (180 / PI), 
          90 + (atan(cmd_msg.y/cmd_msg.z) * -4.0) * (180 / PI));
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Point32> sub("vulcanuino", subscriber_cb);

void setup() {
    vulcan.begin();
    nh.initNode();
    nh.subscribe(sub);
}

void loop(){ 
    nh.spinOnce();
    //vulcan.axisMoveTo(60, 180);
    vulcan.run();
}



