#include "Vulcanuino.h"
#include "TimedAction.h"
#include <ros.h>
#include <geometry_msgs/Point32.h>

Vulcanuino vulcan(3,4,2,10);
ros::NodeHandle nh;

void spinOnceHandler(){
    vulcan.run();
}
void subscriber_cb(const geometry_msgs::Point32& cmd_msg){
    vulcan.axisMoveTo(cmd_msg.x * 60, cmd_msg.y * 180);
}

TimedAction spinOnce(1, spinOnceHandler);

void setup() {
    vulcan.begin();
    nh.initNode();
    ros::Subscriber<geometry_msgs::Point32> sub("vulcanuino", subscriber_cb);
    nh.subscribe(sub);
}

void loop(){ 
    nh.spinOnce();
    vulcan.run();
    //spinOnce.check();
}



