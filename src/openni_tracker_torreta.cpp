// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <algorithm>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

//#include "openni_tracker_kinect.cpp"

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker_torreta");
    ros::NodeHandle nh;

    string configFilename = ros::package::getPath("openni_tracker_torreta") + "/openni_tracker_torreta.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
	    if (nRetVal != XN_STATUS_OK) {
		    ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
	    }
	}

	if (!g_DepthGenerator.IsCapabilitySupported(XN_CAPABILITY_USER_POSITION)) {
		ROS_INFO("Supplied user generator doesn't support user position");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(10);
        
    ros::NodeHandle pnh("~");
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);

    // Create a publisher and name the topic.
    ros::Publisher vulcanuino_pub = nh.advertise<geometry_msgs::Point32>("vulcanuino", 1);
                
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		//publishTransforms(frame_id);
		
		XnUserID users[15];
	    XnUInt16 users_count = g_UserGenerator.GetNumberOfUsers();
	    g_UserGenerator.GetUsers(users, users_count);
	    //ROS_INFO("User count: %d", users_count);
	    for (int i = 0; i < users_count; ++i) {
	        XnUserID user = users[i];
	        XnPoint3D center;
	    	g_UserGenerator.GetCoM(user, center);
	    	if(center.X + center.Y + center.Z > 0){
	    		//ROS_INFO("X: %f, Y:%f Z: %f\n", center.X, center.Y, center.Z);
	    		geometry_msgs::Point32 msg;
	    		msg.x = 1 - std::max<float>(0.0, std::min<float>(1.0, (center.X + 1000) / 2000));
	    		msg.y = std::max<float>(0.0, std::min<float>(1.0, (center.Y + 500) / 1000));
	    		msg.z = std::max<float>(0.0, std::min<float>(1.0, (center.Z) / 4000));
	    		vulcanuino_pub.publish(msg);
	    		//publishMessage
	    	}
	    }


		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
