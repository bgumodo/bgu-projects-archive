#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <sstream>
#include <boost/thread/mutex.hpp>
#include "geometry_msgs/Twist.h"
#include "FaceDetector.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"

using namespace std;

// Synchronization variables to avoid collisions between the threads
boost::mutex range_value_mutex;
boost::mutex dir_value_mutex;
boost::mutex face_mutex;

// Distance given by the left and right range sensor
float right_range=0;
float left_range=0;
// The direction the robot should turn to  - a long and clear pathway 
float direction = 0;
float R1=0,R2=0,L1=0,L2=0,delta=0;
// Whether there aren't any close obstacles that prevent the robot from going forward
bool can_move = true;
// The number of consecutive frames where faces have been detected
int face_detections;
// Whether a person was greeted
bool greeted = false;


//get the range readings from left sensor
void leftURF_Callback(const sensor_msgs::Range::ConstPtr& msg) {
  //cout << "left range reading " << msg->range << endl;
  range_value_mutex.lock();
  left_range = msg->range;
  range_value_mutex.unlock();
}

//get the range readings from right sensor
void rightURF_Callback(const sensor_msgs::Range::ConstPtr& msg) {
  //cout << "right range reading " << msg->range << endl;
  range_value_mutex.lock();
  right_range = msg->range;
  range_value_mutex.unlock();
}

int canAdvance(const vector<float>& ranges, int numClearPixelsPerSide, float safeDistance) {
  // check if the depth pixels around the center are all outside the forbidden zone (if so, the robot can move forward safely)

  int numPixels = ranges.size();
  int middlePixel = numPixels/2;

  if(middlePixel-numClearPixelsPerSide<0 || middlePixel+numClearPixelsPerSide>=numPixels)
    cout << "Error: variable numClearPixelsPerSide is too high in function canAdvance" << endl;

  int canAdvance = 1;
  for (int i = middlePixel-numClearPixelsPerSide; i < middlePixel+numClearPixelsPerSide; ++i)
  {
    if(ranges[i]<safeDistance) {
      canAdvance = 0;
      break;
    }
  }

  return canAdvance;
}

float findClearCorridor(const vector<float>& ranges, int numCorridorPixels, float clearDistance, float pixelAngle) {
  // find the angular direction of a clear and somewhat long corridor to drive to

  int numPixels = ranges.size();
  int middlePixel = numPixels/2;

  // an array to hold all the pixels that are a center of a clear corridor
  int clearPixels[numPixels];
  for (int i = 0; i < numPixels; ++i) {
    clearPixels[i] = 0;
  }

  // check all possible corridors
  for (int i = 0; i < numPixels-numCorridorPixels+1; ++i) {
    int allClear = 1;
    for (int j = 0; j < numCorridorPixels; ++j) {
      if(ranges[i+j] < clearDistance) {
        allClear = 0;
        break;
      }
    }
    if(allClear == 1)
      clearPixels[i+numCorridorPixels/2] = 1;
  }

  // print all clean corridors
  int printAllCorridors = 0;
  if(printAllCorridors == 1) {
    for (int i = 0; i < numPixels; ++i) {
      if(clearPixels[i]==1)
        cout << (middlePixel-i)*pixelAngle << ",";
    }
    cout << endl;
  }

  // find corridor closest to the center
  int closestCorridorPixelDistance = 999999;
  float closestCorridorDegree = 0;
  for (int i = 0; i < numPixels; ++i) {
    if(clearPixels[i]==1 && fabs(middlePixel-i)<closestCorridorPixelDistance) {
      closestCorridorPixelDistance = fabs(middlePixel-i);
      closestCorridorDegree = (middlePixel-i)*pixelAngle;
    }
  }

  return closestCorridorDegree;
}

void LaserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /* Using input from the front laser scanner check for two things:
  1. Whether there are not obstacles in front of the robot (close range)
  2. To which direction should the robot face to drive without hitting an obstacle (mid-range)
  */
  
  

  /*
  Number of range measurements = 512
  Angle of each sensor = about 0.35 degrees (0.00613592332229 radians)
  Field of view = about 180 degrees (3.141592741 radians)
  */

  // look for a clear corridor of 30 degrees and 1.5 meters ahead
  int numPixels = 512;
  float pixelAngle = 0.35; // in degrees


  if(numPixels!=msg->ranges.size())
    cout << "Warning: number of Hokuyo laser sensors different than usual (512)" << endl;


  // check that we don't run into any thing
  float safeDistance = 0.40; // 40cm
  float safetyAngle = 120; // 120 degrees
  int numClearPixelsPerSide = (safetyAngle/2)/pixelAngle;
  if(!canAdvance(msg->ranges,numClearPixelsPerSide,safeDistance)) {
    cout << "Stop, cannot advance" << endl;
	dir_value_mutex.lock();
	can_move = false;
	dir_value_mutex.unlock();
	} else {
		dir_value_mutex.lock();
		can_move = true;
		dir_value_mutex.unlock();
	}

  // find corridor direction
  int numCorridorPixels = 120; // 120*0.00613592332229*180/pi=42 degrees
  float clearDistance = 1.5; // 1.5 meters ahead
  float degreesToTurn = findClearCorridor(msg->ranges,numCorridorPixels,clearDistance,pixelAngle);

  // negative means to turn right
  dir_value_mutex.lock();
  direction = degreesToTurn;
  dir_value_mutex.unlock();
  cout << "Degrees to turn = " << degreesToTurn << endl;
}



void motionControl(ros::Publisher& pub) {
	/*
	This is the main control function of the robot.
	This function stops the robot motion when there are close obstacles ahead or when a faces was found but was not yet greeted. 
	In case the robot can advance this function send the motion command with the right direction.
	*/
	geometry_msgs::Twist motion_command;
	// sample sensors in time t1
	range_value_mutex.lock();
	R2 = right_range;
	L2 =  left_range;
	range_value_mutex.unlock();
	
	dir_value_mutex.lock();
    float dir = direction;
	dir_value_mutex.unlock();

	dir_value_mutex.lock();
	face_mutex.lock();
	if (!can_move || face_detections>5 ) {
		face_mutex.unlock();
		dir_value_mutex.unlock();
		return;
	}
	face_mutex.unlock();
	dir_value_mutex.unlock();
	motion_command.linear.x=0.1;
	motion_command.angular.z=0;
	if (dir >0)
		motion_command.angular.z=-0.6;
	if (dir <0)
		motion_command.angular.z=0.6;
	pub.publish(motion_command);
	//cout << "boop" <<endl;
}


void frontCam_Callback(const sensor_msgs::ImageConstPtr& msg) {
	// Gets color images from the front camera and detects faces. When faces have been detected for 5 consecutive frame play a sound

	//cv_bridge::CvImagePtr cv_ptr;
	//cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	cv::Mat img = cv_bridge::toCvShare(msg,msg->encoding)->image;
	std::vector<Rect> faces;
	String faceModelFilename = "/home/komodo/catkin_ws/src/rob_proj/OpenCV Models/haarcascades/haarcascade_frontalface_alt.xml";
	FaceDetector detector(faceModelFilename);
	faces = detector.detect(img,false);
	face_mutex.lock();
	if (faces.size() > 0 ) {
		face_detections=min(face_detections+1,6);
	} else {
		face_detections=max(face_detections-1,0);
		greeted=false;
	}
	if (face_detections>5 && !greeted) {
		face_mutex.unlock();
		system("aplay /home/komodo/catkin_ws/src/rob_proj/sound/turret_autosearch_5.wav");
		greeted = true;
	} else {
		face_mutex.unlock();
		//cout << "ONWARD TO VICTORY!" << endl;
	}
	

}

int main(int argc, char **argv)
{

  // Initialize ros
  ros::init(argc, argv, "rob");
  ros::NodeHandle n;
  
  // Create out topic
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/komodo_1/cmd_vel", 1000);

  // Subscribe to left and right range scanenrs
  ros::Subscriber subLeftURF;
  ros::Subscriber subRightURF;
  subLeftURF = n.subscribe("/komodo_1/Rangers/Left_URF", 1000, leftURF_Callback);
  subRightURF = n.subscribe("/komodo_1/Rangers/Right_URF", 1000, rightURF_Callback);

  // Subscribe to front lazer scanner
  ros::Subscriber sub = n.subscribe("/scan", 1000, LaserScan_callback);
  
  // Subscribe the range camera's color images
  ros::Subscriber image_sub = n.subscribe("/komodo_1/komodo_1_Asus_Camera/rgb/image_raw", 3, frontCam_Callback);

  // ----------- example of loading an image and detecting faces 
/*
  Mat image;
  image = imread("/home/ehud/people3.jpg", CV_LOAD_IMAGE_COLOR);

  // Detect faces
  std::vector<Rect> faces;
  String faceModelFilename = "/home/ehud/catkin_ws/src/rob_proj/OpenCV Models/haarcascades/haarcascade_frontalface_alt.xml";
  FaceDetector detector(faceModelFilename);
  faces = detector.detect(image,true);
  imshow("Detected faces", image);
  waitKey(0);
*/
  // ------------

  // Main ROS loop
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    // call the main control function
    motionControl(pub);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
