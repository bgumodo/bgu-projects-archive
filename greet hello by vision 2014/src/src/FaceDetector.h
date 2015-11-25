#ifndef FaceDetector_h
#define FaceDetector_h

#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>


using namespace std;
using namespace cv;


class FaceDetector {
    
public:
	
	// constructor - gets the full path (and name) of the face model file to use. 
	// The expected file is:
	// <opencv files>/data/haarcascades/haarcascade_frontalface_alt.xml
	FaceDetector(String faceModelFilename);

	// destructor
	~FaceDetector();

	// detect faces
	// image - an image in OpenCV's format
	// drawResults - whether to draw the found faces on the input image
	// return value - a vector of Rects, where each Rect corresponds to a face. A Rect contains the fields x,y,width,height (where x,y are the center of the Rect)
	std::vector<Rect> detect(Mat& image, bool drawResults);
  
private:
	
	// To hold the face classifier
	CascadeClassifier faceCascade;
    
};

#endif