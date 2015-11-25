#include "FaceDetector.h"



// Method explanation comments are in the .h file

FaceDetector::FaceDetector(String faceModelFilename) {
  	// load face model file
	if(!faceCascade.load(faceModelFilename)) 
		cout << "Error loading face model file: " << endl << faceModelFilename << endl;
}


FaceDetector::~FaceDetector(){

}


std::vector<Rect> FaceDetector::detect(Mat& image, bool drawResults){
  	Mat image_gray; // the gray version of image
  	std::vector<Rect> faces; // to hold the found faces

  	// convert image to gray
  	cvtColor(image, image_gray, CV_BGR2GRAY);
  	// sligthly improve gray image (with histogram equalization)
  	equalizeHist(image_gray, image_gray);

  	// Detect faces
  	faceCascade.detectMultiScale(image_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));
  	//cout << "Number of detected faces = " << faces.size() << endl;

  	// draw found faces on input image
  	if(drawResults) {
  		for( size_t i = 0; i < faces.size(); i++ ) {
  			Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
  			ellipse(image, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
  		}
  	}

  	return faces;
  	}
