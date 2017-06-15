#define _CRT_SECURE_NO_WARNINGS //supresses sprintf errors/warnings
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>
//#include <wiringPi.h>=========================================================================
//#include <wiringSerial.h>=====================================================================
#include <opencv2\highgui\highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2\core\core.hpp>
#include "Ball.h"

using namespace cv;
using namespace std;

//global varibales for easy debug
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 840;
const int FRAME_HEIGHT = 680;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

//==============================================================================================
//global variables for exiting threads safely
bool rotateScanBool = true;
bool rotateToBool = true;
bool moveDirectionBool = true;
bool returnMoveBool = true;
bool timedMoveBool = true;

bool searchBool = true;
bool rotateBool = false;
bool moveBool = false;

//global variables to be accessed by all threads
float roboXPos = 3.625; //center of 7.75'x7.75' grid (-14" for robo size)
float roboYPos = 3.625;
float roboAngle = 0; // 0 is orgin angle faceing bottom left hand corner, 0.5 is 180 deg and 1 is full 360 deg

float angleToRotateTo = 0; //for rotateTo thread
float timeToMove = 0; //for timedMove thread

float roboMovementSpeed = 1; //placeholder value
float roboRotationSpeed = 1; //placeholder value
//==============================================================================================

void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed
}
string intToString(int number);
void createTrackbars();
void drawObject(vector<Ball> balls, Mat &frame);
void drawObject(vector<Vec3f> &v3fCircles, vector<Ball> balls, Mat &frame);
void morphOps(Mat &thresh);
void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed);
void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed, vector<Ball> &newBalls);
void trackFilteredObject(vector<Vec3f> &v3fCircles, Ball ball, vector<Ball> &balls, Mat threshold, Mat HSV, Mat &cameraFeed);

//==============================================
//robofucntions / threads
void *rotateScan(void *data1); //CHANGE TO PI_THREAD(name of thread for all before running)
void *rotateTo(void *data1);
void *moveDirection(void *data1);
void *returnMove(void *data1);
void *timedMove(void *data1);
//================================================

int main(int argc, char* argv[])
{
	//==================================================================================================
	//wiringPiSetup () ;
	//pinMode(17, OUTPUT); // move
	//pinMode(18, OUTPUT); // rotate
	//pinMode(27, OUTPUT); //dump white
	//pinMode(22, OUTPUT); //dump black

	//variables for robot location, speeds, and states
	bool beginingPhase = true;
	bool allBallsFound = false; // second phase
	bool dumpPhase = false; // thrid phase
	bool dump = false;

	clock_t startTime = clock();
	clock_t currentTime;
	clock_t markedTime;

	float ballRotation; //rotation recorded when ball was found
    float ballRadius; //radius of found ball

	Ball currentBall; //current farthest away ball;
	currentBall.setYPos(1000);

	vector<Ball> foundBalls;

	//goal check vectors
	vector<Ball> whiteDetectedThings;
	vector<Ball> blackDetectedThings;

	//============================================================================================

	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = true;

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat HSV;

	if (calibrationMode) 
	{
		//create slider bars for HSV filtering
		createTrackbars();
	}

	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while (true)
	{
		//store image to matrix
		capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);

		if (calibrationMode == true) 
		{
			//if in calibration mode, we track objects based on the HSV slider values.
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
			morphOps(threshold);
			imshow(windowName2, threshold);
			trackFilteredObject(threshold, HSV, cameraFeed);
		}
		else 
		{
			//========================================================================================
			if (searchBool)
			{
				for (int i = 0; i < foundBalls.size(); i++)
				{
					if (foundBalls.at(i).getYPos() < currentBall.getYPos())
					{
						ballRotation = roboAngle;
						currentBall = foundBalls.at(i);
					}
				}
			}
			//=========================================================================================

			foundBalls.clear();

			vector<Vec3f> v3fCircles;

			Ball whiteBall;
			Ball BlackBall;

			
			whiteBall.setHSVmin(Scalar(49, 105, 61));
			whiteBall.setHSVmax(Scalar(96, 255, 255));
			whiteBall.setColor(Scalar(0, 255, 0));
			whiteBall.setType("GREEN");

			BlackBall.setHSVmin(Scalar(91, 135, 82));
			BlackBall.setHSVmax(Scalar(142, 255, 255));
			BlackBall.setColor(Scalar(255, 0, 0));
			BlackBall.setType("BLUE");

			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, whiteBall.getHSVmin(), whiteBall.getHSVmax(), threshold);
			morphOps(threshold);
			trackFilteredObject(v3fCircles, whiteBall, foundBalls, threshold, HSV, cameraFeed);

			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, BlackBall.getHSVmin(), BlackBall.getHSVmax(), threshold);
			morphOps(threshold);
			trackFilteredObject(v3fCircles, BlackBall, foundBalls, threshold, HSV, cameraFeed);
		}

		
		//=============================================================================================================
		currentTime = clock();
		//begining phase
		//if 3 minutes have passed (180 seconds) and we are still in the begining phase
		if (beginingPhase && (currentTime - startTime) / (double)CLOCKS_PER_SEC >= 180)
		{
			beginingPhase = false;
			allBallsFound = true;
			rotateBool = true;
		}

		//---PHASE 1----
		if (beginingPhase)
		{
			if (searchBool)
			{
				//piThreadCreate(RotateScan);
				searchBool = false;
			}
			if (rotateBool)
			{
				if (currentBall.getYPos() == 1000)
				{
					beginingPhase = false;
				}
				else
				{
					//piThreadCreate(rotateTo);
					angleToRotateTo = ballRotation;
					rotateBool = false;
				}
			}
			if (moveBool)
			{
				//piThreadCreate(moveDirection);
				moveBool = false;
			}
			if (!moveDirectionBool)
			{
				//when ball is no longer tracked
				if (foundBalls.size() == 0)
				{
					//delay(250); // 0.25 seconds
					moveDirectionBool = true;
					//delay(1000); //1 second

					if (foundBalls.size() == 0)
					{
						searchBool = true;
						currentBall.setYPos(1000);
					}
					else
					{
						//piThreadCreate(moveDirection);
					}
				}
			}
		}

		//---PHASE 2---
		if (allBallsFound)
		{
			if (rotateBool)
			{
				//piThreadCreate(returnMove);
				rotateBool = false;
			}
			if (roboXPos == 4 && roboYPos == 4)
			{
				allBallsFound = false;
				rotateBool = true;
			}
		}

		//---PHASE 3---
		if (dumpPhase)
		{
			bool whiteDetected = false;
			bool blackDetected = false;
			float timeToRotate;

			timeToRotate = (roboAngle - 0.25) / roboRotationSpeed;
			if (timeToRotate < 0)
			{
				timeToRotate *= -1;
			}
			else
			{
				timeToRotate = 1; //[time of complete rotation] - timeToRotate;
			}
			//digitalWrite(18, HIGH);
			//delay(timeToRotate * 1000);
			//digitalWrite(18, LOW);
			roboAngle = angleToRotateTo;
			/*

			piThreadCreate(timedMove);
			timeToMove = [time it takes to move to goal where tape is visible]);
			delay(timetoMove * 1000);*/

			//~~~code to find which object is present (black or white tape)~~~
			/*cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold); // set scalars to white high and low HSV ranges
			morphOps(threshold);
			trackFilteredObject(threshold, HSV, cameraFeed, whiteDetectedThings);
			
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold); // set scalars to black high and low HSV ranges
			morphOps(threshold);
			trackFilteredObject(threshold, HSV, cameraFeed, blackDetectedThings);
			*/

			if (whiteDetectedThings.size() >= blackDetectedThings.size()) 
			{
				whiteDetected = true;
			}
			else 
			{
				blackDetected = true;
			}

			/*piThreadCreate(timedMove);
			timeToMove = [time it takes to finish moving to goal];
			delay(timetoMove * 1000);

			//turn around to dump
			timeToRotate = (roboAngle - 0.75) / roboRotationSpeed;
			if (timeToRotate < 0)
			{
				timeToRotate *= -1;
			}
			else
			{
				timeToRotate = 1; //[time of complete rotation] - timeToRotate;
			}
			//digitalWrite(18, HIGH);
			//delay(timeToRotate * 1000);
			//digitalWrite(18, LOW);
			roboAngle = angleToRotateTo;

			if(whiteDetected)
			{
				//digitalWrite(27, HIGH);
			}
			else
			{
				digitalWrite(22, HIGH);
			}

			delay([time in milliseconds it takes to dump balls]);

			if(whiteDetected)
			{
			//digitalWrite(27, LOW);
			}
			else
			{
			digitalWrite(22, LOW);
			}

			piThreadCreate(timedMove);
			timeToMove = [time it takes to finish moving];
			delay(timetoMove * 1000);

			//turn around to dump
			timeToRotate = (roboAngle - 0.25) / roboRotationSpeed;
			if (timeToRotate < 0)
			{
				timeToRotate *= -1;
			}
			else
			{
				timeToRotate = 1; //[time of complete rotation] - timeToRotate;
			}
			//digitalWrite(18, HIGH);
			//delay(timeToRotate * 1000);
			//digitalWrite(18, LOW);
			roboAngle = angleToRotateTo;

			if(whiteDetected)
			{
				digitalWrite(22, HIGH);
			}
			else
			{
				digitalWrite(27, HIGH);
			}

			delay([time in milliseconds it takes to dump balls]);

			if(whiteDetected)
			{
			digitalWrite(22, LOW);
			}
			else
			{
			digitalWrite(27, LOW);
			}

			return 0; done :D*/
		}
		//=============================================================================================================

		//show frames 
		imshow(windowName, cameraFeed);
		//imshow(windowName2,threshold);
		//imshow(windowName1,HSV);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		//wait for 'esc' key press for 30ms (27 is esc key value). If 'esc' key is pressed, break loop
		if (waitKey(30) == 27) 
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;
}

string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars()
{
	//create window for trackbars

	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX);
}

void drawObject(vector<Ball> balls, Mat &frame)
{
	for (int i = 0; i < balls.size(); i++)
	{
		cv::circle(frame, cv::Point(balls.at(i).getXPos(), balls.at(i).getYPos()), 10, balls.at(i).getColor());
		cv::putText(frame, intToString(balls.at(i).getXPos()) + " , " + intToString(balls.at(i).getYPos()), cv::Point(balls.at(i).getXPos(), balls.at(i).getYPos() + 20), 1, 1, balls.at(i).getColor());
		cv::putText(frame, balls.at(i).getType(), cv::Point(balls.at(i).getXPos(), balls.at(i).getYPos() - 30), 1, 2, balls.at(i).getColor());
	}
}

void drawObject(vector<Vec3f> &v3fCircles, vector<Ball> balls, Mat &frame)
{
	for (int i = 0; i < balls.size(); i++)
	{
		cv::circle(frame, cv::Point(balls.at(i).getXPos(), balls.at(i).getYPos()), 10, balls.at(i).getColor());
		cv::putText(frame, intToString(balls.at(i).getXPos()) + " , " + intToString(balls.at(i).getYPos()), cv::Point(balls.at(i).getXPos(), balls.at(i).getYPos() + 20), 1, 1, balls.at(i).getColor());
		cv::putText(frame, balls.at(i).getType(), cv::Point(balls.at(i).getXPos(), balls.at(i).getYPos() - 30), 1, 2, balls.at(i).getColor());
	}
	for (int i = 0; i < v3fCircles.size(); i++) {		// for each circle . . .
														// show ball position x, y, and radius to command line
		std::cout << "ball position x = " << v3fCircles[i][0]			// x position of center point of circle
			<< ", y = " << v3fCircles[i][1]								// y position of center point of circle
			<< ", radius = " << v3fCircles[i][2] << "\n";				// radius of circle

																		// draw small green circle at center of detected object
		cv::circle(frame,												// draw on original image
			cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
			3,																// radius of circle in pixels
			cv::Scalar(0, 255, 0),											// draw pure green (remember, its BGR, not RGB)
			CV_FILLED);														// thickness, fill in the circle

																			// draw red circle around the detected object
		cv::circle(frame,												// draw on original image
			cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
			(int)v3fCircles[i][2],											// radius of circle in pixels
			cv::Scalar(0, 0, 255),											// draw pure red (remember, its BGR, not RGB)
			3);																// thickness of circle in pixels
	}
}

void morphOps(Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed)
{
	vector<Ball> newBalls;

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA)
				{
					Ball newBall;

					newBall.setXPos(moment.m10 / area);
					newBall.setYPos(moment.m01 / area);

					newBalls.push_back(newBall);

					objectFound = true;

				}
				else
				{
					objectFound = false;
				}
			}
			//let user know you found an object
			if (objectFound == true) {
				//draw object location on screen
				drawObject(newBalls, cameraFeed);
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}

void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed, vector<Ball> &newBalls)
{
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA)
				{
					Ball newBall;

					newBall.setXPos(moment.m10 / area);
					newBall.setYPos(moment.m01 / area);

					newBalls.push_back(newBall);

					objectFound = true;

				}
				else
				{
					objectFound = false;
				}
			}
			//let user know you found an object
			if (objectFound == true) {
				//draw object location on screen
				drawObject(newBalls, cameraFeed);
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}

void trackFilteredObject(vector<Vec3f> &v3fCircles, Ball ball, vector<Ball> &balls, Mat threshold, Mat HSV, Mat &cameraFeed)
{

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 10 px by 10px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA)
				{
					Ball newBall;

					newBall.setXPos(moment.m10 / area);
					newBall.setYPos(moment.m01 / area);
					newBall.setType(ball.getType());
					newBall.setColor(ball.getColor());

					balls.push_back(newBall);

					objectFound = true;

					cv::HoughCircles(threshold,			// input image
						v3fCircles,							// function output (must be a standard template library vector
						CV_HOUGH_GRADIENT,					// two-pass algorithm for detecting circles, this is the only choice available
						2,									// size of image / this value = "accumulator resolution", i.e. accum res = size of image / 2
						threshold.rows / 16,				// min distance in pixels between the centers of the detected circles
						75,								// high threshold of Canny edge detector (called by cvHoughCircles)						
						37,									// low threshold of Canny edge detector (set at 1/2 previous value)
						8,									// min circle radius (any circles with smaller radius will not be returned)
						115);								// max circle radius (any circles with larger radius will not be returned)
					for (int i = 0; i < v3fCircles.size(); i++)
					{
						v3fCircles[i][0] = moment.m10 / area;
						v3fCircles[i][1] = moment.m01 / area;
					}
				}
				else
				{
					objectFound = false;
				}
			}
			//let user know you found an object
			if (objectFound == true)
			{
				//draw object location on screen
				drawObject(v3fCircles, balls, cameraFeed);
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}


//=========================================================================================================================
void *rotateScan(void *data1)
{
	//rotate until clock_t reaches time of complete rotation
	if (rotateToBool && moveDirectionBool && returnMoveBool && timedMoveBool)
	{
		rotateScanBool = false;

		clock_t currentTime;
		clock_t intervalTime;

		while (!rotateScanBool) 
		{
			intervalTime = clock();
			//digitalWrite(18, HIGH);
			currentTime = clock();
			float timeSpentRotating = (currentTime - intervalTime) / (float)CLOCKS_PER_SEC;
			roboAngle += (timeSpentRotating * roboRotationSpeed);
			if (roboAngle >= 1) 
			{
				roboAngle -= 1;
			}
		}
		//digitalWrite(18, LOW);
		rotateScanBool = true;
		rotateBool = true;
	}
	return 0;
}

void *rotateTo(void *data1)
{
	if (rotateScanBool && moveDirectionBool && returnMoveBool && timedMoveBool)
	{
		rotateToBool = false;

		//for rotating clockwise
		float timeToRotate; 
		if (angleToRotateTo != 0 && angleToRotateTo != 1)
		{
			timeToRotate = (roboAngle - angleToRotateTo) / roboRotationSpeed;
			if (timeToRotate < 0)
			{
				timeToRotate *= -1;
			}
			else
			{
				timeToRotate = 1; //[time of complete rotation] - timeToRotate;
			}
		}
		else 
		{
			timeToRotate = (1 - roboAngle) / roboRotationSpeed;
		}
		//digitalWrite(18, HIGH);
		//delay(timeToRotate * 1000);
		//digitalWrite(18, LOW);
		roboAngle = angleToRotateTo;
		rotateToBool = true;	
		moveBool = true;
	}
	return 0;
}

//*************************WARNING****************************
//--this thread will need to be stopped from the main thread--
void *moveDirection(void *data1)
{
	//move forward :P
	if (rotateToBool && rotateScanBool && returnMoveBool && timedMoveBool)
	{
		moveDirectionBool = false;

		clock_t beginMoveTime = clock();
		clock_t currentTime;

		int xDirection = 0;
		int yDirection = 0;
		int quadNum = 0;

		float xMoved = 0;
		float yMoved = 0;

		float degreesToXAxis = 0;
		float degreesToYAxis = 0;

		//robot is flush on x axis
		if (roboAngle == 0.125)
		{
			xDirection = -1;
			quadNum = 0;
		}
		else if (roboAngle == 0.625) 
		{
			xDirection = 1;
			quadNum = 0;
		}
		//robot is flush on y axis
		else if (roboAngle == 0.375)
		{
			yDirection = -1;
			quadNum = -1;
		}
		else if (roboAngle == 0.875)
		{
			yDirection = 1;
			quadNum = -1;
		}

		//in quadrant 3
		if(roboAngle < 0.125 || roboAngle > 0.875) 
		{ 
			xDirection = -1;
			yDirection = 1;
			quadNum = 3;
		}
		//in quadrant 1
		else if (roboAngle > 0.125 && roboAngle < 0.375)
		{
			xDirection = -1;
			yDirection = -1;
			quadNum = 1;
		}
		//in quadrant 2
		else if (roboAngle > 0.375 && roboAngle < 0.625)
		{
			xDirection = 1;
			yDirection = -1;
			quadNum = 2;
		}
		//in quadrant 4
		else if (roboAngle > 0.625 && roboAngle < 0.875)
		{
			xDirection = 1;
			yDirection = 1;
			quadNum = 4;
		}

		float angleToXAxis;
		float angleToYAxis;

		while(!moveDirectionBool)
		{

			//digitalWrite(17, HIGH);

			currentTime = clock();
			float timeSpentMoving = (currentTime - beginMoveTime) / (float)CLOCKS_PER_SEC;
			beginMoveTime = clock(); //also the invertval clock AKA the time between calculations

			float  hypot = timeSpentMoving * roboMovementSpeed;

			//if on axis
			if (quadNum == -1) 
			{
				xMoved = hypot * xDirection;
				roboXPos += xMoved;
			}
			else if (quadNum == 0) 
			{
				yMoved = hypot * yDirection;
				roboYPos += yMoved;
			}

			//if in a quadrant
			else if (quadNum == 1) 
			{
				degreesToXAxis = roboAngle - 0.125;
				degreesToYAxis = 0.25 - degreesToXAxis;
			}
			else if (quadNum == 2) 
			{
				degreesToXAxis = 0.625 - roboAngle;
				degreesToYAxis = 0.25 - degreesToXAxis;
			}
			else if (quadNum == 3) 
			{
				if (roboAngle < 0.125) 
				{
					degreesToXAxis = 0.125 - roboAngle;
					degreesToYAxis = 0.25 - degreesToXAxis;
				}
				else if (roboAngle > 0.875) 
				{
					degreesToYAxis = roboAngle - 0.875;
					degreesToXAxis = 0.25 - degreesToYAxis;
				}
			}
			else if (quadNum == 4) 
			{
				degreesToXAxis = roboAngle - 0.625;
				degreesToYAxis = 0.25 - degreesToXAxis;
			}

			//update pos (if not on axis)
			if (quadNum != -1 || quadNum != 0) 
			{
				xMoved = hypot * (sin(((degreesToXAxis * 360) * 3.14159) / 180)) * xDirection;
				yMoved = hypot * (sin(((degreesToYAxis * 360) * 3.14159) / 180)) * yDirection;
				roboXPos += xMoved;
				roboYPos += yMoved;
				if (roboXPos <= 0 || roboYPos >= 7.75 || roboYPos <= 0 || roboXPos >= 7.75) 
				{
					moveDirectionBool = true;
					searchBool = true;
				}
			}
		}
		//digitalWrite(17, LOW);
	}

	return 0;
}
void * returnMove(void * data1)
{
	//for rotating clockwise
	float timeToRotate = (1 - roboAngle) / roboRotationSpeed;
	//digitalWrite(18, HIGH);
	//delay(timeToRotate * 1000);
	//digitalWrite(18, LOW);
	roboAngle = angleToRotateTo;

	moveDirectionBool = false;

	clock_t beginMoveTime = clock();
	clock_t currentTime;

	int xDirection = 0;
	int yDirection = 0;
	int quadNum = 0;

	float xMoved = 0;
	float yMoved = 0;

	float degreesToXAxis = 0;
	float degreesToYAxis = 0;

	//robot is flush on x axis
	if (roboAngle == 0.125)
	{
		xDirection = -1;
		quadNum = 0;
	}
	else if (roboAngle == 0.625)
	{
		xDirection = 1;
		quadNum = 0;
	}
	//robot is flush on y axis
	else if (roboAngle == 0.375)
	{
		yDirection = -1;
		quadNum = -1;
	}
	else if (roboAngle == 0.875)
	{
		yDirection = 1;
		quadNum = -1;
	}

	//in quadrant 3
	if (roboAngle < 0.125 || roboAngle > 0.875)
	{
		xDirection = -1;
		yDirection = 1;
		quadNum = 3;
	}
	//in quadrant 1
	else if (roboAngle > 0.125 && roboAngle < 0.375)
	{
		xDirection = -1;
		yDirection = -1;
		quadNum = 1;
	}
	//in quadrant 2
	else if (roboAngle > 0.375 && roboAngle < 0.625)
	{
		xDirection = 1;
		yDirection = -1;
		quadNum = 2;
	}
	//in quadrant 4
	else if (roboAngle > 0.625 && roboAngle < 0.875)
	{
		xDirection = 1;
		yDirection = 1;
		quadNum = 4;
	}

	float angleToXAxis;
	float angleToYAxis;

	while (!moveDirectionBool)
	{

		//digitalWrite(17, HIGH);

		currentTime = clock();
		float timeSpentMoving = (currentTime - beginMoveTime) / (float)CLOCKS_PER_SEC;
		beginMoveTime = clock(); //also the invertval clock AKA the time between calculations

		float  hypot = timeSpentMoving * roboMovementSpeed;

		//if on axis
		if (quadNum == -1)
		{
			xMoved = hypot * xDirection;
			roboXPos += xMoved;
			if (roboXPos >= 3.95 && roboXPos <= 4.05)
			{
				moveDirectionBool = true;
			}
		}
		else if (quadNum == 0)
		{
			yMoved = hypot * yDirection;
			roboYPos += yMoved;
			if (roboYPos >= 3.95 && roboYPos <= 4.05)
			{
				moveDirectionBool = true;
			}
		}

		//if in a quadrant
		else if (quadNum == 1)
		{
			degreesToXAxis = roboAngle - 0.125;
			degreesToYAxis = 0.25 - degreesToXAxis;
		}
		else if (quadNum == 2)
		{
			degreesToXAxis = 0.625 - roboAngle;
			degreesToYAxis = 0.25 - degreesToXAxis;
		}
		else if (quadNum == 3)
		{
			if (roboAngle < 0.125)
			{
				degreesToXAxis = 0.125 - roboAngle;
				degreesToYAxis = 0.25 - degreesToXAxis;
			}
			else if (roboAngle > 0.875)
			{
				degreesToYAxis = roboAngle - 0.875;
				degreesToXAxis = 0.25 - degreesToYAxis;
			}
		}
		else if (quadNum == 4)
		{
			degreesToXAxis = roboAngle - 0.625;
			degreesToYAxis = 0.25 - degreesToXAxis;
		}

		//update pos (if not on axis)
		if (quadNum != -1 || quadNum != 0)
		{
			xMoved = hypot * (sin(((degreesToXAxis * 360) * 3.14159) / 180)) * xDirection;
			yMoved = hypot * (sin(((degreesToYAxis * 360) * 3.14159) / 180)) * yDirection;
			roboXPos += xMoved;
			roboYPos += yMoved;
			if (roboXPos >= 3.95 && roboXPos <= 4.05 && roboYPos >= 3.95 && roboYPos <= 4.05)
			{
				moveDirectionBool = true;
			}
		}
	}
	//digitalWrite(17, LOW);
	moveDirectionBool = true;

	return 0;
}

void * timedMove(void * data1)
{
	moveDirectionBool = false;

	clock_t beginMoveTime = clock();
	clock_t intervalMoveTime = clock();
	clock_t currentTime;

	int xDirection = 0;
	int yDirection = 0;
	int quadNum = 0;

	float xMoved = 0;
	float yMoved = 0;

	float degreesToXAxis = 0;
	float degreesToYAxis = 0;

	//robot is flush on x axis
	if (roboAngle == 0.125)
	{
		xDirection = -1;
		quadNum = 0;
	}
	else if (roboAngle == 0.625)
	{
		xDirection = 1;
		quadNum = 0;
	}
	//robot is flush on y axis
	else if (roboAngle == 0.375)
	{
		yDirection = -1;
		quadNum = -1;
	}
	else if (roboAngle == 0.875)
	{
		yDirection = 1;
		quadNum = -1;
	}

	//in quadrant 3
	if (roboAngle < 0.125 || roboAngle > 0.875)
	{
		xDirection = -1;
		yDirection = 1;
		quadNum = 3;
	}
	//in quadrant 1
	else if (roboAngle > 0.125 && roboAngle < 0.375)
	{
		xDirection = -1;
		yDirection = -1;
		quadNum = 1;
	}
	//in quadrant 2
	else if (roboAngle > 0.375 && roboAngle < 0.625)
	{
		xDirection = 1;
		yDirection = -1;
		quadNum = 2;
	}
	//in quadrant 4
	else if (roboAngle > 0.625 && roboAngle < 0.875)
	{
		xDirection = 1;
		yDirection = 1;
		quadNum = 4;
	}

	float angleToXAxis;
	float angleToYAxis;

	while (!moveDirectionBool)
	{
		//digitalWrite(17, HIGH);

		currentTime = clock();
		//if time is passed stop
		if (currentTime - intervalMoveTime / (float)CLOCKS_PER_SEC >= timeToMove) 
		{
			moveDirectionBool = true;
		}
		float timeSpentMoving = (currentTime - beginMoveTime) / (float)CLOCKS_PER_SEC;
		beginMoveTime = clock(); //also the invertval clock AKA the time between calculations

		float  hypot = timeSpentMoving * roboMovementSpeed;

		//if on axis
		if (quadNum == -1)
		{
			xMoved = hypot * xDirection;
			roboXPos += xMoved;
		}
		else if (quadNum == 0)
		{
			yMoved = hypot * yDirection;
			roboYPos += yMoved;
		}

		//if in a quadrant
		else if (quadNum == 1)
		{
			degreesToXAxis = roboAngle - 0.125;
			degreesToYAxis = 0.25 - degreesToXAxis;
		}
		else if (quadNum == 2)
		{
			degreesToXAxis = 0.625 - roboAngle;
			degreesToYAxis = 0.25 - degreesToXAxis;
		}
		else if (quadNum == 3)
		{
			if (roboAngle < 0.125)
			{
				degreesToXAxis = 0.125 - roboAngle;
				degreesToYAxis = 0.25 - degreesToXAxis;
			}
			else if (roboAngle > 0.875)
			{
				degreesToYAxis = roboAngle - 0.875;
				degreesToXAxis = 0.25 - degreesToYAxis;
			}
		}
		else if (quadNum == 4)
		{
			degreesToXAxis = roboAngle - 0.625;
			degreesToYAxis = 0.25 - degreesToXAxis;
		}

		//update pos (if not on axis)
		if (quadNum != -1 || quadNum != 0)
		{
			xMoved = hypot * (sin(((degreesToXAxis * 360) * 3.14159) / 180)) * xDirection;
			yMoved = hypot * (sin(((degreesToYAxis * 360) * 3.14159) / 180)) * yDirection;
			roboXPos += xMoved;
			roboYPos += yMoved;
			if (roboXPos <= 0 || roboYPos >= 7.75 || roboYPos <= 0 || roboXPos >= 7.75)
			{
				moveDirectionBool = true;
			}
		}
	}
	//digitalWrite(17, LOW);
	moveDirectionBool = true;

	return 0;
}
//===========================================================================================================================
