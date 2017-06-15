#ifndef BALL_H
#define BALL_H

#include <opencv2\core\core.hpp>
#include<string>

class Ball 
{
public:
	Ball();
	~Ball();

	Ball(std::string name);

	void setXPos(int newX);
	void setYPos(int newY);

	int getXPos();
	int getYPos();

	void setType(std::string name);
	std::string getType();

	void setHSVmin(cv::Scalar newHSV);
	void setHSVmax(cv::Scalar newHSV);

	cv::Scalar getHSVmin();
	cv::Scalar getHSVmax();

	void setColor(cv::Scalar newColor);
	cv::Scalar getColor();

private:

	int x;
	int y;
	
	std::string type;

	cv::Scalar HSVmin;
	cv::Scalar HSVmax;

	cv::Scalar color;
};

#endif // !BALL_H

