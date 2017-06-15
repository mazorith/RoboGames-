#include "Ball.h"

Ball::Ball()
{
}

Ball::~Ball()
{
}

Ball::Ball(std::string name)
{
	type = name;
}

void Ball::setXPos(int newX)
{
	x = newX;
}

void Ball::setYPos(int newY)
{
	y = newY;
}

int Ball::getXPos()
{
	return x;
}

int Ball::getYPos()
{
	return y;
}

void Ball::setType(std::string name)
{
	type = name;
}

std::string Ball::getType()
{
	return type;
}

void Ball::setHSVmin(cv::Scalar newHSV)
{
	HSVmin = newHSV;
}

void Ball::setHSVmax(cv::Scalar newHSV)
{
	HSVmax = newHSV;
}

cv::Scalar Ball::getHSVmin()
{
	return HSVmin;
}

cv::Scalar Ball::getHSVmax()
{
	return HSVmax;
}

void Ball::setColor(cv::Scalar newColor)
{
	color = newColor;
}

cv::Scalar Ball::getColor()
{
	return color;
}
