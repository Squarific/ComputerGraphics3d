#include "2dhelpers.h"

Color::Color (double red, double green, double blue) {
	this->red = red;
	this->green = green;
	this->blue = blue;
}

Point2d::Point2d (double x, double y) {
	this->x = x;
	this->y = y;
}

Line2d::Line2d (Point2d start, Point2d end, Color color) {
	this->start = start;
	this->end = end;
	this->color = color;
}