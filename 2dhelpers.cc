#include "2dhelpers.h"
#include <vector>
#include <limits>

Color::Color (double red, double green, double blue) {
	this->red = red;
	this->green = green;
	this->blue = blue;
}

Point2d::Point2d (double x, double y) {
	this->x = x;
	this->y = y;
}

Point2d::Point2d (double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}


Line2d::Line2d (Point2d start, Point2d end, Color color) {
	this->start = start;
	this->end = end;
	this->color = color;
}

ZBuffer::ZBuffer (int width, int height) {
	for (int i = 0; i < width; i++) {
		std::vector<double> collumn;
		for (int j = 0; j < height; j++) {
			collumn.push_back(std::numeric_limits<double>::infinity());
		}
		this->push_back(collumn);
	}
}