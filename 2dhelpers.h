#ifndef TWODHELPERS_H
#define TWODHELPERS_H

#include <list>

class Color {
public:
	Color() {};
	Color(double red, double green, double blue);
	
	double red; // 0 - 255
	double green; // 0 - 255
	double blue; // 0- 255
};

class Point2d {
public:
	Point2d() {};
	Point2d(double x, double y);
	
	double x;
	double y;
};

class Line2d {
public:
	Line2d() {};
	Line2d(Point2d start, Point2d end, Color color);

	Point2d start;
	Point2d end;
	Color color;
};

typedef std::list<Line2d> Lines2d;

#endif /* TWODHELPERS_H */