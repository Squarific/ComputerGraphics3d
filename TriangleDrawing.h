// Line drawing
// LICENSE: MIT

#ifndef TRIANGLEDRAWING_H
#define TRIANGLEDRAWING_H

// Clases used to construct a 2d drawing

#include "EasyImage.h"
#include "3dfigures.h"
#include "2dhelpers.h"
#include <list>

class Triangle {
public:
	Triangle () {};
	Triangle (std::vector<Point2d> points, Color color) {
		this->points = points;
		this->color = color;
	};

	std::vector<Point2d> points;
	Color color;
};

// The class used for drawings
class TriangleDrawing {
public:
	TriangleDrawing(){};

	std::vector<Triangle> triangles;

	// Return the coordinates of the biggest and smallest point
	// Returns point infinity if drawing does not contain lines
	Point2d getMaxPoint ();
	Point2d getMinPoint ();

	// Add a triangle to the drawing
	void addTriangle (Triangle triangle);

	// Scales the triangles in this drawing
	void scaleTriangles (double scaleFactor);

	// Move all points of all triangles from x, y to x + dx, y + dy
	void moveTriangles (double dx, double dy);

	// Add triangles using 3d figures
	// This changes the 3d figures
	void addTrianglesFromProjection (Figures3D& figures);

	// Draw the lines to the image with size the max size of x or y
	img::EasyImage drawTriangles(img::EasyImage& image);
	img::EasyImage drawTriangles(img::EasyImage& image, int size);
	img::EasyImage drawTrianglesZBuffered(double d, ZBuffer buffer, img::EasyImage& image);
	img::EasyImage drawTrianglesZBuffered(img::EasyImage& image, int size);
};

#endif /* TRIANGLEDRAWING_H */