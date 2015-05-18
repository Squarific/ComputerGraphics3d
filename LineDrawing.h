// Line drawing
// LICENSE: MIT

#ifndef LINEDRAWING_H
#define LINEDRAWING_H

// Clases used to construct a 2d drawing

#include "EasyImage.h"
#include "lparser.h"
#include "3dfigures.h"
#include "2dhelpers.h"
#include <list>

// The class used for drawings
class LineDrawing {
public:
	LineDrawing(){};
	LineDrawing(Lines2d& lines);

	Lines2d lines;

	// Return the coordinates of the biggest and smallest point
	// Returns point infinity if drawing does not contain lines
	Point2d getMaxPoint ();
	Point2d getMinPoint ();

	// Add a line to the drawing
	void addLine (Line2d line);

	// Scales the lines in this drawing
	void scaleLines (double scaleFactor);

	// Move all points of all lines from x, y to x + dx, y + dy
	void moveLines (double dx, double dy);

	void addLinesFromLSystem2D (LParser::LSystem2D& lSystem2d, img::Color color);
	void addLinesFromLString (LParser::LSystem2D& lSystem2d, std::string lString, img::Color color);

	// Add lines using 3d figures
	// This changes the 3d figures
	void addLinesFromProjection (Figures3D& figures);

	// Draw the lines to the image with size the max size of x or y
	img::EasyImage draw2dLines(img::EasyImage& image);
	img::EasyImage draw2dLines(img::EasyImage& image, int size);
};

#endif /* LINEDRAWING_H */