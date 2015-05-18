#include "EasyImage.h"
#include "lparser.h"
#include "2dhelpers.h"
#include "LineDrawing.h"
#include <limits>
#include <cmath>

inline int roundToInt(double d) {
    return d < 0 ? 
        std::ceil(d-0.5):
        std::floor(d+0.5);
}

LineDrawing::LineDrawing (Lines2d& lines) {
	this->lines = lines;
};

void LineDrawing::addLine (Line2d line) {
	this->lines.push_back(line);
};

Point2d LineDrawing::getMaxPoint () {
	Point2d max = Point2d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
	for (Lines2d::iterator linesIter = this->lines.begin(); linesIter != this->lines.end(); linesIter++) {
		if (max.x < linesIter->start.x) max.x = linesIter->start.x;
		if (max.y < linesIter->start.y) max.y = linesIter->start.y;
		if (max.x < linesIter->end.x) max.x = linesIter->end.x;
		if (max.y < linesIter->end.y) max.y = linesIter->end.y;
	}
	return max;
}

Point2d LineDrawing::getMinPoint () {
	Point2d min = Point2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	for (Lines2d::iterator linesIter = this->lines.begin(); linesIter != this->lines.end(); linesIter++) {
		if (min.x > linesIter->start.x) min.x = linesIter->start.x;
		if (min.y > linesIter->start.y) min.y = linesIter->start.y;
		if (min.x > linesIter->end.x) min.x = linesIter->end.x;
		if (min.y > linesIter->end.y) min.y = linesIter->end.y;
	}
	return min;
}

void LineDrawing::scaleLines (double scaleFactor) {
	for (auto &line : lines) {
		line.start.x *= scaleFactor;
		line.start.y *= scaleFactor;
		line.end.x *= scaleFactor;
		line.end.y *= scaleFactor;
	}
}

void LineDrawing::moveLines (double dx, double dy) {
	for (auto &line : lines) {
		line.start.x += dx;
		line.start.y += dy;
		line.end.x += dx;
		line.end.y += dy;
	}
}

void LineDrawing::addLinesFromLSystem2D (LParser::LSystem2D& lSystem2d, img::Color color) {
	int times = lSystem2d.get_nr_iterations();
	std::string fullString = lSystem2d.get_initiator();

	while (times > 0) {
		std::string tempString = "";
		
		for(std::string::const_iterator i = fullString.begin(); i != fullString.end(); i++) {
			if (lSystem2d.get_alphabet().find(*i) != lSystem2d.get_alphabet().end()) {
				tempString += lSystem2d.get_replacement(*i);
			} else {
				tempString += *i;
			}
		}
		
		fullString = tempString;
		times--;
	}

	this->addLinesFromLString(lSystem2d, fullString, color);
}

void LineDrawing::addLinesFromLString (LParser::LSystem2D& lSystem2d, std::string lString, img::Color color) {
	double currentAngle = lSystem2d.get_starting_angle() * 0.0174532925;
	double lSystemAngle = lSystem2d.get_angle() * 0.0174532925;

	double currentX = 0;
	double currentY = 0;

	std::vector <std::vector <double>> positions;

	for(std::string::const_iterator i = lString.begin(); i != lString.end(); i++) {
		char c = *i;
		if (c == '-') {
			currentAngle -= lSystemAngle;
			continue;
		}

		if (c == '+') {
			currentAngle += lSystemAngle;
			continue;
		}

		if (c == '(') {
			std::vector <double> currentPos;

			currentPos.push_back(currentX);
			currentPos.push_back(currentY);
			
			positions.push_back(currentPos);
			continue;
		}

		if (c == ')') {
			if (positions.empty()) {
				std::cerr << "No positions saved!" << std::endl;
				std::cerr << "lSystem string " << lString << std::endl;
				throw;
			}
			
			std::vector <double> lastPos = positions.back();
			currentX = lastPos[0];
			currentY = lastPos[1];

			positions.pop_back();
			continue;
		}

		double nextX = currentX + std::cos(currentAngle);
		double nextY = currentY + std::sin(currentAngle);

		if (lSystem2d.draw(c)) {
			this->addLine(Line2d(Point2d(currentX, currentY), Point2d(nextX, nextY), Color(color.red, color.green, color.blue)));
		}

		currentX = nextX;
		currentY = nextY;
	}
}

void LineDrawing::addLinesFromProjection (Figures3D& figures) {
	for (auto &figure : figures) {
		std::vector<Point2d> projected_points;

		for (auto &point : figure.points) {
			projected_points.push_back(Point2d(point.x / -point.z, point.y / -point.z));
		}

		for (auto &face : figure.faces) {
			for (auto &point_key1 : face.point_indexes) {
				for (auto &point_key2 : face.point_indexes) {
					if (point_key1 != point_key2) {
						Color color = Color(figure.color.red, figure.color.green, figure.color.blue);
						std::cout << "From " << projected_points[point_key1].x << ", " << projected_points[point_key1].y << std::endl;
						std::cout << "To " << projected_points[point_key2].x << ", " << projected_points[point_key2].y << std::endl;
						this->addLine(Line2d(projected_points[point_key1], projected_points[point_key2], color));
					}
				}
			}
		}
	}
}

img::EasyImage LineDrawing::draw2dLines (img::EasyImage& image) {
	for (auto &line : lines) {
		image.draw_line(roundToInt(line.start.x),
		                roundToInt(line.start.y),
		                roundToInt(line.end.x),
		                roundToInt(line.end.y),
		                img::Color(line.color.red, line.color.green, line.color.blue));
	}
	return image;
}

img::EasyImage LineDrawing::draw2dLines (img::EasyImage& image, int size) {
	Point2d max = this->getMaxPoint();
	Point2d min = this->getMinPoint();

	double xRange = max.x - min.x;
	double yRange = max.y - min.y;

	double biggestRange = std::max(xRange, yRange);

	double imageXRange = size * xRange / biggestRange;

	double scaleFactor = 0.95 * imageXRange / xRange;
	this->scaleLines(scaleFactor);

	double dcx = scaleFactor * (max.x + min.x) / 2;
	double dcy = scaleFactor * (max.y + min.y) / 2;

	double dx = image.get_width() / 2 - dcx;
	double dy = image.get_height() / 2 - dcy;
	this->moveLines(dx, dy);

	return this->draw2dLines(image);
}