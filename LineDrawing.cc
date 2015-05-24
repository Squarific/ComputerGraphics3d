#include "EasyImage.h"
#include "lparser.h"
#include "2dhelpers.h"
#include "3dfigures.h"
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
			projected_points.push_back(Point2d(point.x / -point.z, point.y / -point.z, point.z));
		}

		Color color = Color(figure.color.red, figure.color.green, figure.color.blue);
		for (auto &face : figure.faces) {
			int size = face.point_indexes.size();
			for (int i = 0; i < size; i++) {
				this->addLine(Line2d(projected_points[face.point_indexes[i]], projected_points[face.point_indexes[(i + 1) % size]], color));
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

img::EasyImage LineDrawing::draw2dLinesZBuffered (ZBuffer buffer, img::EasyImage& image) {
	for (auto &line : lines) {
		img::Color color = img::Color(line.color.red, line.color.green, line.color.blue);
		unsigned int x0 = roundToInt(line.start.x);
		unsigned int x1 = roundToInt(line.end.x);
		unsigned int y0 = roundToInt(line.start.y);
		unsigned int y1 = roundToInt(line.end.y);

		if (x0 == x1) {
			//special case for x0 == x1
			for (unsigned int i = std::min(y0, y1); i <= std::max(y0, y1); i++) {
				// Calculate Z Value
				double oneOverZ;
				if (std::max(y0, y1) - std::min(y0, y1) == 0) {
					// Neither x nor y changes, so just use the biggest Z
					oneOverZ = 1 / std::max(line.start.z, line.end.z);
				} else {
					double p = (i - std::min(y0, y1)) / (std::max(y0, y1) - std::min(y0, y1));
					if (line.start.y == std::min(y0, y1)) {
						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
					} else {
						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
					}
				}
				if (buffer[x0][i] > oneOverZ) {
					image(x0, i) = color;
					buffer[x0][i] = oneOverZ;
				}
			}
		} else if (y0 == y1) {
			//special case for y0 == y1
			for (unsigned int i = std::min(x0, x1); i <= std::max(x0, x1); i++) {
				double oneOverZ;
				double p = (i - std::min(x0, x1)) / (std::max(x0, x1) - std::min(x0, x1));
				if (line.start.x == std::min(x0, x1)) {
					oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
				} else {
					oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
				}
				if (buffer[i][y0] > oneOverZ) {
					image(i, y0) = color;
					buffer[i][y0] = oneOverZ;
				}
			}
		} else {
			if (x0 > x1) {
				//flip points if x1>x0: we want x0 to have the lowest value
				std::swap(x0, x1);
				std::swap(y0, y1);
			}
			double m = ((double) y1 - (double) y0) / ((double) x1 - (double) x0);
			if (-1.0 <= m && m <= 1.0) {
				for (unsigned int i = 0; i <= (x1 - x0); i++) {
					double oneOverZ;
					double p = i / (x1 - x0);
					if (line.start.x == x0) {
						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
					} else {
						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
					}
					if (buffer[x0 + i][(unsigned int) round(y0 + m * i)] > oneOverZ) {
						image(x0 + i, (unsigned int) round(y0 + m * i)) = color;
						buffer[x0 + i][(unsigned int) round(y0 + m * i)] = oneOverZ;
					}
				}
			} else if (m > 1.0) {
				for (unsigned int i = 0; i <= (y1 - y0); i++) {
					double oneOverZ;
					double p = i / (y1 - y0);
					if (line.start.y == y0) {
						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
					} else {
						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
					}
					if (buffer[(unsigned int) round(x0 + (i / m))][y0 + i] > oneOverZ) {
						image((unsigned int) round(x0 + (i / m)), y0 + i) = color;
						buffer[(unsigned int) round(x0 + (i / m))][y0 + i] = oneOverZ;
					}
				}
			}
			else if (m < -1.0) {
				for (unsigned int i = 0; i <= (y0 - y1); i++) {
					double oneOverZ;
					double p = i / (y0 - y1);
					if (line.start.y == y1) {
						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
					} else {
						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
					}
					if (buffer[(unsigned int) round(x0 - (i / m))][y0 - i] > oneOverZ) {
						image((unsigned int) round(x0 - (i / m)), y0 - i) = color;
						buffer[(unsigned int) round(x0 - (i / m))][y0 - i] = oneOverZ;
					}
				}
			}
		}
	}
	return image;
}

img::EasyImage LineDrawing::draw2dLinesZBuffered (img::EasyImage& image, int size) {
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

	return this->draw2dLinesZBuffered(ZBuffer(size, size), image);
}