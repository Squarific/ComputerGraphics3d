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

void TriangleDrawing::addTriangle (Triangle triangle) {
	this->triangles.push_back(triangle);
};

Point2d TriangleDrawing::getMaxPoint () {
	Point2d max = Point2d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle) {
			if (max.x < point.x) max.x = point.x;
			if (max.y < point.y) max.y = point.y;
		}
	}
	return max;
}

Point2d TriangleDrawing::getMinPoint () {
	Point2d min = Point2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle) {
			if (min.x > point.x) min.x = point.x;
			if (min.y > point.y) min.y = point.y;
		}
	}
	return min;
}

void TriangleDrawing::scaleTriangles (double scaleFactor) {
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle) {
			point.x *= scaleFactor;
			point.y *= scaleFactor;
		}
	}
}

void TriangleDrawing::moveTriangles (double dx, double dy) {
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle) {
			point.x += dx;
			point.y += dy;
		}
	}
}

void TriangleDrawing::addTrianglesFromProjection (Figures3D& figures) {
	for (auto &figure : figures) {
		std::vector<Point2d> projected_points;

		for (auto &point : figure.points) {
			projected_points.push_back(Point2d(point.x / -point.z, point.y / -point.z, point.z));
		}

		Color color = Color(figure.color.red, figure.color.green, figure.color.blue);
		for (auto &face : figure.faces) {
			int size = face.point_indexes.size() - 1;
			for (int i = 0; i < size; i += 2) {
				this->addTriangle(Triangle(
					std::vector<Point2d> {projected_points[face.point_indexes[i]],
						                  projected_points[face.point_indexes[(i + 1) % size]],
						                  projected_points[face.point_indexes[(i + 2) % size]]},
					color
				));
			}
		}
	}
}

img::EasyImage TriangleDrawing::drawTriangles (img::EasyImage& image) {
	for (auto &triangle : triangles) {
		int minX = roundToInt(std::min(std::min(triangle.points[0], triangle.points[1]), triangle.points[2]));
		int minY = roundToInt(std::min(std::min(triangle.points[0], triangle.points[1]), triangle.points[2]));
		int maxX = roundToInt(std::min(std::min(triangle.points[0], triangle.points[1]), triangle.points[2]));
		int minX = roundToInt(std::min(std::min(triangle.points[0], triangle.points[1]), triangle.points[2]));
	}
	return image;
}

img::EasyImage TriangleDrawing::drawTriangles (img::EasyImage& image, int size) {
	Point2d max = this->getMaxPoint();
	Point2d min = this->getMinPoint();

	double xRange = max.x - min.x;
	double yRange = max.y - min.y;

	double biggestRange = std::max(xRange, yRange);

	double imageXRange = size * xRange / biggestRange;

	double scaleFactor = 0.95 * imageXRange / xRange;
	this->scaleTriangles(scaleFactor);

	double dcx = scaleFactor * (max.x + min.x) / 2;
	double dcy = scaleFactor * (max.y + min.y) / 2;

	double dx = image.get_width() / 2 - dcx;
	double dy = image.get_height() / 2 - dcy;
	this->moveTriangles(dx, dy);

	return this->drawTriangles(image);
}

// img::EasyImage LineDrawing::draw2dLinesZBuffered (ZBuffer buffer, img::EasyImage& image) {
// 	for (auto &line : lines) {
// 		img::Color color = img::Color(line.color.red, line.color.green, line.color.blue);
// 		unsigned int x0 = roundToInt(line.start.x);
// 		unsigned int x1 = roundToInt(line.end.x);
// 		unsigned int y0 = roundToInt(line.start.y);
// 		unsigned int y1 = roundToInt(line.end.y);

// 		if (x0 == x1) {
// 			//special case for x0 == x1
// 			for (unsigned int i = std::min(y0, y1); i <= std::max(y0, y1); i++) {
// 				// Calculate Z Value
// 				double oneOverZ;
// 				if (std::max(y0, y1) - std::min(y0, y1) == 0) {
// 					// Neither x nor y changes, so just use the biggest Z
// 					oneOverZ = 1 / std::max(line.start.z, line.end.z);
// 				} else {
// 					double p = (i - std::min(y0, y1)) / (std::max(y0, y1) - std::min(y0, y1));
// 					if (line.start.y == std::min(y0, y1)) {
// 						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
// 					} else {
// 						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
// 					}
// 				}
// 				if (buffer[x0][i] > oneOverZ) {
// 					image(x0, i) = color;
// 					buffer[x0][i] = oneOverZ;
// 				}
// 			}
// 		} else if (y0 == y1) {
// 			//special case for y0 == y1
// 			for (unsigned int i = std::min(x0, x1); i <= std::max(x0, x1); i++) {
// 				double oneOverZ;
// 				double p = (i - std::min(x0, x1)) / (std::max(x0, x1) - std::min(x0, x1));
// 				if (line.start.x == std::min(x0, x1)) {
// 					oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
// 				} else {
// 					oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
// 				}
// 				if (buffer[i][y0] > oneOverZ) {
// 					image(i, y0) = color;
// 					buffer[i][y0] = oneOverZ;
// 				}
// 			}
// 		} else {
// 			if (x0 > x1) {
// 				//flip points if x1>x0: we want x0 to have the lowest value
// 				std::swap(x0, x1);
// 				std::swap(y0, y1);
// 			}
// 			double m = ((double) y1 - (double) y0) / ((double) x1 - (double) x0);
// 			if (-1.0 <= m && m <= 1.0) {
// 				for (unsigned int i = 0; i <= (x1 - x0); i++) {
// 					double oneOverZ;
// 					double p = i / (x1 - x0);
// 					if (line.start.x == x0) {
// 						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
// 					} else {
// 						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
// 					}
// 					if (buffer[x0 + i][(unsigned int) round(y0 + m * i)] > oneOverZ) {
// 						image(x0 + i, (unsigned int) round(y0 + m * i)) = color;
// 						buffer[x0 + i][(unsigned int) round(y0 + m * i)] = oneOverZ;
// 					}
// 				}
// 			} else if (m > 1.0) {
// 				for (unsigned int i = 0; i <= (y1 - y0); i++) {
// 					double oneOverZ;
// 					double p = i / (y1 - y0);
// 					if (line.start.y == y0) {
// 						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
// 					} else {
// 						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
// 					}
// 					if (buffer[(unsigned int) round(x0 + (i / m))][y0 + i] > oneOverZ) {
// 						image((unsigned int) round(x0 + (i / m)), y0 + i) = color;
// 						buffer[(unsigned int) round(x0 + (i / m))][y0 + i] = oneOverZ;
// 					}
// 				}
// 			}
// 			else if (m < -1.0) {
// 				for (unsigned int i = 0; i <= (y0 - y1); i++) {
// 					double oneOverZ;
// 					double p = i / (y0 - y1);
// 					if (line.start.y == y1) {
// 						oneOverZ = 1 / ((line.end.z - line.start.z) * p + line.start.z);
// 					} else {
// 						oneOverZ = 1 / ((line.start.z - line.end.z) * p + line.end.z);
// 					}
// 					if (buffer[(unsigned int) round(x0 - (i / m))][y0 - i] > oneOverZ) {
// 						image((unsigned int) round(x0 - (i / m)), y0 - i) = color;
// 						buffer[(unsigned int) round(x0 - (i / m))][y0 - i] = oneOverZ;
// 					}
// 				}
// 			}
// 		}
// 	}
// 	return image;
// }

// img::EasyImage LineDrawing::draw2dLinesZBuffered (img::EasyImage& image, int size) {
// 	Point2d max = this->getMaxPoint();
// 	Point2d min = this->getMinPoint();

// 	double xRange = max.x - min.x;
// 	double yRange = max.y - min.y;

// 	double biggestRange = std::max(xRange, yRange);

// 	double imageXRange = size * xRange / biggestRange;

// 	double scaleFactor = 0.95 * imageXRange / xRange;
// 	this->scaleLines(scaleFactor);

// 	double dcx = scaleFactor * (max.x + min.x) / 2;
// 	double dcy = scaleFactor * (max.y + min.y) / 2;

// 	double dx = image.get_width() / 2 - dcx;
// 	double dy = image.get_height() / 2 - dcy;
// 	this->moveLines(dx, dy);

// 	return this->draw2dLinesZBuffered(ZBuffer(size, size), image);
// }