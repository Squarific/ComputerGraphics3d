#include "EasyImage.h"
#include "lparser.h"
#include "2dhelpers.h"
#include "3dfigures.h"
#include "LineDrawing.h"
#include "TriangleDrawing.h"
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
		for (auto &point : triangle.points) {
			if (max.x < point.x) max.x = point.x;
			if (max.y < point.y) max.y = point.y;
		}
	}
	return max;
}

Point2d TriangleDrawing::getMinPoint () {
	Point2d min = Point2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle.points) {
			if (min.x > point.x) min.x = point.x;
			if (min.y > point.y) min.y = point.y;
		}
	}
	return min;
}

void TriangleDrawing::scaleTriangles (double scaleFactor) {
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle.points) {
			point.x *= scaleFactor;
			point.y *= scaleFactor;
		}
	}
}

void TriangleDrawing::moveTriangles (double dx, double dy) {
	for (auto &triangle : this->triangles) {
		for (auto &point : triangle.points) {
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
			int size = face.point_indexes.size();
			for (int i = 1; i < size - 1; i ++) {
				this->addTriangle(Triangle(
					std::vector<Point2d> {projected_points[face.point_indexes[i]],
						                  projected_points[face.point_indexes[(i + 1) % size]],
						                  projected_points[face.point_indexes[0]]},
					color
				));
			}
		}
	}
}

img::EasyImage TriangleDrawing::drawTriangles (img::EasyImage& image) {
	for (auto &triangle : triangles) {
		// Bounding box
		int minY = roundToInt(std::min(std::min(triangle.points[0].y, triangle.points[1].y), triangle.points[2].y) + 0.5);
		int maxY = roundToInt(std::max(std::max(triangle.points[0].y, triangle.points[1].y), triangle.points[2].y) - 0.5);

		Point2d temp;
		img::Color color = img::Color(triangle.color.red, triangle.color.green, triangle.color.blue);
		for (temp.y = minY; temp.y <= maxY; temp.y++) {
			double xABL = std::numeric_limits<double>::infinity();
			double xACL = std::numeric_limits<double>::infinity();
			double xBCL = std::numeric_limits<double>::infinity();
			double xABR = -std::numeric_limits<double>::infinity();
			double xACR = -std::numeric_limits<double>::infinity();
			double xBCR = -std::numeric_limits<double>::infinity();

			// p0 to p1
			if ((temp.y - triangle.points[0].y) * (temp.y - triangle.points[1].y) <= 0 && triangle.points[0].y != triangle.points[1].y) {
				double xI = triangle.points[0].x + (triangle.points[1].x - triangle.points[0].x) * (temp.y - triangle.points[0].y) / (triangle.points[1].y - triangle.points[0].y);
				xABL = xI;
				xABR = xI;
			}

			// p1 to p2
			if ((temp.y - triangle.points[1].y) * (temp.y - triangle.points[2].y) <= 0 && triangle.points[1].y != triangle.points[2].y) {
				double xI = triangle.points[1].x + (triangle.points[2].x - triangle.points[1].x) * (temp.y - triangle.points[1].y) / (triangle.points[2].y - triangle.points[1].y);
				xACL = xI;
				xACR = xI;
			}

			// p2 to p0
			if ((temp.y - triangle.points[2].y) * (temp.y - triangle.points[0].y) <= 0 && triangle.points[2].y != triangle.points[0].y) {
				double xI = triangle.points[2].x + (triangle.points[0].x - triangle.points[2].x) * (temp.y - triangle.points[2].y) / (triangle.points[0].y - triangle.points[2].y);
				xBCL = xI;
				xBCR = xI;
			}

			int xL = roundToInt(std::min(xABL, std::min(xACL, xBCL)) + 0.5);
			int xR = roundToInt(std::max(xABR, std::max(xACR, xBCR)) - 0.5);

			for (temp.x = xL; temp.x <= xR; temp.x++) {
				image(temp.x, temp.y) = color;
			}
		}
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

img::EasyImage TriangleDrawing::drawTrianglesZBuffered (double d, ZBuffer buffer, img::EasyImage& image) {
	for (auto &triangle : triangles) {
		// Bounding box
		int minY = roundToInt(std::min(std::min(triangle.points[0].y, triangle.points[1].y), triangle.points[2].y) + 0.5);
		int maxY = roundToInt(std::max(std::max(triangle.points[0].y, triangle.points[1].y), triangle.points[2].y) - 0.5);

		Point2d middle = Point2d((triangle.points[0].x + triangle.points[1].x + triangle.points[2].x) / 3,
		                         (triangle.points[0].y + triangle.points[1].y + triangle.points[2].y) / 3,
		                         1 / (3 * triangle.points[0].z) + 1 / (3 * triangle.points[1].z) + 1 / (3 * triangle.points[2].z));

		Point2d temp;
		img::Color color = img::Color(triangle.color.red, triangle.color.green, triangle.color.blue);
		for (temp.y = minY; temp.y <= maxY; temp.y++) {
			double xABL = std::numeric_limits<double>::infinity();
			double xACL = std::numeric_limits<double>::infinity();
			double xBCL = std::numeric_limits<double>::infinity();
			double xABR = -std::numeric_limits<double>::infinity();
			double xACR = -std::numeric_limits<double>::infinity();
			double xBCR = -std::numeric_limits<double>::infinity();

			// p0 to p1
			if ((temp.y - triangle.points[0].y) * (temp.y - triangle.points[1].y) <= 0 && triangle.points[0].y != triangle.points[1].y) {
				double xI = triangle.points[0].x + (triangle.points[1].x - triangle.points[0].x) * (temp.y - triangle.points[0].y) / (triangle.points[1].y - triangle.points[0].y);
				xABL = xI;
				xABR = xI;
			}

			// p1 to p2
			if ((temp.y - triangle.points[1].y) * (temp.y - triangle.points[2].y) <= 0 && triangle.points[1].y != triangle.points[2].y) {
				double xI = triangle.points[1].x + (triangle.points[2].x - triangle.points[1].x) * (temp.y - triangle.points[1].y) / (triangle.points[2].y - triangle.points[1].y);
				xACL = xI;
				xACR = xI;
			}

			// p2 to p0
			if ((temp.y - triangle.points[2].y) * (temp.y - triangle.points[0].y) <= 0 && triangle.points[2].y != triangle.points[0].y) {
				double xI = triangle.points[2].x + (triangle.points[0].x - triangle.points[2].x) * (temp.y - triangle.points[2].y) / (triangle.points[0].y - triangle.points[2].y);
				xBCL = xI;
				xBCR = xI;
			}

			int xL = roundToInt(std::min(xABL, std::min(xACL, xBCL)) + 0.5);
			int xR = roundToInt(std::max(xABR, std::max(xACR, xBCR)) - 0.5);

			for (temp.x = xL; temp.x <= xR; temp.x++) {
				Vector3D u = Vector3D::vector(triangle.points[1].x - triangle.points[0].x, triangle.points[1].y - triangle.points[0].y, triangle.points[1].z - triangle.points[0].z);
				Vector3D v = Vector3D::vector(triangle.points[2].x - triangle.points[0].x, triangle.points[2].y - triangle.points[0].y, triangle.points[2].z - triangle.points[0].z);

				double w1 = u.y * v.z - u.z * v.y;
				double w2 = u.z * v.x - u.x * v.z;
				double w3 = u.x * v.y - u.y * v.x;

				double k = w1 * triangle.points[0].x + w2 * triangle.points[0].y + w3 * triangle.points[0].z;
				double dzdx = w1 / (-k * d);
				double dzdy = w2 / (-k * d);

				double oneOverZ = 1.0001 * middle.z + (temp.x - middle.x) * dzdx + (temp.y - middle.y) * dzdy;
				if (buffer[temp.x][temp.y] > oneOverZ) {
					image(temp.x, temp.y) = color;
					buffer[temp.x][temp.y] = oneOverZ;
				}
			}
		}
	}
	return image;
}

img::EasyImage TriangleDrawing::drawTrianglesZBuffered (img::EasyImage& image, int size) {
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

	return this->drawTrianglesZBuffered(scaleFactor, ZBuffer(size, size), image);
}