#include "vector.hh"
#include "3dfigures.h"
#include "EasyImage.h"
#include "vectorHelpers.h"
#include "ini_configuration.hh"
#include <string>
#include <cmath>

inline int roundToInt(double d) {
    return d < 0 ? 
        std::ceil(d-0.5):
        std::floor(d+0.5);
}

void Figure::applyTransformation (Matrix const& t) {
	for (auto &point : this->points) {
		point *= t;
	}
}

Figure figure3dFromConfig (ini::Section const& figureConfig) {
	double scale = figureConfig["scale"].as_double_or_die();

	double rotateX = figureConfig["rotateX"].as_double_or_die();
	double rotateY = figureConfig["rotateY"].as_double_or_die();
	double rotateZ = figureConfig["rotateZ"].as_double_or_die();

	std::vector<double> center = figureConfig["center"].as_double_tuple_or_die();
	Vector3D centerVector = Vector3D::vector(center[0], center[1], center[2]);

	VectorHelpers vh;
	Matrix scaleMatrix = vh.scaleMatrix(scale);
	Matrix rotateMatrixX = vh.rotXMatrix(rotateX);
	Matrix rotateMatrixY = vh.rotYMatrix(rotateY);
	Matrix rotateMatrixZ = vh.rotZMatrix(rotateZ);
	Matrix centerMatrix = vh.translateMatrix(centerVector);

	Matrix allMatrix = scaleMatrix * rotateMatrixX * rotateMatrixY * rotateMatrixZ * centerMatrix;

	std::vector<double> color = figureConfig["color"].as_double_tuple_or_die();

	int nrPoints = figureConfig["nrPoints"].as_int_or_die();
	int nrLines = figureConfig["nrLines"].as_int_or_die();

	Figure figure;

	int currentPoint = 0;
	while (currentPoint < nrPoints) {
		std::vector<double> coords = figureConfig[(std::string("point") + std::to_string(currentPoint)).c_str()].as_double_tuple_or_die();
		
		Vector3D point = Vector3D::point(coords[0], coords[1], coords[2]);
		std::cout << coords[0] << ", " << coords[1] << ", " << coords[2] << std::endl;
		point *= allMatrix;

		figure.points.push_back(point);
		currentPoint++;
	}

	int currentLine = 0;
	while (currentLine < nrLines) {
		std::vector<int> points = figureConfig[(std::string("line") + std::to_string(currentPoint)).c_str()].as_int_tuple_or_die();

		Face face;
		face.point_indexes.push_back(points[0]);
		face.point_indexes.push_back(points[1]);

		figure.faces.push_back(face);
		currentLine++;
	}

	figure.color = img::Color(roundToInt(color[0] * 255), roundToInt(color[1] * 255), roundToInt(color[2] * 255));

	return figure;
}