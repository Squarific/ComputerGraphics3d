#include "vector.hh"
#include "3dfigures.h"
#include "lparser.h"
#include "EasyImage.h"
#include "vectorHelpers.h"
#include "ini_configuration.hh"
#include <string>
#include <fstream>
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

Figure createCube () {
	Figure cube;

	cube.points.push_back(Vector3D::point(0, 0, 0));

	cube.points.push_back(Vector3D::point( 1, -1, -1));
	cube.points.push_back(Vector3D::point(-1,  1, -1));
	cube.points.push_back(Vector3D::point( 1,  1,  1));
	cube.points.push_back(Vector3D::point(-1, -1,  1));

	cube.points.push_back(Vector3D::point( 1,  1, -1));
	cube.points.push_back(Vector3D::point(-1, -1, -1));
	cube.points.push_back(Vector3D::point( 1, -1,  1));
	cube.points.push_back(Vector3D::point(-1,  1,  1));

	cube.faces.push_back(Face(std::vector<int> {1, 5, 3, 7}));
	cube.faces.push_back(Face(std::vector<int> {5, 2, 8, 3}));
	cube.faces.push_back(Face(std::vector<int> {2, 6, 4, 8}));

	cube.faces.push_back(Face(std::vector<int> {6, 1, 7, 4}));
	cube.faces.push_back(Face(std::vector<int> {7, 3, 8, 4}));
	cube.faces.push_back(Face(std::vector<int> {1, 6, 2, 5}));

	return cube;
}

Figure createTetrahedron () {
	Figure tetrahedron;

	tetrahedron.points.push_back(Vector3D::point(0, 0, 0));

	tetrahedron.points.push_back(Vector3D::point( 1, -1, -1));
	tetrahedron.points.push_back(Vector3D::point(-1,  1, -1));
	tetrahedron.points.push_back(Vector3D::point( 1,  1,  1));
	tetrahedron.points.push_back(Vector3D::point(-1, -1,  1));

	tetrahedron.faces.push_back(Face(std::vector<int> {1, 2, 3}));
	tetrahedron.faces.push_back(Face(std::vector<int> {2, 4, 3}));
	tetrahedron.faces.push_back(Face(std::vector<int> {1, 4, 2}));
	tetrahedron.faces.push_back(Face(std::vector<int> {1, 3, 4}));

	return tetrahedron;
}

Figure createOctahedron () {
	Figure octahedron;

	octahedron.points.push_back(Vector3D::point(0, 0, 0));

	octahedron.points.push_back(Vector3D::point( 1,  0,  0));
	octahedron.points.push_back(Vector3D::point( 0,  1,  0));
	octahedron.points.push_back(Vector3D::point(-1,  0,  0));

	octahedron.points.push_back(Vector3D::point( 0, -1,  0));
	octahedron.points.push_back(Vector3D::point( 0,  0, -1));
	octahedron.points.push_back(Vector3D::point( 0,  0,  1));

	octahedron.faces.push_back(Face(std::vector<int> {1, 2, 6}));
	octahedron.faces.push_back(Face(std::vector<int> {2, 3, 6}));
	octahedron.faces.push_back(Face(std::vector<int> {3, 4, 6}));
	octahedron.faces.push_back(Face(std::vector<int> {4, 1, 6}));

	octahedron.faces.push_back(Face(std::vector<int> {2, 1, 5}));
	octahedron.faces.push_back(Face(std::vector<int> {3, 2, 5}));
	octahedron.faces.push_back(Face(std::vector<int> {4, 3, 5}));
	octahedron.faces.push_back(Face(std::vector<int> {1, 4, 5}));

	return octahedron;
}

Figure createIcosahedron () {
	Figure icosahedron;

	icosahedron.points.push_back(Vector3D::point(0, 0, 0));
	icosahedron.points.push_back(Vector3D::point(0, 0, std::sqrt(5) / 2));

	for (int i = 2; i < 7; i++) {
		double angle = (i - 2) * 2 * M_PI / 5;
		icosahedron.points.push_back(Vector3D::point(std::cos(angle), std::sin(angle), 0.5));
	}

	for (int i = 7; i < 12; i++) {
		double angle = M_PI / 5 + (i - 7) * 2 * M_PI / 5;
		icosahedron.points.push_back(Vector3D::point(std::cos(angle), std::sin(angle), -0.5));
	}

	icosahedron.points.push_back(Vector3D::point(0, 0, -std::sqrt(5) / 2));

	icosahedron.faces.push_back(Face(std::vector<int> {1, 2, 3}));
	icosahedron.faces.push_back(Face(std::vector<int> {1, 3, 4}));
	icosahedron.faces.push_back(Face(std::vector<int> {1, 4, 5}));
	icosahedron.faces.push_back(Face(std::vector<int> {1, 5, 6}));

	icosahedron.faces.push_back(Face(std::vector<int> {1, 6, 2}));
	icosahedron.faces.push_back(Face(std::vector<int> {2, 7, 3}));
	icosahedron.faces.push_back(Face(std::vector<int> {3, 7, 8}));
	icosahedron.faces.push_back(Face(std::vector<int> {3, 8, 4}));
	
	icosahedron.faces.push_back(Face(std::vector<int> {4,  8,  9}));
	icosahedron.faces.push_back(Face(std::vector<int> {4,  9,  5}));
	icosahedron.faces.push_back(Face(std::vector<int> {5,  9, 10}));
	icosahedron.faces.push_back(Face(std::vector<int> {5, 10,  6}));
	
	icosahedron.faces.push_back(Face(std::vector<int> {6,  10, 11}));
	icosahedron.faces.push_back(Face(std::vector<int> {6,  11,  2}));
	icosahedron.faces.push_back(Face(std::vector<int> {2,  11,  7}));
	icosahedron.faces.push_back(Face(std::vector<int> {12,  8,  7}));
	
	icosahedron.faces.push_back(Face(std::vector<int> {12,  9,  8}));
	icosahedron.faces.push_back(Face(std::vector<int> {12, 10,  9}));
	icosahedron.faces.push_back(Face(std::vector<int> {12, 11, 10}));
	icosahedron.faces.push_back(Face(std::vector<int> {12,  7, 11}));

	return icosahedron;
}

Figure createDodecahedron () {
	Figure icosahedron = createIcosahedron();
	Figure dodecahedron;

	dodecahedron.points.push_back(Vector3D::point(0, 0, 0));

	for (Face& face : icosahedron.faces) {
		dodecahedron.points.push_back((icosahedron.points[face.point_indexes[0]] + icosahedron.points[face.point_indexes[1]] + icosahedron.points[face.point_indexes[2]]) / 3);
	}

	dodecahedron.faces.push_back(Face(std::vector<int> {1,  2,  3,  4, 5}));
	dodecahedron.faces.push_back(Face(std::vector<int> {1,  6,  7,  8, 2}));
	dodecahedron.faces.push_back(Face(std::vector<int> {2,  8,  9, 10, 3}));
	dodecahedron.faces.push_back(Face(std::vector<int> {3, 10, 11, 12, 4}));

	dodecahedron.faces.push_back(Face(std::vector<int> { 4, 12, 13, 14,  5}));
	dodecahedron.faces.push_back(Face(std::vector<int> { 5, 14, 15,  6,  1}));
	dodecahedron.faces.push_back(Face(std::vector<int> {20, 19, 18, 17, 16}));
	dodecahedron.faces.push_back(Face(std::vector<int> {20, 15, 14, 13, 19}));

	dodecahedron.faces.push_back(Face(std::vector<int> {19, 13, 12, 11, 18}));
	dodecahedron.faces.push_back(Face(std::vector<int> {18, 11, 10,  9, 17}));
	dodecahedron.faces.push_back(Face(std::vector<int> {17,  9,  8,  7, 16}));
	dodecahedron.faces.push_back(Face(std::vector<int> {16,  7,  6, 15, 20}));

	return dodecahedron;
}

Figure createSphere (double radius, int n) {
	Figure sphere = createIcosahedron();

	while (n > 0) {
		n--;

		std::vector<Face> new_faces;
		std::vector<Vector3D> new_points = sphere.points;

		for (Face& face : sphere.faces) {
			Vector3D A = sphere.points[face.point_indexes[0]];
			int A_index = face.point_indexes[0];

			Vector3D B = sphere.points[face.point_indexes[1]];
			int B_index = face.point_indexes[1];

			Vector3D C = sphere.points[face.point_indexes[2]];
			int C_index = face.point_indexes[2];

			Vector3D D = (A + B) / 2;
			int D_index = new_points.size();
			new_points.push_back(D);

			Vector3D E = (A + C) / 2;
			int E_index = new_points.size();
			new_points.push_back(E);
			
			Vector3D F = (B + C) / 2;
			int F_index = new_points.size();
			new_points.push_back(F);

			new_faces.push_back(Face(std::vector<int> {A_index, D_index, E_index}));
			new_faces.push_back(Face(std::vector<int> {B_index, F_index, D_index}));
			new_faces.push_back(Face(std::vector<int> {C_index, E_index, F_index}));
			new_faces.push_back(Face(std::vector<int> {D_index, F_index, E_index}));
		}

		sphere.faces = new_faces;
		sphere.points = new_points;
	}

	for (Vector3D& point : sphere.points) {
		point.normalise();
		point /= radius;
	}

	return sphere;
}

Figure createCone (int n, double h) {
	Figure cone;

	for (int i = 0; i < n; i++) {
		double angle = 2 * i * M_PI / n;
		cone.points.push_back(Vector3D::point(std::cos(angle), std::sin(angle), 0));
	}

	cone.points.push_back(Vector3D::point(0, 0, h));

	for (int i = 0; i < n; i++) {
		cone.faces.push_back(Face(std::vector <int> {i, (i + 1) % n, n}));
	}

	std::vector <int> points;
	for (int i = 0; i < n; i++) {
		points.push_back(i);
	}
	cone.faces.push_back(Face(points));

	return cone;
}

Figure createCylinder (int n, double h) {
	Figure cylinder;

	for (int i = 0; i < n; i++) {
		double angle = 2 * i * M_PI / n;
		cylinder.points.push_back(Vector3D::point(std::cos(angle), std::sin(angle), 0));
	}

	for (int i = 0; i < n; i++) {
		double angle = 2 * i * M_PI / n;
		cylinder.points.push_back(Vector3D::point(std::cos(angle), std::sin(angle), h));
	}

	for (int i = 0; i < n; i++) {
		cylinder.faces.push_back(Face(std::vector <int> {i, (i + 1) % n, ((i + 1) % n) + n, i + n}));
	}

	std::vector <int> points;
	for (int i = 0; i < n; i++) {
		points.push_back(i);
	}
	cylinder.faces.push_back(Face(points));

	std::vector <int> points2;
	for (int i = n; i < 2 * n; i++) {
		points2.push_back(i);
	}
	cylinder.faces.push_back(Face(points2));

	return cylinder;
}

Figure createTorus (double r, double R, int n, int m) {
	Figure torus;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			double u = 2 * i * M_PI / n;
			double v = 2 * j * M_PI / m;

			double x = (R + r * std::cos(v)) * std::cos(u);
			double y = (R + r * std::cos(v)) * std::sin(u);
			double z = r * std::sin(v);

			torus.points.push_back(Vector3D::point(x, y, z));
		}
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			torus.faces.push_back(Face(std::vector <int> {i * m + j, ((i + 1) % n) * m + j, ((i + 1) % n) * m + ((j + 1) % m), i * m + ((j + 1) % m)}));
		}
	}	

	return torus;
}

Figure figure3dFromLineDrawingConfig (ini::Section const& figureConfig) {
	int nrPoints = figureConfig["nrPoints"].as_int_or_die();
	int nrLines = figureConfig["nrLines"].as_int_or_die();

	Figure figure;

	int currentPoint = 0;
	while (currentPoint < nrPoints) {
		std::vector<double> coords = figureConfig[(std::string("point") + std::to_string(currentPoint)).c_str()].as_double_tuple_or_die();
		figure.points.push_back(Vector3D::point(coords[0], coords[1], coords[2]));
		currentPoint++;
	}

	int currentLine = 0;
	while (currentLine < nrLines) {
		std::vector<int> points = figureConfig[(std::string("line") + std::to_string(currentLine)).c_str()].as_int_tuple_or_die();
		figure.faces.push_back(Face(std::vector <int> {points[0], points[1]}));
		currentLine++;
	}

	return figure;
}

void addLinesFromLString (Figure& l3dsystemFigure, LParser::LSystem3D l_system, std::string fullString) {
	Vector3D H = Vector3D::vector(1, 0, 0);
	Vector3D L = Vector3D::vector(0, 1, 0);
	Vector3D U = Vector3D::vector(0, 0, 1);

	Vector3D position = Vector3D::point(0, 0, 0);
	std::vector <Vector3D> positions;

	double lSystemAngle = l_system.get_angle() * 0.0174532925;

	//std::cout << fullString << std::endl;

	for(std::string::const_iterator i = fullString.begin(); i != fullString.end(); i++) {
		char c = *i;
		//std::cout << c << " ";
		if (c == '-') {
			H =  H * std::cos(-lSystemAngle) + L * std::sin(-lSystemAngle);
			L = -H * std::sin(-lSystemAngle) + L * std::cos(-lSystemAngle);
			continue;
		}

		if (c == '+') {
			H =  H * std::cos(lSystemAngle) + L * std::sin(lSystemAngle);
			L = -H * std::sin(lSystemAngle) + L * std::cos(lSystemAngle);
			continue;
		}

		if (c == '^') {
			H =  H * std::cos(lSystemAngle) + U * std::sin(lSystemAngle);
			U = -H * std::sin(lSystemAngle) + U * std::cos(lSystemAngle);
			continue;
		}

		if (c == '&') {
			H =  H * std::cos(-lSystemAngle) + U * std::sin(-lSystemAngle);
			U = -H * std::sin(-lSystemAngle) + U * std::cos(-lSystemAngle);
			continue;
		}

		if (c == '\\') {
			L = L * std::cos(lSystemAngle) - U * std::sin(lSystemAngle);
			U = L * std::sin(lSystemAngle) + U * std::cos(lSystemAngle);
			continue;
		}

		if (c == '/') {
			L = L * std::cos(-lSystemAngle) - U * std::sin(-lSystemAngle);
			U = L * std::sin(-lSystemAngle) + U * std::cos(-lSystemAngle);
			continue;
		}

		if (c == '|') {
			H = -H;
			L = -L;
			continue;
		}

		if (c == '(') {
			positions.push_back(position);
			continue;
		}

		if (c == ')') {
			if (positions.empty()) {
				std::cerr << "No positions saved!" << std::endl;
				std::cerr << "lSystem string " << fullString << std::endl;
				throw;
			}
			
			position = positions.back();
			positions.pop_back();
			continue;
		}

		Vector3D nextPos = position + H;

		if (l_system.draw(c)) {
			int index = l3dsystemFigure.points.size();
			l3dsystemFigure.points.push_back(position);
			l3dsystemFigure.points.push_back(nextPos);

			//std::cout << "Adding line from " << position << " to " << nextPos << std::endl;

			l3dsystemFigure.faces.push_back(Face(std::vector<int> {index, index + 1}));
		}

		position = nextPos;
	}
}

Figure createFrom3DLSystem (std::string inputfile) {
	Figure l3dsystemFigure;
	LParser::LSystem3D l_system;

	std::ifstream input_stream(inputfile.c_str());
    input_stream >> l_system;
    input_stream.close();

    int times = l_system.get_nr_iterations();
	std::string fullString = l_system.get_initiator();

	while (times > 0) {
		std::string tempString = "";
		
		for(std::string::const_iterator i = fullString.begin(); i != fullString.end(); i++) {
			if (l_system.get_alphabet().find(*i) != l_system.get_alphabet().end()) {
				tempString += l_system.get_replacement(*i);
			} else {
				tempString += *i;
			}
		}
		
		fullString = tempString;
		times--;
	}

	addLinesFromLString(l3dsystemFigure, l_system, fullString);

	return l3dsystemFigure;
}

Figure figure3dFromConfig (ini::Section const& figureConfig) {
	Figure figure;
	std::string type = figureConfig["type"].as_string_or_die();

	if (type == std::string("LineDrawing")) {
		figure = figure3dFromLineDrawingConfig(figureConfig);
	} else if (type == std::string("Cube")) {
		figure = createCube();
	} else if (type == std::string("Tetrahedron")) {
		figure = createTetrahedron();
	} else if (type == std::string("Octahedron")) {
		figure = createOctahedron();
	} else if (type == std::string("Icosahedron")) {
		figure = createIcosahedron();
	} else if (type == std::string("Dodecahedron")) {
		figure = createDodecahedron();
	} else if (type == std::string("Cone")) {
		figure = createCone(figureConfig["n"].as_int_or_die(), figureConfig["height"].as_double_or_die());
	} else if (type == std::string("Cylinder")) {
		figure = createCylinder(figureConfig["n"].as_int_or_die(), figureConfig["height"].as_double_or_die());
	} else if (type == std::string("Sphere")) {
		figure = createSphere(1, figureConfig["n"].as_int_or_die());
	} else if (type == std::string("Torus")) {
		figure = createTorus(figureConfig["r"].as_double_or_die(), figureConfig["R"].as_double_or_die(), figureConfig["n"].as_int_or_die(), figureConfig["m"].as_int_or_die());
	} else if (type == std::string("3DLSystem")) {
		figure = createFrom3DLSystem(figureConfig["inputfile"].as_string_or_die());
	}

	double scale = figureConfig["scale"].as_double_or_die();
	
	double rotateX = figureConfig["rotateX"].as_double_or_die() * 0.0174532925;
	double rotateY = figureConfig["rotateY"].as_double_or_die() * 0.0174532925;
	double rotateZ = figureConfig["rotateZ"].as_double_or_die() * 0.0174532925;

	std::vector<double> center = figureConfig["center"].as_double_tuple_or_die();
	Vector3D centerVector = Vector3D::vector(center[0], center[1], center[2]);

	VectorHelpers vh;
	Matrix scaleMatrix = vh.scaleMatrix(scale);
	Matrix rotateMatrixX = vh.rotXMatrix(rotateX);
	Matrix rotateMatrixY = vh.rotYMatrix(rotateY);
	Matrix rotateMatrixZ = vh.rotZMatrix(rotateZ);
	Matrix centerMatrix = vh.translateMatrix(centerVector);
	figure.allMatrix = scaleMatrix * rotateMatrixX * rotateMatrixY * rotateMatrixZ * centerMatrix;

	std::vector<double> color = figureConfig["color"].as_double_tuple_or_die();
	figure.color = img::Color(roundToInt(color[0] * 255), roundToInt(color[1] * 255), roundToInt(color[2] * 255));

	return figure;
}