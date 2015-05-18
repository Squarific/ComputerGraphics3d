#include "EasyImage.h"
#include "lparser.h"
#include "LineDrawing.h"
#include "render.h"
#include "vector.hh"
#include "3dfigures.h"
#include "vectorHelpers.h"
#include "ini_configuration.hh"
#include <fstream>
#include <cmath>
#include <string>

inline int roundToInt(double d) {
    return d < 0 ? 
        std::ceil(d-0.5):
        std::floor(d+0.5);
}

img::EasyImage Render::renderTwo2dLSystem (const ini::Configuration &configuration) {
    LineDrawing lSystemLines = LineDrawing();
    LParser::LSystem2D l_system;

    std::ifstream input_stream(configuration["2DLSystem"]["inputfile"].as_string_or_die().c_str());
    input_stream >> l_system;
    input_stream.close();

    int size = configuration["General"]["size"].as_int_or_die();
    std::vector<double> back = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
    std::vector<double> color = configuration["2DLSystem"]["color"].as_double_tuple_or_die();
    img::EasyImage image = img::EasyImage(size, size, img::Color(roundToInt(back[0] * 255), roundToInt(back[1] * 255), roundToInt(back[2] * 255)));

    lSystemLines.addLinesFromLSystem2D(l_system, img::Color(roundToInt(color[0] * 255), roundToInt(color[1] * 255), roundToInt(color[2] * 255)));
    lSystemLines.draw2dLines(image, size);

    return image;
}

img::EasyImage Render::renderWireFrame (const ini::Configuration &configuration) {
	LineDrawing wireFrameLines = LineDrawing();

    int size = configuration["General"]["size"].as_int_or_die();
    int nrFigures = configuration["General"]["nrFigures"].as_int_or_die();

    std::vector<double> back = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
    std::vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
    Vector3D eyePoint = Vector3D::point(eye[0], eye[1], eye[2]);

    Figures3D figures;

	while (nrFigures > 0) {
        nrFigures--;
        figures.push_back(figure3dFromConfig(configuration[(std::string("Figure") + std::to_string(nrFigures)).c_str()]));
    }

    VectorHelpers vh;

    double theta;
    double phi;
    double r;
    vh.toPolar(eyePoint, theta, phi, r);
    
    Matrix eyePointMatrix = vh.eyePointMatrix(theta, phi, r);
    std::cout << eyePointMatrix;

    for (auto &figure : figures) {
        figure.applyTransformation(eyePointMatrix);
    }

    wireFrameLines.addLinesFromProjection(figures);

    img::EasyImage image = img::EasyImage(size, size, img::Color(roundToInt(back[0] * 255), roundToInt(back[1] * 255), roundToInt(back[2] * 255)));
    wireFrameLines.draw2dLines(image, size);

    return image;
}