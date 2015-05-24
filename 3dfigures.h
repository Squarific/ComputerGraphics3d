#ifndef THREEDFIGURES_H
#define THREEDFIGURES_H

#include "vector.hh"
#include "EasyImage.h"
#include "ini_configuration.hh"
#include <vector>
#include <list>

class Face {
  public:
	//De indexen refereren naar
	//punten in de ‘points’ vector
	//van de Figure-klasse	
	std::vector<int> point_indexes;
	Face () {};
	Face (std::vector<int> point_indexes) {
		this->point_indexes = point_indexes;
	};
};

class Figure {
public:
	std::vector<Vector3D> points;
	std::vector<Face> faces;
	img::Color color;
	Matrix allMatrix;

	Figure () {};
	void applyTransformation (Matrix const& t);
};

typedef std::list<Figure> Figures3D;
Figure figure3dFromConfig (ini::Section const& figureConfig);

#endif /* THREEDFIGURES_H */