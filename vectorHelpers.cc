#include "vector.hh"
#include "vectorHelpers.h"
#include <cmath>

Matrix VectorHelpers::scaleMatrix(double scaleFactor) {
	Matrix scaleMatrix;

	scaleMatrix(1, 1) = scaleFactor;
	scaleMatrix(2, 2) = scaleFactor;
	scaleMatrix(3, 3) = scaleFactor;

	return scaleMatrix;
}

Matrix VectorHelpers::rotXMatrix (double angle) {
	Matrix rotXMatrix;

	rotXMatrix(2, 2) =  std::cos(angle);
	rotXMatrix(3, 2) = -std::sin(angle);
	rotXMatrix(2, 3) =  std::sin(angle);
	rotXMatrix(3, 3) =  std::cos(angle);

	return rotXMatrix;
}

Matrix VectorHelpers::rotYMatrix (double angle) {
	Matrix rotYMatrix;

	rotYMatrix(1, 1) =  std::cos(angle);
	rotYMatrix(3, 1) =  std::sin(angle);
	rotYMatrix(1, 3) = -std::sin(angle);
	rotYMatrix(3, 3) =  std::cos(angle);

	return rotYMatrix;
}

Matrix VectorHelpers::rotZMatrix (double angle) {
	Matrix rotZMatrix;

	rotZMatrix(1, 1) =  std::cos(angle);
	rotZMatrix(1, 2) =  std::sin(angle);
	rotZMatrix(2, 1) = -std::sin(angle);
	rotZMatrix(2, 2) =  std::cos(angle);

	return rotZMatrix;
}


Matrix VectorHelpers::translateMatrix (Vector3D translation) {
	Matrix translateMatrix;

	translateMatrix(4, 1) = translation.x;
	translateMatrix(4, 2) = translation.y;
	translateMatrix(4, 3) = translation.z;

	return translateMatrix;
}

void VectorHelpers::toPolar (Vector3D point, double& theta, double& phi, double& r) {
	r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
	theta = std::atan2(point.y, point.x);
	phi = std::acos(point.z / r);
}

Matrix VectorHelpers::eyePointMatrix (double& theta, double& phi, double&r) {
	Matrix eyePointMatrix;

	eyePointMatrix(1, 1) = -std::sin(theta);
	eyePointMatrix(2, 1) =  std::cos(theta);

	eyePointMatrix(1, 2) = -std::cos(theta) * std::cos(phi);
	eyePointMatrix(2, 2) = -std::sin(theta) * std::cos(phi);
	eyePointMatrix(3, 2) =  std::sin(phi);

	eyePointMatrix(1, 3) = std::cos(theta) * std::sin(phi);
	eyePointMatrix(2, 3) = std::sin(theta) * std::sin(phi);
	eyePointMatrix(3, 3) = std::cos(phi);
	eyePointMatrix(4, 3) = -r;

	return eyePointMatrix;
}