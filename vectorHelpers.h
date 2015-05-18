#include "vector.hh"

class VectorHelpers {
public:
	VectorHelpers() {};

	// Generate the scale-matrix
	Matrix scaleMatrix(double scaleFactor);

	// Generate the rotation-matrix with the given angle in rads
	Matrix rotXMatrix (double angle);
	Matrix rotYMatrix (double angle);
	Matrix rotZMatrix (double angle);

	// Generate the translate-matrix
	Matrix translateMatrix (Vector3D translation);

	// Carth coordinates to polar
	void toPolar (Vector3D point, double& theta, double& phi, double& r);

	Matrix eyePointMatrix (double& theta, double& phi, double&r);
};