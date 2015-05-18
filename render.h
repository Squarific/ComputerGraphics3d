#include "EasyImage.h"
#include "ini_configuration.hh"

class Render {
public:
	Render() {};
	img::EasyImage renderTwo2dLSystem (const ini::Configuration &configuration);
	img::EasyImage renderWireFrame (const ini::Configuration &configuration);
};