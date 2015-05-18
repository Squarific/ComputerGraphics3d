LineDrawing.o: LineDrawing.cc EasyImage.h lparser.h 2dhelpers.h \
 LineDrawing.h 3dfigures.h vector.hh ini_configuration.hh
	$(CC) $(CXXFLAGS) -c $< -o $@
