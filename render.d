render.o: render.cc EasyImage.h lparser.h LineDrawing.h 3dfigures.h \
 vector.hh ini_configuration.hh 2dhelpers.h render.h vectorHelpers.h
	$(CC) $(CXXFLAGS) -c $< -o $@
