3dfigures.o: 3dfigures.cc vector.hh 3dfigures.h EasyImage.h \
 ini_configuration.hh vectorHelpers.h
	$(CC) $(CXXFLAGS) -c $< -o $@
