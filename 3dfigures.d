3dfigures.o: 3dfigures.cc vector.hh 3dfigures.h EasyImage.h \
 ini_configuration.hh lparser.h vectorHelpers.h
	$(CC) $(CXXFLAGS) -c $< -o $@
