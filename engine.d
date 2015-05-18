engine.o: engine.cc EasyImage.h ini_configuration.hh render.h
	$(CC) $(CXXFLAGS) -c $< -o $@
