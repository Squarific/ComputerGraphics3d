ini_configuration.o: ini_configuration.cc ini_configuration.hh
	$(CC) $(CXXFLAGS) -c $< -o $@
