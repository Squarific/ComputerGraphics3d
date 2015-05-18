vectorHelpers.o: vectorHelpers.cc vector.hh vectorHelpers.h
	$(CC) $(CXXFLAGS) -c $< -o $@
