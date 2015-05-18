vector.o: vector.cc vector.hh
	$(CC) $(CXXFLAGS) -c $< -o $@
