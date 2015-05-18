lparser.o: lparser.cc lparser.h
	$(CC) $(CXXFLAGS) -c $< -o $@
