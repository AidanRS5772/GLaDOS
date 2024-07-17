CXXFLAGS = -I/utils/eigen-3.4.0

all:
	@g++ -Wall homology.cpp -o target/homology
	@./target/homology

del:
	@rm homology
