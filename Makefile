all:
	@g++ -std=c++20 -Wall homology.cpp -o target/homology
	@./target/homology

del:
	@rm homology
