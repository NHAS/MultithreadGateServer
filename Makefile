CC=g++

build: gatecontroller.cpp
	$(CC) gatecontroller.cpp -o gatecontroller -std=c++14 -lpthread

clean: gatecontroller gatecontroller.o
	rm -f gatecontroller gatecontroller.o
