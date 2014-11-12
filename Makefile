aughit:
	g++ -ggdb `pkg-config --cflags opencv` `basename opencvtest.cpp .cpp` opencvtest.cpp `pkg-config --libs opencv` -lBox2D -o aughit 

clean:
	rm aughit
