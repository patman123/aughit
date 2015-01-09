aughit:
	g++ -ggdb `pkg-config --cflags opencv` opencvtest.cpp `pkg-config --libs opencv` -lBox2D -fpermissive -o aughit  
vidplay:
	g++ -ggdb `pkg-config --cflags opencv` vidplay.cpp `pkg-config --libs opencv` -o vidplay

clean:
	rm aughit vidplay
