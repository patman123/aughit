aughit:
	g++ -ggdb `pkg-config --cflags opencv` opencvtest.cpp `pkg-config --libs opencv` -lBox2D -fpermissive -o aughit
round3:
	g++ -ggdb `pkg-config --cflags opencv` round3.cpp `pkg-config --libs opencv` -lBox2D -fpermissive -o round3  
vidplay:
	g++ -ggdb `pkg-config --cflags opencv` vidplay.cpp `pkg-config --libs opencv` -o vidplay

clean:
	rm aughit round3 vidplay
