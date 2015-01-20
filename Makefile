aughit:
	g++ -ggdb `pkg-config --cflags opencv` aughit_round_1_and_2.cpp `pkg-config --libs opencv` -lBox2D -fpermissive -o aughit
participant:
	g++ -ggdb `pkg-config --cflags opencv` Aughit_Participant_Copy.cpp `pkg-config --libs opencv` -lBox2D -fpermissive -o participant
round3:
	g++ -ggdb `pkg-config --cflags opencv` aughit_round3.cpp `pkg-config --libs opencv` -lBox2D -fpermissive -o round3  
vidplay:
	g++ -ggdb `pkg-config --cflags opencv` vidplay.cpp `pkg-config --libs opencv` -o vidplay

clean:
	rm aughit round3 vidplay participant
