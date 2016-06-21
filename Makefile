all: main

main: main.o rayCast.o material.o
	g++ -o main main.o rayCast.o material.o -O3

main.o: main.cpp
	g++ -c main.cpp -O3

rayCast.o: rayCast.cpp
	g++ -c rayCast.cpp -O3

material.o: material.cpp
	g++ -c material.cpp -O3

run: all
	./main

clean:
	rm -f *.o
	rm -f main
