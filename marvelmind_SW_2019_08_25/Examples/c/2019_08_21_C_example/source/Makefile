all: marvelmind_c

marvelmind_c:
	gcc -g -pthread -c marvelmind.c -o marvelmind.o
	gcc -g -pthread -c example.c -o example.o
	gcc -o marvelmind_c example.o marvelmind.o -pthread

clean:
	rm -rf *.o

