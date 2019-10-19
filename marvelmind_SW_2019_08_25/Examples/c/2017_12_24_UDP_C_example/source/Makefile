all: marvelmind_udp_c

marvelmind_udp_c:
	gcc -g -pthread -c udp_marvelmind.c -o udp_marvelmind.o
	gcc -g -pthread -c udp_example.c -o udp_example.o
	gcc -o marvelmind_udp_c udp_example.o udp_marvelmind.o -pthread

clean:
	rm -rf *.o

