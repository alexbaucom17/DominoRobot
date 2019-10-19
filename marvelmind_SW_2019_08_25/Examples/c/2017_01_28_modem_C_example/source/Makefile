all: marvelmind_modem_c

marvelmind_modem_c:
	gcc -g -pthread -c marvelmind_modem.c -o marvelmind_modem.o
	gcc -g -pthread -c modem_example.c -o modem_example.o
	gcc -o marvelmind_modem_c modem_example.o marvelmind_modem.o -pthread

clean:
	rm -rf *.o

