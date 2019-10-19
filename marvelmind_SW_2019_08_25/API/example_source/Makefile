all: mm_api_example

mm_api_example:
	gcc -g -c main.c -o main.o
	gcc -g -c -Wno-attributes marvelmind_api.c -o marvelmind_api.o
	gcc -g -c marvelmind_devices.c -o marvelmind_devices.o
	gcc -g -c marvelmind_example.c -o marvelmind_example.o
	gcc -g -c marvelmind_pos.c -o marvelmind_pos.o
	gcc -g -c marvelmind_utils.c -o marvelmind_utils.o
	gcc -o mm_api_example main.o marvelmind_api.o marvelmind_devices.o marvelmind_example.o marvelmind_pos.o marvelmind_utils.o -ldl

clean:
	rm -rf *.o
	rm ./mm_api_example
