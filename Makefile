all: clean
	gcc src/main.c -o src/main.o -c
	gcc src/msp.c -o src/msp.o -c
	gcc src/serial.c -o src/serial.o -c
	gcc -o obj src/main.o src/msp.o src/serial.o
	rm src/*.o
clean:
	rm -rf obj