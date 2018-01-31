all:	gps-nokia-n900

gps-nokia-n900:	gps-nokia-n900.c
	gcc -std=c99 gps-nokia-n900.c -o gps-nokia-n900
