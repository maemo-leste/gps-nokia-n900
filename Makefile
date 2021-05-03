.POSIX:

NAME = gps-nokia-n900
PREFIX = /usr

GPSNOKIA_CFLAGS = $(CFLAGS)
GPSNOKIA_CPPFLAGS = -D_DEFAULT_SOURCE -D_XOPEN_SOURCE=600 -Wall -pedantic $(CPPFLAGS)
GPSNOKIA_LDFLAGS = $(LDFLAGS)

SRC = $(NAME).c
BIN = $(NAME)
OBJ = $(SRC:.c=.o)

all: $(BIN)

.c.o:
	$(CC) -c $(GPSNOKIA_CFLAGS) $(GPSNOKIA_CPPFLAGS) $<

$(OBJ):

$(BIN): $(OBJ)
	$(CC) -o $@ $(OBJ) $(GPSNOKIA_LDFLAGS)

clean:
	rm -f $(BIN) $(OBJ)

install: all
	mkdir -p $(DESTDIR)$(PREFIX)/sbin
	cp -f $(BIN) $(DESTDIR)$(PREFIX)/sbin
	chmod 755 $(DESTDIR)$(PREFIX)/sbin/$(BIN)

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/sbin/$(BIN)

.PHONY: all clean install uninstall
