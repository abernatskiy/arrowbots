CC=g++
CFLAGS=-std=c++11 -O2 -msse2 -ffast-math -m64 -fno-rtti -fno-exceptions -fno-stack-protector ${DISTFLAGS} ${MORECFLAGS} -g -ggdb -Wall
LDFLAGS=-m64 -g -ggdb -Wall
OBJECTS=main.o arrowBot.o

all: arrowBotEvaluator

arrowBotEvaluator : $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

##########################################
# Generic rules
##########################################

%.o: %.cpp %.h
	$(CC) -o $@ -c $< $(CFLAGS)

%.o: %.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -f *.o *~ arrowBotEvaluator
