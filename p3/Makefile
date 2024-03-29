APELLIDO1_APELLIDO2_NOMBRE = APELLIDO1_APELLIDO2_NOMBRE
LEVEL = 3324
LEVELS = 500 1500 2500 3500 4500 5500 6500
ABSOLUTE_ERROR_TIME = 0.01
RELATIVE_ERROR_TIME = 0.1

####################### FIN CONFIGURACION ###########################

# Cuidado con la configuración del editor de texto. Los comandos de los objetivos deben de ir precedidos por tabuladores, NO ESPACIOS (error: falta separador). 

FILES_TO_PACK_P1 = ../p1/DefenseStrategy.cpp ../p1/*.tex ../p1/p1.pdf ../autor.tex ../p1/Makefile ../p1/ppm.h
FILES_TO_PACK_P2 = ../p2/DefensesSelection.cpp ../p2/*.tex ../p2/p2.pdf ../p2/Makefile
FILES_TO_PACK_P3 = ../p3/DefenseStrategy.cpp ../p3/*.tex ../p3/p3.pdf ../p3/Makefile ../p3/cronometro.h
CC = g++ # C++ compiler
CFLAGS = -fPIC -O2 -ffloat-store -std=c++11 -fpermissive # -Wall -Wextra -g 
LDFLAGS = -shared # linking flags
RM = rm -f # rm command
TARGET_LIB = libDefenseStrategy3.so # target lib
SIMPATH = ../simulador/
EXE = $(SIMPATH)simulador
VIEWER = ../visor/bin/lin/asedio_player.x86_64
VIEWERFLAGS = # -logfile log.txt
REPLAYFILE = replay.txt

SRCS = DefenseStrategy.cpp # source files
OBJS = $(SRCS:.cpp=.o)
 
.PHONY: all
all: ${TARGET_LIB}
 
$(TARGET_LIB): $(SRCS)
	$(CC) $(CFLAGS) ${LDFLAGS} -o $@ $^

test: $(EXE)
	./$< -l $(LEVEL) -v

savereplay: $(EXE)
	./$< -l $(LEVEL) -v -replay $(REPLAYFILE)

loadreplay: $(REPLAYFILE)
	${VIEWER} -replay $(REPLAYFILE) $(VIEWERFLAGS)

testpaths: $(EXE)
	./$< -l $(LEVEL) -v -p paths

convertimages: 
	for i in *.ppm ; do \
		convert $$i -scale 400x400 $$i.png ; \
	done

convertpaths: 
	for i in paths/*.ppm ; do \
		convert $$i -scale 400x400 $$i.png ; \
	done
	
time: $(EXE)
	./$< -l $(LEVEL) -time-placeDefenses3 -time-absolute-error $(ABSOLUTE_ERROR_TIME) -time-relative-error $(RELATIVE_ERROR_TIME)
	
data: $(EXE)
	rm data.txt ; 
	for level in $(LEVELS); do \
		./$< -l $$level -silent >> data.txt ; \
	done
	
plot:
	gnuplot graphic.plot

doc:
	pdflatex p3.tex
pack:
	tar -czvf $(APELLIDO1_APELLIDO2_NOMBRE)-p3.tar.gz $(FILES_TO_PACK_P1) $(FILES_TO_PACK_P2) $(FILES_TO_PACK_P3)
 
.PHONY: clean
clean:
	-${RM} ${TARGET_LIB} ${OBJS} $(SRCS:.cpp=.d) \
	${RM} *.ppm* *.aux *.log p*.pdf *.tar.gz *.png *.eps data.txt
	${RM} paths/*.ppm*

