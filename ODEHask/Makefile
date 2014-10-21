
BIN = bin
OBJ = obj
CPP_SRC = ../ODE_01/src
HS_SRC = src

INCLUDES = -I${CPP_SRC}/OgreProcedural/include -I/usr/include/eigen3 -I/usr/include/OGRE

CC_FLAGS = -DdDOUBLE=1 -std=c++0x -O2 -g3 -Wall -fmessage-length=0 -fPIC
CPP_FILES = $(wildcard ${CPP_SRC}/*.cpp)
OBJ_FILES = $(addprefix ${OBJ}/,$(notdir $(CPP_FILES:.cpp=.o))) $(wildcard ../Debug/ODE_01/src/OgreProcedural/src/*.o)
OBJ_FILES_NOOGRE = ${OBJ}/interface.o ${OBJ}/interface.o

LIBS = -lstdc++ -lOgreMain -lOIS -lboost_system -lboost_chrono -lode -lpthread -lglpk
LIBS_NOOGRE = -lstdc++  -lboost_system -lboost_chrono -lode -lpthread -lglpk
GHCOPTS_STATIC = -static -optl-pthread -optl-static
GHCOPTS = -O2 -XBangPatterns -XMultiParamTypeClasses -prof -auto-all -caf-all

${OBJ}/%.o: ${CPP_SRC}/%.cpp
	g++ ${INCLUDES} $(CC_FLAGS) -c -o $@ $<


NOOGRE_${OBJ}/%.o: ${CPP_SRC}/%.cpp
	g++ ${INCLUDES} $(CC_FLAGS) -D NOOGRE -c -o $@ $<


all: ${OBJ_FILES}
	ghc ${GHCOPTS} -outputdir bin -o ${BIN}/Main -i${HS_SRC} ${HS_SRC}/Main.hs ${OBJ_FILES} ${LIBS}


static-noogre:
	g++ ${INCLUDES} $(CC_FLAGS) -D NOOGRE -c -o ${OBJ}/Interface.o ${CPP_SRC}/Interface.cpp
	ghc ${GHCOPTS} ${GHCOPTS_STATIC} -outputdir bin -o ${BIN}/Main -i${HS_SRC} ${HS_SRC}/Main.hs ${OBJ}/* ${LIBS_NOOGRE}

clean:
	rm -r ${BIN}/*
	rm ${OBJ}/*
