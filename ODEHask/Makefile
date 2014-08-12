
BIN = bin
OBJ = obj
CPP_SRC = ../ODE_01/src
HS_SRC = src

INCLUDES = -I${CPP_SRC}/OgreProcedural/include -I/usr/include/eigen3 -I/usr/include/OGRE

CC_FLAGS = -DdDOUBLE=1 -std=c++0x -O0 -g3 -Wall -fmessage-length=0 -fPIC 
CPP_FILES = $(wildcard ${CPP_SRC}/*.cpp)
OBJ_FILES = $(addprefix ${OBJ}/,$(notdir $(CPP_FILES:.cpp=.o))) $(wildcard ../Debug/ODE_01/src/OgreProcedural/src/*.o)

LIBS = -lstdc++ -lOgreMain -lOIS -lboost_system -lboost_chrono -lode -lpthread
GHCOPTS = -O2 -XBangPatterns -XMultiParamTypeClasses -prof -auto-all -caf-all

${OBJ}/%.o: ${CPP_SRC}/%.cpp
	g++ ${INCLUDES} $(CC_FLAGS) -c -o $@ $<


all: ${OBJ_FILES}
	ghc ${GHCOPTS} -outputdir bin -o ${BIN}/Main -i${HS_SRC} ${HS_SRC}/Main.hs ${OBJ_FILES} ${LIBS}

	
clean:
	rm -r ${BIN}/*
	rm ${OBJ}/*