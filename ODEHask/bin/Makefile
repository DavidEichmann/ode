
CSRC = /home/david/git/ode/Debug/ODE_01/src
HSSRC = ../src

COBJ = ${CSRC}/*.o ${CSRC}/OgreProcedural/src/*.o
LIBS = -lstdc++ -lOgreMain -lOIS -lboost_system -lboost_chrono -lode

GHCOPTS = -O2

all:
	ghc ${GHCOPTS} -outputdir . -o Main -i${HSSRC} ${HSSRC}/Main.hs ${COBJ} ${LIBS}
