################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ODE_01/src/DynamicsMotionData.cpp \
../ODE_01/src/Human.cpp \
../ODE_01/src/MotionData.cpp \
../ODE_01/src/ODE_01.cpp \
../ODE_01/src/OgreCanvas.cpp \
../ODE_01/src/OgreSimulation.cpp \
../ODE_01/src/Simulation.cpp \
../ODE_01/src/Skeleton.cpp 

OBJS += \
./ODE_01/src/DynamicsMotionData.o \
./ODE_01/src/Human.o \
./ODE_01/src/MotionData.o \
./ODE_01/src/ODE_01.o \
./ODE_01/src/OgreCanvas.o \
./ODE_01/src/OgreSimulation.o \
./ODE_01/src/Simulation.o \
./ODE_01/src/Skeleton.o 

CPP_DEPS += \
./ODE_01/src/DynamicsMotionData.d \
./ODE_01/src/Human.d \
./ODE_01/src/MotionData.d \
./ODE_01/src/ODE_01.d \
./ODE_01/src/OgreCanvas.d \
./ODE_01/src/OgreSimulation.d \
./ODE_01/src/Simulation.d \
./ODE_01/src/Skeleton.d 


# Each subdirectory must supply rules for building sources it contributes
ODE_01/src/%.o: ../ODE_01/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -DdDOUBLE -D__GXX_EXPERIMENTAL_CXX0X__ -I/usr/include/OGRE -I/usr/include/eigen3 -I"/home/david/git/ode/ODE_01/src/OgreProcedural/include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++0x -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


