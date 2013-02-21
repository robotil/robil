################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BT.cpp \
../src/BTExecuter.cpp \
../src/Dec.cpp \
../src/ExeEnergy.cpp \
../src/ExeStack.cpp \
../src/ExecuterStatus.cpp \
../src/Info.cpp \
../src/Logger.cpp \
../src/Lookup.cpp \
../src/MapReader.cpp \
../src/Node.cpp \
../src/Par.cpp \
../src/Result.cpp \
../src/Sel.cpp \
../src/Seq.cpp \
../src/Swi.cpp \
../src/Task.cpp \
../src/TaskProxyTableXML.cpp \
../src/TestTaskProxyTable.cpp \
../src/UnknownNode.cpp \
../src/main.cpp 

OBJS += \
./src/BT.o \
./src/BTExecuter.o \
./src/Dec.o \
./src/ExeEnergy.o \
./src/ExeStack.o \
./src/ExecuterStatus.o \
./src/Info.o \
./src/Logger.o \
./src/Lookup.o \
./src/MapReader.o \
./src/Node.o \
./src/Par.o \
./src/Result.o \
./src/Sel.o \
./src/Seq.o \
./src/Swi.o \
./src/Task.o \
./src/TaskProxyTableXML.o \
./src/TestTaskProxyTable.o \
./src/UnknownNode.o \
./src/main.o 

CPP_DEPS += \
./src/BT.d \
./src/BTExecuter.d \
./src/Dec.d \
./src/ExeEnergy.d \
./src/ExeStack.d \
./src/ExecuterStatus.d \
./src/Info.d \
./src/Logger.d \
./src/Lookup.d \
./src/MapReader.d \
./src/Node.d \
./src/Par.d \
./src/Result.d \
./src/Sel.d \
./src/Seq.d \
./src/Swi.d \
./src/Task.d \
./src/TaskProxyTableXML.d \
./src/TestTaskProxyTable.d \
./src/UnknownNode.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -DEXE_COMPILATION -O0 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


