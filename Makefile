ME = Makefile
EXEC_FILE=bin/out.a
INCLUDES=-Iinclude/
BUILD_ARGS=
GPP_ARGS=$(INCLUDES) $(BUILD_ARGS) -Wall -Wextra -Wpedantic -std=c++20
GCC_ARGS=$(INCLUDES) $(BUILD_ARGS)
LD_ARGS=-L/usr/lib/x86_64-linux-gnu -lglfw

CC_DEBUG_ARGS=-O0 -g # -fsanitize=address
LD_DEBUG_ARGS=# -fsanitize=address

CPP_SRCS=$(wildcard src/*.cpp)
CPP_OBJS=$(patsubst %.cpp,%_cpp.o,$(CPP_SRCS))
CPP_DEPENDS=$(patsubst %.cpp,%_cpp.d,$(CPP_SRCS))
-include $(CPP_DEPENDS)

%_cpp.o: %.cpp $(ME)
	g++ $(GPP_ARGS) -MMD -MP -c $< -o $@

C_SRCS=$(wildcard src/*.c)
C_OBJS=$(patsubst %.c,%_c.o,$(C_SRCS))

%_c.o: %.c $(ME) $(C_HEADERS)
	g++ $(GCC_ARGS) $< -c -o $@

$(EXEC_FILE): $(CPP_OBJS) $(C_OBJS)
	g++ $(CPP_OBJS) $(C_OBJS) $(LD_ARGS) -MMD -MP -o $(EXEC_FILE)

.PHONY: all debug release test run rund runr runt clean

all: debug

debug: GPP_ARGS += $(CC_DEBUG_ARGS)
debug: GCC_ARGS += $(CC_DEBUG_ARGS)
debug: LD_ARGS += $(LD_DEBUG_ARGS)
debug: BUILD_ARGS += -DPLATFORM_LINUX
debug: $(EXEC_FILE)

release: GPP_ARGS += -O3
release: GCC_ARGS += -O3
release: BUILD_ARGS += -DPLATFORM_LINUX
release: $(EXEC_FILE)

test: GPP_ARGS += $(CC_DEBUG_ARGS)
test: GCC_ARGS += $(CC_DEBUG_ARGS)
test: LD_ARGS += $(LD_DEBUG_ARGS)
test: BUILD_ARGS += -DTEST_LINUX
test: $(EXEC_FILE)

run: 
	./$(EXEC_FILE)

rund: debug
rund: 
	./$(EXEC_FILE)

runr: release
runr: 
	./$(EXEC_FILE)

clean:
	rm $(CPP_OBJS) $(C_OBJS) $(CPP_DEPENDS)
	rm -rf bin/*
	touch bin/.gitkeep