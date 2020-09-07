# Started with https://spin.atomicobject.com/2016/08/26/makefile-c-projects/
# Additional inspiration from https://codereview.stackexchange.com/questions/157780/generic-makefile-handling-unit-testing-and-folder-structure

TARGET_EXEC ?= robot-main
TEST_EXEC ?= test-main

BUILD_DIR ?= build
SRC_DIRS ?= src
TEST_DIRS ?=  test

SRCS := $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c -or -name *.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := ./lib ./src /usr/local/include
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wwrite-strings -Wno-parentheses -pthread -g
LDFLAGS += -ldl -lconfig++ -lstdc++fs
LIBS += /usr/local/lib/libserial.a ./lib/marvelmind/marvelmind.a

TEST_SRCS := $(shell find $(TEST_DIRS) -name *.cpp -or -name *.c -or -name *.s)
TEST_OBJS := $(TEST_SRCS:%=$(BUILD_DIR)/%.o) $(filter-out build/src/main.cpp.o, $(OBJS))

vpath %.cpp $(SRC_DIRS)

.PHONY: all
all: $(BUILD_DIR)/$(TARGET_EXEC)

.PHONY: test
test: $(BUILD_DIR)/$(TEST_EXEC)

# Target file
$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@ $(LIBS) $(LDFLAGS)

# Test file
$(BUILD_DIR)/$(TEST_EXEC): $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) $(TEST_OBJS) -o $@ $(LIBS) $(LDFLAGS)

# Source files
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)


-include $(DEPS)
MKDIR_P ?= mkdir -p

.DELETE_ON_ERROR:
