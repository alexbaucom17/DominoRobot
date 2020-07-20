# Started with https://spin.atomicobject.com/2016/08/26/makefile-c-projects/
# Additional inspiration from https://codereview.stackexchange.com/questions/157780/generic-makefile-handling-unit-testing-and-folder-structure

TARGET_EXEC ?= robot

BUILD_DIR ?= ./build
SRC_DIRS ?= ./src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c -or -name *.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := ./lib
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wwrite-strings -Wno-parentheses

.PHONY: all
all: $(BUILD_DIR)/$(TARGET_EXEC)

# Target file
$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@ $(LDFLAGS)

# Source files
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)


-include $(DEPS)
MKDIR_P ?= mkdir -p
