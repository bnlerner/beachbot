# Makefile for C++23 project

# Auto-detect available compiler (prefer newest versioned compilers)
CLANG_CAND := $(firstword $(foreach v,20 19 18 17 16 15 14 13,$(shell which clang++-$(v) 2>/dev/null)))
GCC_CAND   := $(firstword $(foreach v,14 13 12 11 10,$(shell which g++-$(v) 2>/dev/null)))

ifeq ($(CLANG_CAND),)
  ifeq ($(shell which clang++ 2>/dev/null),)
    ifeq ($(GCC_CAND),)
      ifeq ($(shell which g++ 2>/dev/null),)
        $(error No C++ compiler found)
      else
        CXX = g++
      endif
    else
      CXX = $(GCC_CAND)
    endif
  else
    CXX = clang++
  endif
else
  CXX = $(CLANG_CAND)
endif

# Base flags (add standard flag detection below)
BASE_CXXFLAGS = -Wall -Wextra -O2 -g

# Extra flags for specific platforms/compilers
LIBCXX_FLAG =
ifeq ($(findstring clang++,$(notdir $(CXX))),clang++)
  ifeq ($(shell uname -s),Darwin)
    LIBCXX_FLAG = -stdlib=libc++
  endif
endif

# Detect the correct C++23 standard flag supported by the chosen compiler
STD23_OK := $(shell $(CXX) -std=c++23 -E -x c++ /dev/null -o /dev/null >/dev/null 2>&1 && echo yes || echo no)
STD2B_OK := $(shell $(CXX) -std=c++2b -E -x c++ /dev/null -o /dev/null >/dev/null 2>&1 && echo yes || echo no)

ifeq ($(STD23_OK),yes)
  STD_FLAG = -std=c++23
else ifeq ($(STD2B_OK),yes)
  # Older compilers used the provisional -std=c++2b flag
  STD_FLAG = -std=c++2b
else
  $(error The compiler '$(CXX)' does not support C++23. Install a newer compiler (g++ >= 13 or clang++ >= 16), or set CXX to one that supports it.)
endif

# Final CXXFLAGS
# Platform thread flag (-pthread needed on Linux for both compile and link)
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  PTHREAD_FLAG :=
else
  PTHREAD_FLAG := -pthread
endif

# Final CXXFLAGS
CXXFLAGS = $(STD_FLAG) $(LIBCXX_FLAG) $(BASE_CXXFLAGS) $(PTHREAD_FLAG)
INCLUDES = -I.
SRCDIR = .
BUILDDIR = ../../build
OBJDIR = $(BUILDDIR)/obj/cpp
BINDIR = $(BUILDDIR)/bin

# Sources
MAIN_SRC =
IPC_SRCS = $(SRCDIR)/ipc/core.cpp $(SRCDIR)/ipc/messages.cpp $(SRCDIR)/ipc/pubsub.cpp $(SRCDIR)/ipc/request.cpp $(SRCDIR)/ipc/registry.cpp
TEST_REQ_IPC_SRC = $(SRCDIR)/tests/request_ipc_communication.cpp

# Objects
MAIN_OBJ = $(OBJDIR)/main.o
IPC_OBJS = $(IPC_SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
TEST_REQ_IPC_OBJ = $(OBJDIR)/tests_request_ipc_communication.o

# Targets
TARGET = $(BINDIR)/beachbot_cpp
TEST_REQ_IPC_BIN = $(BINDIR)/request_ipc_communication

.PHONY: all clean test

all: $(TEST_REQ_IPC_BIN)

$(TARGET): $(IPC_OBJS) | $(BINDIR)
	@true

$(TEST_REQ_IPC_BIN): $(TEST_REQ_IPC_OBJ) $(IPC_OBJS) | $(BINDIR)
	$(CXX) $(TEST_REQ_IPC_OBJ) $(IPC_OBJS) $(PTHREAD_FLAG) -o $@

$(OBJDIR)/main.o: $(MAIN_SRC) | $(OBJDIR)
	@true

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJDIR)/tests_request_ipc_communication.o: $(TEST_REQ_IPC_SRC) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/bin:
	mkdir -p $(BUILDDIR)/bin

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR) $(TARGET) $(TEST_REQ_IPC_BIN)

debug: CXXFLAGS += -DDEBUG -g
debug: $(TARGET)

release: CXXFLAGS += -DNDEBUG -O3
release: $(TARGET)

# Generate compile_commands.json for clangd
compile_commands:
	bear -- make clean all

