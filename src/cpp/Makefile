# Makefile for C++23 project
CXX = clang++
CXXFLAGS = -std=c++23 -stdlib=libc++ -Wall -Wextra -O2 -g
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
	$(CXX) $(TEST_REQ_IPC_OBJ) $(IPC_OBJS) -o $@

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
