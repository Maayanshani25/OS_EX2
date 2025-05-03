# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -std=c++11 -O3

# Source and object files
SRCS = uthreads.cpp
OBJS = $(SRCS:.cpp=.o)

# Header files
HEADERS = constants.h

# Target static library
LIBRARY = libuthreads.a

# Default target: build static library
all: $(LIBRARY)

$(LIBRARY): $(OBJS)
	ar rcs $@ $^

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target
clean:
	rm -f $(OBJS) $(LIBRARY)

# Tar target for submission
tar:
	tar -cvf ex2.tar README Makefile uthreads.cpp constants.h
