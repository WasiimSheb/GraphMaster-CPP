# Wasimshebalny@gmail.com

# Compiler
CC = clang++

# Compiler flags
CFLAGS = -std=c++11 -Wall -Wextra

# Executable names
DEMO_EXEC = demo
TEST_EXEC = test

# Source files
DEMO_SRC = Demo.cpp Graph.cpp Algorithms.cpp
TEST_SRC = Test.cpp TestCounter.cpp Graph.cpp Algorithms.cpp

# Object files
DEMO_OBJ = $(DEMO_SRC:.cpp=.o)
TEST_OBJ = $(TEST_SRC:.cpp=.o)

# Targets
all: $(DEMO_EXEC) $(TEST_EXEC)

$(DEMO_EXEC): $(DEMO_OBJ)
	$(CC) $(CFLAGS) $(DEMO_OBJ) -o $(DEMO_EXEC)

$(TEST_EXEC): $(TEST_OBJ)
	$(CC) $(CFLAGS) $(TEST_OBJ) -o $(TEST_EXEC)

# Object file dependencies
demo.o: Demo.cpp Graph.hpp Algorithms.hpp
	$(CC) $(CFLAGS) -c Demo.cpp -o demo.o

TestCounter.o: TestCounter.cpp Graph.hpp Algorithms.hpp
	$(CC) $(CFLAGS) -c TestCounter.cpp -o TestCounter.o

Test.o: Test.cpp Graph.hpp Algorithms.hpp
	$(CC) $(CFLAGS) -c Test.cpp -o Test.o

Graph.o: Graph.cpp Graph.hpp
	$(CC) $(CFLAGS) -c Graph.cpp -o Graph.o

Algorithms.o: Algorithms.cpp Algorithms.hpp Graph.hpp
	$(CC) $(CFLAGS) -c Algorithms.cpp -o Algorithms.o

# Clean
clean:
	rm -f $(DEMO_OBJ) $(TEST_OBJ) $(DEMO_EXEC) $(TEST_EXEC)
