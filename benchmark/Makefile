CXX?=g++
CFLAGS+=-I ../cppbktree -std=c++17 -Wall -Wextra -Wshadow -Wsuggest-override -O3 -msse4.2 -mavx2 -fopenmp
LFLAGS+=-fopenmp

all: benchmark-cppbktree benchmark benchmark-cppbktree.s

benchmark: benchmark-cppbktree
	./$<

benchmark-cppbktree: benchmark-cppbktree.o
	$(CXX) $(LFLAGS) $^ -o $@

benchmark-cppbktree.o: benchmark-cppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CXX) $(CFLAGS) -c $< -o $@

benchmark-cppbktree.s: benchmark-cppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CXX) $(CFLAGS) -fverbose-asm -masm=intel -S -c $< -o $@
