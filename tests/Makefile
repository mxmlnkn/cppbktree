CPP=g++
CFLAGS=-I ../cppbktree -std=c++17 -Wall -Wextra -Wshadow -Wsuggest-override -O3 -DNDEBUG -fopenmp -march=native

all: testcppbktree check

check: testcppbktree
	./$<

testcppbktree: testcppbktree.o
	$(CPP) -fopenmp $^ -o $@

testcppbktree.o: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CPP) $(CFLAGS) -c $< -o $@
