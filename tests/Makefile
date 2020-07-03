CPP=g++
CFLAGS=-I ../cppbktree -std=c++11 -Wall -Wextra -Wshadow -Wsuggest-override -O3 -DNDEBUG

all: testcppbktree check

check: testcppbktree
	./$<

testcppbktree: testcppbktree.o
	$(CPP) $^ -o $@

testcppbktree.o: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CPP) $(CFLAGS) -c $< -o $@
