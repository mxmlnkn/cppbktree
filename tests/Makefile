CXX?=g++
CFLAGS+=-I ../cppbktree -std=c++17 -Wall -Wextra -Wshadow -Wsuggest-override -O3 -msse4.2
LFLAGS+=

all: testcppbktree check

check: testcppbktree
	./$<

testcppbktree: testcppbktree.o
	$(CXX) $(LFLAGS) $^ -o $@

testcppbktree.o: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CXX) $(CFLAGS) -c $< -o $@
