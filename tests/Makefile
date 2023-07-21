CPP=g++
CFLAGS=-I ../cppbktree -I../external/xsimd/include -std=c++17 -Wall -Wextra -Wshadow -Wsuggest-override -O3 -fopenmp -msse4.2

all: testcppbktree check testcppbktree.s

check: testcppbktree
	./$<

testcppbktree: testcppbktree.o
	$(CPP) -fopenmp $^ -o $@

testcppbktree.o: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CPP) $(CFLAGS) -c $< -o $@

testcppbktree.s: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CPP) $(CFLAGS) -fverbose-asm -masm=intel -S -c $< -o $@
