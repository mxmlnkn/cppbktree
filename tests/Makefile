CXX?=g++
CFLAGS+=-I ../cppbktree -I../external/xsimd/include -std=c++17 -Wall -Wextra -Wshadow -Wsuggest-override -O3 -msse4.2
LFLAGS+=

all: testcppbktree check testcppbktree.s

check: testcppbktree
	./$<

testcppbktree: testcppbktree.o
	$(CXX) $(LFLAGS) $^ -o $@

testcppbktree.o: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CXX) $(CFLAGS) -c $< -o $@

testcppbktree.s: testcppbktree.cpp ../cppbktree/cppbktree.hpp
	$(CXX) $(CFLAGS) -fverbose-asm -masm=intel -S -c $< -o $@
