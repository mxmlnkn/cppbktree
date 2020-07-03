from libc.stdlib cimport malloc, free
from libc.stdio cimport SEEK_SET
from libc.stdint cimport uint8_t
from libcpp.map cimport map
from libcpp.vector cimport vector
from libcpp cimport bool


cdef extern from "Python.h":
    char * PyString_AsString(object)
    object PyString_FromStringAndSize(char*, int)

cdef extern from "cppbktree.hpp":
    cppclass BKTree:
        BKTree(const vector[vector[uint8_t]]&) except +
        vector[size_t] find(const vector[uint8_t]&, unsigned short int) except +

cdef class _BKTree:
    cdef BKTree* tree

    def __cinit__(self, list_of_hashes):
        self.tree = new BKTree(<vector[vector[uint8_t]]>list_of_hashes)

    def __dealloc__(self):
        del self.tree

    def find(self, query, distance=0):
        return <list>(self.tree.find(<vector[uint8_t]>query, distance))


# Extra class because cdefs are not visible from outside
class CppBKTree:
    def __init__(self, list_of_hashes):
        self.tree = _BKTree(list_of_hashes)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

__version__ = '0.0.1'
