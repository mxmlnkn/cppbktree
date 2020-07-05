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
    size_t hammingDistance(const vector[uint8_t]&, const vector[uint8_t]& ) except +

    cppclass CppBKTree[T_ValueType, T_DistanceType]:
        struct TreeStatistics:
            TreeStatistics()

            size_t nodeCount
            size_t leafCount
            size_t valueCount
            double averageChildCountPerNode
            size_t maxDepth
            size_t minChildrenPerNode
            size_t maxChildrenPerNode
            size_t duplicateCount
            size_t valueBitCount

        # ToDo: Use the constructor which takes a functor:
        #   https://docs.python.org/3/library/ctypes.html#callback-functions
        #   https://stackoverflow.com/questions/39044063/pass-a-closure-from-cython-to-c
        #   https://stackoverflow.com/questions/34878942/using-function-pointers-to-methods-of-classes-without-the-gil/34900829#34900829
        CppBKTree(const vector[T_ValueType]&) except +
        vector[size_t] find(const T_ValueType&, unsigned short int) except +
        size_t size() except +
        TreeStatistics statistics() except +


cdef class _BKTree:
    cdef CppBKTree[vector[uint8_t], size_t]* tree

    def __cinit__(self, list_of_hashes):
        self.tree = new CppBKTree[vector[uint8_t], size_t](<vector[vector[uint8_t]]>list_of_hashes)

    def __dealloc__(self):
        del self.tree

    def find(self, query, distance=0):
        return <list>(self.tree.find(<vector[uint8_t]>query, distance))

    def size(self):
        return self.tree.size()

    def statistics(self):
        # Automatic POD to dict conversion did not work for me. Maybe because the contained types?
        cdef CppBKTree[vector[uint8_t], size_t].TreeStatistics result = self.tree.statistics();
        stats = {
            'nodeCount'                : result.nodeCount               ,
            'leafCount'                : result.leafCount               ,
            'valueCount'               : result.valueCount              ,
            'averageChildCountPerNode' : result.averageChildCountPerNode,
            'maxDepth'                 : result.maxDepth                ,
            'minChildrenPerNode'       : result.minChildrenPerNode      ,
            'maxChildrenPerNode'       : result.maxChildrenPerNode      ,
            'duplicateCount'           : result.duplicateCount          ,
            'valueBitCount'            : result.valueBitCount           ,
        }
        return stats


# Extra class because cdefs are not visible from outside
class BKTree:
    def __init__(self, list_of_hashes):
        self.tree = _BKTree(list_of_hashes)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

    def size(self):
        return self.tree.size()

    def statistics(self):
        return self.tree.statistics()

__version__ = '0.0.1'
