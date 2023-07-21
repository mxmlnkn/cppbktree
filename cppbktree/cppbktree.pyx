# distutils: language=c++

from libc.stdlib cimport malloc, free
from libc.stdio cimport SEEK_SET
from libc.stdint cimport uint8_t, uint64_t
from libcpp.map cimport map
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool


cdef extern from "Python.h":
    char * PyString_AsString(object)
    object PyString_FromStringAndSize(char*, int)


cdef extern from "LinearLookup.hpp":
    cppclass CppLinearLookup[T_ValueType]:
        CppLinearLookup(vector[T_ValueType]) except +
        vector[size_t] find(T_ValueType, unsigned short int) except +
        size_t size() except +


cdef class _LinearLookup64:
    cdef CppLinearLookup[uint64_t]* data

    def __cinit__(self, list_of_hashes):
        self.data = new CppLinearLookup[uint64_t](list_of_hashes)

    def __dealloc__(self):
        del self.data

    def find(self, query, distance=0):
        return <list>(self.data.find(query, distance))

    def size(self):
        return self.data.size()

# Extra class because cdefs are not visible from outside
class LinearLookup64:
    def __init__(self, list_of_hashes):
        self.tree = _LinearLookup64(list_of_hashes)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

    def size(self):
        return self.tree.size()


cdef extern from "cppbktree.hpp":
    size_t hammingDistance(const vector[uint8_t]&, const vector[uint8_t]& ) except +
    size_t hammingDistance64( const uint64_t, const uint64_t ) except +;

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
            size_t minPayloadsPerNode
            size_t maxPayloadsPerNode
            size_t duplicateCount
            size_t valueBitCount

        # ToDo: Use the constructor which takes a functor:
        #   https://docs.python.org/3/library/ctypes.html#callback-functions
        #   https://stackoverflow.com/questions/39044063/pass-a-closure-from-cython-to-c
        #   https://stackoverflow.com/questions/34878942/using-function-pointers-to-methods-of-classes-without-the-gil/34900829#34900829
        CppBKTree(const vector[T_ValueType]&) except +
        CppBKTree(const string&) except +
        vector[size_t] find(const T_ValueType&, unsigned short int) except +
        size_t size() except +
        TreeStatistics statistics() except +
        void serialize(const string&) except +


cdef class _BKTree:
    cdef CppBKTree[vector[uint8_t], size_t]* tree

    def __cinit__(self, list_of_hashes_or_file_name):
        if isinstance(list_of_hashes_or_file_name, str):
            self.tree = new CppBKTree[vector[uint8_t], size_t](<string>list_of_hashes_or_file_name.encode())
        elif isinstance(list_of_hashes_or_file_name, bytes):
            self.tree = new CppBKTree[vector[uint8_t], size_t](<string>list_of_hashes_or_file_name)
        else:
            self.tree = new CppBKTree[vector[uint8_t], size_t](<vector[vector[uint8_t]]>list_of_hashes_or_file_name)

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
            'minPayloadsPerNode'       : result.minPayloadsPerNode      ,
            'maxPayloadsPerNode'       : result.maxPayloadsPerNode      ,
            'duplicateCount'           : result.duplicateCount          ,
            'valueBitCount'            : result.valueBitCount           ,
        }
        return stats

    def serialize(self, file_name):
        if isinstance(file_name, str):
            return self.tree.serialize(<string>file_name.encode())
        else:
            return self.tree.serialize(<string>file_name)


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

    def serialize(self, file_name):
        return self.tree.serialize(file_name)


__version__ = '0.0.1'
