# distutils: language=c++

from libc.stdlib cimport malloc, free
from libc.stdio cimport SEEK_SET
from libc.stdint cimport uint8_t, uint64_t, int64_t
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


cdef class _LinearLookup64s:
    cdef CppLinearLookup[int64_t]* data

    def __cinit__(self, list_of_hashes):
        self.data = new CppLinearLookup[int64_t](list_of_hashes)

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


class LinearLookup64s:
    def __init__(self, list_of_hashes):
        self.tree = _LinearLookup64s(list_of_hashes)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

    def size(self):
        return self.tree.size()


cdef extern from "cppbktree.hpp":
    size_t hammingDistance(const vector[uint8_t]&, const vector[uint8_t]& ) except +
    size_t hammingDistance64( const uint64_t, const uint64_t ) except +;
    size_t hammingDistance64s( const int64_t, const int64_t ) except +;

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
        CppBKTree(vector[T_ValueType]) except +
        void add(vector[T_ValueType]) except +
        vector[size_t] find(const T_ValueType&, unsigned short int) except +
        size_t size() except +
        TreeStatistics statistics() except +
        void rebalance(const size_t) except +


cdef class _BKTree:
    cdef CppBKTree[vector[uint8_t], size_t]* tree
    cdef int max_element_count
    cdef bool _needs_rebalance

    def __cinit__(self, list_of_hashes_or_file_name, max_element_count = 32 * 1024):
        self.tree = new CppBKTree[vector[uint8_t], size_t](<vector[vector[uint8_t]]>list_of_hashes_or_file_name)
        self.tree.rebalance(max_element_count)
        self.max_element_count = max_element_count
        self._needs_rebalance = False

    def __dealloc__(self):
        del self.tree

    def add(self, list_of_hashes_or_file_name):
        self.tree.add(<vector[vector[uint8_t]]>list_of_hashes_or_file_name)
        self._needs_rebalance = True

    def find(self, query, distance=0):
        if self._needs_rebalance:
            self.rebalance()
            self._needs_rebalance = False
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

    def rebalance(self, max_element_count = None):
        return self.tree.rebalance(self.max_element_count if max_element_count is None else <size_t>max_element_count)


cdef class _BKTree64:
    cdef CppBKTree[uint64_t, size_t]* tree
    cdef int max_element_count
    cdef bool _needs_rebalance

    def __cinit__(self, list_of_hashes_or_file_name, max_element_count = 32 * 1024):
        self.tree = new CppBKTree[uint64_t, size_t](list_of_hashes_or_file_name)
        self.tree.rebalance(max_element_count)
        self.max_element_count = max_element_count
        self._needs_rebalance = False

    def __dealloc__(self):
        del self.tree

    def add(self, list_of_hashes_or_file_name):
        self.tree.add(<vector[uint64_t]>list_of_hashes_or_file_name)
        self._needs_rebalance = True

    def find(self, query, distance=0):
        if self._needs_rebalance:
            self.rebalance()
            self._needs_rebalance = False
        return <list>(self.tree.find(query, distance))

    def size(self):
        return self.tree.size()

    def statistics(self):
        # Automatic POD to dict conversion did not work for me. Maybe because the contained types?
        cdef CppBKTree[uint64_t, size_t].TreeStatistics result = self.tree.statistics();
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

    def rebalance(self, max_element_count = None):
        return self.tree.rebalance(self.max_element_count if max_element_count is None else <size_t>max_element_count)


cdef class _BKTree64s:
    cdef CppBKTree[int64_t, size_t]* tree
    cdef int max_element_count
    cdef bool _needs_rebalance

    def __cinit__(self, list_of_hashes_or_file_name, max_element_count = 32 * 1024):
        self.tree = new CppBKTree[int64_t, size_t](list_of_hashes_or_file_name)
        self.tree.rebalance(max_element_count)
        self.max_element_count = max_element_count
        self._needs_rebalance = False

    def __dealloc__(self):
        del self.tree

    def add(self, list_of_hashes_or_file_name):
        self.tree.add(<vector[int64_t]>list_of_hashes_or_file_name)
        self._needs_rebalance = True

    def find(self, query, distance=0):
        if self._needs_rebalance:
            self.rebalance()
            self._needs_rebalance = False
        return <list>(self.tree.find(query, distance))

    def size(self):
        return self.tree.size()

    def statistics(self):
        # Automatic POD to dict conversion did not work for me. Maybe because the contained types?
        cdef CppBKTree[int64_t, size_t].TreeStatistics result = self.tree.statistics();
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

    def rebalance(self, max_element_count = None):
        return self.tree.rebalance(self.max_element_count if max_element_count is None else <size_t>max_element_count)


# Extra class because cdefs are not visible from outside
class BKTree:
    def __init__(self, list_of_hashes, max_element_count = 32 * 1024):
        self.tree = _BKTree(list_of_hashes, max_element_count)

    def add(self, list_of_hashes_or_file_name):
        self.tree.add(list_of_hashes_or_file_name)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

    def size(self):
        return self.tree.size()

    def statistics(self):
        return self.tree.statistics()

    def rebalance(self, max_element_count = None):
        return self.tree.rebalance(max_element_count)


class BKTree64:
    def __init__(self, list_of_hashes, max_element_count = 32 * 1024):
        self.tree = _BKTree64(list_of_hashes, max_element_count)

    def add(self, list_of_hashes_or_file_name):
        self.tree.add(list_of_hashes_or_file_name)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

    def size(self):
        return self.tree.size()

    def statistics(self):
        return self.tree.statistics()

    def rebalance(self, max_element_count = None):
        return self.tree.rebalance(max_element_count)


class BKTree64s:
    def __init__(self, list_of_hashes, max_element_count = 32 * 1024):
        self.tree = _BKTree64s(list_of_hashes, max_element_count)

    def add(self, list_of_hashes_or_file_name):
        self.tree.add(list_of_hashes_or_file_name)

    def find(self, query, distance=0):
        return self.tree.find(query, distance)

    def size(self):
        return self.tree.size()

    def statistics(self):
        return self.tree.statistics()

    def rebalance(self, max_element_count = None):
        return self.tree.rebalance(max_element_count)


__version__ = '0.2.0'
