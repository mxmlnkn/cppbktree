from cppbktree import BKTree, BKTree64

def test_bktree():
    tree = BKTree([bytes([x]) for x in [0, 4, 5, 14]])
    tree.add([bytes([15])])
    assert tree.find(bytes([13]), 1) == [2, 4]

def test_bktree64():
    tree64 = BKTree64([0, 4, 5, 14])
    tree64.add([15])

    # Find exact matches. The returned lists will be the indexes,
    # not the matching values itself in case there are hash collisions.
    assert tree64.find(0, 0) == [0]
    assert tree64.find(4, 0) == [1]
    assert tree64.find(5, 0) == [2]
    assert tree64.find(14, 0) == [3]
    assert tree64.find(15, 0) == [4]

    # Find inexact match to 13 = 0b1101
    #  -> 0b101  =  5 at index 2
    #  -> 0b1111 = 15 at index 4
    assert tree64.find(13, 1) == [2, 4]
