
# Version 0.1.0 built on 2023-07-21

 - Chunk up to 8K (configurable) elements making the lookup a mix of BK tree traversal and linear
   lookup in the chunk. This improves performance more than 100x for large distances and datasets.

# Version 0.0.1 built on 2020-07-03

 - First hopefully stable version
