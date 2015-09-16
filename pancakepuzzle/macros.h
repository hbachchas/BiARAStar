#ifndef MACROS
#define MACROS

#define SWAP_INTS(a, b) b=(a+b)-(a=b)

#define DELETE_PUZZLENODE(node) delete [] *(node->array);\
    delete [] node->array;\
    delete node;

#endif // MACROS

