#ifndef HEAP
#define HEAP

#include "structures.h"

using namespace searchEnv;

//@function     testing function
void testor(void);
void writeOutput2File(QPoint *pt);
//@function     insert current in openlist heap     2*O(lg n)
void minHeapInsertF(QList<PUZZLENODE*> *openlist, PUZZLENODE* current, Environment *env);
//@function     decrease key of an element at position pos      O(lg n)
void heapDecreaseKeyF(QList<PUZZLENODE*> *openlist, qint64 pos, Environment *env);
//@function     build min heap      O(n)
void buildMinHeapF(QList<PUZZLENODE*> *openlist, Environment *env);
//@function     extract minimum     O(lg n)
PUZZLENODE* extractMinF(QList<PUZZLENODE*> *openlist, Environment *env);
//@function     min Heapify openlist    O(lg n)
void minHeapifyF(QList<PUZZLENODE*> *openlist, qint64 i, Environment *env);




// backward functions
void minHeapInsertB(QList<PUZZLENODE*> *openlist, PUZZLENODE* current, Environment *env);
void heapDecreaseKeyB(QList<PUZZLENODE*> *openlist, qint64 pos, Environment *env);
void buildMinHeapB(QList<PUZZLENODE*> *openlist, Environment *env);
PUZZLENODE* extractMinB(QList<PUZZLENODE*> *openlist, Environment *env);
void minHeapifyB(QList<PUZZLENODE*> *openlist, qint64 i, Environment *env);

#endif // HEAP

