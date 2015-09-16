#ifndef MACROS
#define MACROS

#define SWAP_INTS(a, b) b=(a+b)-(a=b)

#define DELETE_PUZZLENODE(node) delete [] *(node->array);\
    delete [] node->array;\
    delete node;

#define CURRENT_BLANK_POSITION(currentBlankPos, current) currentBlankPos.x = 0+\
    !current->array[1][0]*1+\
    !current->array[1][1]*1+\
    !current->array[1][2]*1+\
    !current->array[2][0]*2+\
    !current->array[2][1]*2+\
    !current->array[2][2]*2;\
    currentBlankPos.y = 0+\
    !current->array[0][1]*1+\
    !current->array[0][2]*2+\
    !current->array[1][1]*1+\
    !current->array[1][2]*2+\
    !current->array[2][1]*1+\
    !current->array[2][2]*2;

#define GETHASHKEY3(a,b,c,d,e,f,g,h,i,str) \
    arr[0] = 48 + a;\
    arr[1] = 48 + b;\
    arr[2] = 48 + c;\
    arr[3] = 48 + d;\
    arr[4] = 48 + e;\
    arr[5] = 48 + f;\
    arr[6] = 48 + g;\
    arr[7] = 48 + h;\
    arr[8] = 48 + i;\
    arr[9] = '\0';\
    str = QString::fromRawData(arr, 9);

#define GETHASHKEY4(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,str) \
    str = QString::number(a1)\
    +QString::number(a2)\
    +QString::number(a3)\
    +QString::number(a4)\
    +QString::number(a5)\
    +QString::number(a6)\
    +QString::number(a7)\
    +QString::number(a8)\
    +QString::number(a9)\
    +QString::number(a10)\
    +QString::number(a11)\
    +QString::number(a12)\
    +QString::number(a13)\
    +QString::number(a14)\
    +QString::number(a15)\
    +QString::number(a16);

#define GETHASHKEY5(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25, str) \
    str = QString::number(a1)\
    +QString::number(a2)\
    +QString::number(a3)\
    +QString::number(a4)\
    +QString::number(a5)\
    +QString::number(a6)\
    +QString::number(a7)\
    +QString::number(a8)\
    +QString::number(a9)\
    +QString::number(a10)\
    +QString::number(a11)\
    +QString::number(a12)\
    +QString::number(a13)\
    +QString::number(a14)\
    +QString::number(a15)\
    +QString::number(a16)\
    +QString::number(a17)\
    +QString::number(a18)\
    +QString::number(a19)\
    +QString::number(a20)\
    +QString::number(a21)\
    +QString::number(a22)\
    +QString::number(a23)\
    +QString::number(a24)\
    +QString::number(a25);


#endif // MACROS

