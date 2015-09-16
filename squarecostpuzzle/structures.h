#ifndef STRUCTURES
#define STRUCTURES

#include <QCoreApplication>
#include <QDebug>
#include <QVector>
#include <QPoint>
#include <QFile>
#include <QTextStream>
#include <QThread>
#include <QMutableListIterator>
#include <QRegularExpression>
#include <QtMath>
#include <QRect>
#include <QTime>
#include <QLinkedList>
#include <QPropertyAnimation>

#define MY_INFINITY 2000000000000   // 12 zeros
#define MAX_POSSIBLE_TIME_MACRO 20*1000 //5*60*1000       // total 5 min
#define TIME_INCREMENT_FACTOR 1000          // every sec
#define TOTAL_NUMBER_OF_CONFIG_FILES 50
#define FRACTIONAL_PRECISION 1000
#define SQRT_PRECISION 100

#define ARAstar_MIN_EPSILON 1.000000 //5//1       // for ARA* family
#define ARAstar_MAX_EPSILON 10.000000 //3       // for ARA* family
#define DECREMENT 0.5000000

#define CAUTION     // some measures to have utmost care in execution; slows down a bit
//#define DEBUG       // turns on general debugging
#define CALLFROMCOMMANDLINE       // turn this on when calling from command line
#define MYASSERT           // assertions
#define CONFIGFILEGENERATION    // suppress all printing


typedef struct myFloat{
    int int_p;          // Integer part, used for comparision
    int frac_p;         // Fractional part, used for comparision
    float float_p;      // Float value, used for calculation & updating above int values
    myFloat():
        int_p(-1), frac_p(-1), float_p(-1) {}
    myFloat(int i, int j, float f):
        int_p(i), frac_p(j), float_p(f) {}
public:
    inline void updateInt();       // Perform operation on float_p and call this function to update int_p & frac_p
    inline void updateFloat();     // Perform operation on int_p & frac_p and call this to update float_p
    inline void setFloat_p(float f);
    inline void setInt_p(int new_int_p, int new_frac_p);
}MYFLOAT;

inline void MYFLOAT::updateInt()
{
    this->int_p = (int) this->float_p;
    this->frac_p = (int)(this->float_p * FRACTIONAL_PRECISION) - this->int_p * FRACTIONAL_PRECISION;
}

inline void MYFLOAT::updateFloat()
{
    this->float_p = (float)(this->int_p * FRACTIONAL_PRECISION + this->frac_p) / FRACTIONAL_PRECISION;
}

inline void MYFLOAT::setFloat_p(float f)
{
    this->float_p = f;
    this->updateInt();
}

inline void MYFLOAT::setInt_p(int new_int_p, int new_frac_p)
{
    this->int_p = new_int_p;
    this->frac_p = new_frac_p;
    updateFloat();
}

namespace searchEnv {
    struct puzzle;
    struct printInformation;
    class Environment;
}

typedef struct _Point
{
    int x,y;
    _Point():x(-1),y(-1){}
    _Point(int i, int j):x(i),y(j){}
public:
    QString getPointString();
} POINT;

typedef struct searchEnv::puzzle       // stores information pertaining to a state
{
    int **array;

    struct searchEnv::puzzle *parentForward;
    qint64 fForward;
    qint64 gForward;
    qint64 vForward;
    bool onClosedListForward;
    qint64 heapIndexForward;
    bool visitedForward;

    struct searchEnv::puzzle *parentBackward;
    qint64 gBackward;
    qint64 fBackward;
    qint64 vBackward;
    bool onClosedListBackward;
    qint64 heapIndexBackward;
    bool visitedBackward;

    puzzle():
        fForward(MY_INFINITY), gForward(MY_INFINITY), fBackward(MY_INFINITY), gBackward(MY_INFINITY), vForward(MY_INFINITY), vBackward(MY_INFINITY),
        heapIndexForward(-1),
        heapIndexBackward(-1),
        onClosedListForward(false),
        onClosedListBackward(false),
        parentForward(NULL),
        parentBackward(NULL),
        visitedForward(false),
        visitedBackward(false),
        array(NULL)
    {}    // constructor initialization
    puzzle(int **arr, int size);    // copy constructor
} PUZZLENODE;

class searchEnv::Environment
{
public:
    PUZZLENODE *startForward, *goalForward;
    PUZZLENODE *startBackward, *goalBackward;
    int puzzleSize;
    QVector<POINT*> goalPointsForward;
    QVector<POINT*> goalPointsBackward;
    QHash<QString, PUZZLENODE*> hashTable;

    MYFLOAT epsilon;
    qint64 numberOfNodesExpandedForward;
    qint64 numberOfNodesExpandedBackward;
    qint64 arastarTotalNodesExpanded;
    qint64 arastarTotalTimeTaken;

    Environment():
        numberOfNodesExpandedForward(0), numberOfNodesExpandedBackward(0), arastarTotalNodesExpanded(0), arastarTotalTimeTaken(0),
        startForward(NULL), startBackward(NULL), goalForward(NULL), goalBackward(NULL) {}
};

typedef struct searchEnv::printInformation     // Contains printing information
{
    qint64 solutionCost;
    qint64 runtime;
    qint64 noOfExpansions;
    QString directionMap;
    printInformation():
        solutionCost(MY_INFINITY),
        runtime(MY_INFINITY),
        noOfExpansions(MY_INFINITY),
        directionMap(""){}
    printInformation(qint64 solc, qint64 runt, qint64 noe):
        solutionCost(solc), runtime(runt), noOfExpansions(noe) {}
} PRINTINFORMATION;

typedef struct ARAstarPrintInformation     // Contains printing information
{
    qint64 solutionCost;
    MYFLOAT bound;
    qint64 timeInstant;
    ARAstarPrintInformation():
        solutionCost(MY_INFINITY),
        bound(MY_INFINITY, MY_INFINITY, MY_INFINITY),
        timeInstant(MY_INFINITY){}
    ARAstarPrintInformation(qint64 solc, MYFLOAT bnd, qint64 tmInst):
        solutionCost(solc), bound(bnd), timeInstant(tmInst) {}
} ARASTARPRINTINFORMATION;










#endif // STRUCTURES

