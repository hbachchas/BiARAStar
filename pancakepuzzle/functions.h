#ifndef FUNCTIONS
#define FUNCTIONS

#include "structures.h"

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


void callMeFromCMD();
int prepareEnvironment(QString configFile, searchEnv::Environment *env);
void findSuccessor(PUZZLENODE *current, QList<PUZZLENODE *> *successors, searchEnv::Environment *env);
qint64 findHeuristic_F(PUZZLENODE *current, searchEnv::Environment *env);
qint64 findHeuristic_B(PUZZLENODE *current, searchEnv::Environment *env);
void getFlippedArray(int *intoArr, int *fromArr, int flipSize);
QString getHashKey(int *arr);
QString getHashKey(PUZZLENODE *current);
void initSymbolTable();
void printArray(int *array, QTextStream *out = NULL);
void printPath(searchEnv::Environment *env);
void printOpenlist(QList<PUZZLENODE*> *openlist);

int on_btnSearchARAstar_clicked();
int compareSearch(bool *signalUnfinishedSearch, QString searchType, searchEnv::Environment *env = NULL, ARASTARPRINTINFORMATION *printInfo = NULL, float decrement = -1);      // WA* and BiWA* search
bool canRunAnotherARAstarIteration(MYFLOAT epsilonDash);
int improvePathCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, PUZZLENODE *goal, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlist, QHash<QString, PUZZLENODE*> *inconslist,
                   searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);
int improvePatho1BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                              QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                              searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);
int improvePatho12BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);
int improvePatho12BiARAstarNEW(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);
int improvePatho123BiARAstarNEW(qint64 &HcorrectionForward, qint64 &minHerrNodeCountForward, MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);





int improvePatho123BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                                QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                                searchEnv::Environment *env, qint64 *HerrForward, qint64 *HerrBackward, ARASTARPRINTINFORMATION *printInfo);
int improvePatho123BiARAstarFCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                                 QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                                 searchEnv::Environment *env, qint64 *HerrForward, qint64 *HerrBackward, PUZZLENODE **ptrMinHerrNodeForward,
                                 PUZZLENODE **ptrMinHerrNodeBackward, ARASTARPRINTINFORMATION *printInfo);
int improvePathro12BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);      // sensing search
int improvePath_IKKA(QHash<QString, bool> *closedlistBackward, qint64 &HcorrectionForward, MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                              QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                              searchEnv::Environment *env, ARASTARPRINTINFORMATION *printInfo);



double findMinMYFLOAT(MYFLOAT num1, MYFLOAT num2);
int getRandomNumber(int low, int high);
void generateConfigFiles();



int getMean(QList<int> tempValuesList);
int getStdDev(qint64 mean, QList<int> tempValuesList);
void findMeanStdDev();
void findMeanStdDevARAstar();
void findWeightWiseIntersection();
void printOctaveQueries();
void copyFiles();    // copy from Korf benchmarks











#endif // FUNCTIONS

