
#include "structures.h"
#include "functions.h"
#include "heap.h"
#include "macros.h"

#define PANCAKE_COST 1

bool boolPUZZLESIZE3 = false;
bool boolPUZZLESIZE4 = false;
bool boolPUZZLESIZE5 = false;
bool boolPUZZLESIZE7 = false;
bool boolPUZZLESIZE9 = false;

int GPuzzleSize = -1;
QList<QString> GSymbolList;
QString GConfigFilePath = "";

//---------------------------- i is row, j is col, typical matrix ordering--------------------------//
//--------------------puzzleSize is 'n' but array size is 'n+1' which includes the plate as well-----------------//
//-----------------------Don't go beyond 80 symbols, otherwise you will run out of ascii symbols

void callMeFromCMD()    // parameter 1-config 2-epsilon 3-searchType 4-outfile
{
    //QString asdfsdfsfd = "/\~`!@#$%^&*()_+=-';:?.>,<[]{}\|*-+.";      // testing

    GConfigFilePath = QCoreApplication::arguments().at(1);
    //GConfigFilePath = "/home/peace/Projects/Qt/searchalgos/pancakePuzzle/pancakePuzzleConsole/resources/70puz/env13.cfg";
    on_btnSearchARAstar_clicked();

    //findWeightWiseIntersection();
    //findMeanStdDevARAstar();
    //printOctaveQueries();
    //copyFiles();
    //generateConfigFiles();


    /*MYFLOAT num1, num2; num1.setFloat_p(ARAstar_MAX_EPSILON); num2.setFloat_p(1.25);
    num1.setFloat_p(num1.float_p - DECREMENT);
    qDebug() << (int)(num1.float_p * FRACTIONAL_PRECISION);
    qDebug() << findMinMYFLOAT(num1, num2);
    num2.setFloat_p(num2.float_p - .25);
    qDebug() << num2.float_p;
    qDebug() << num1.float_p;*/
}

//int prepareEnvironment(QString configFile, searchEnv::Environment *env, QVector<QPoint*> *goalPoints)
int prepareEnvironment(QString configFile, searchEnv::Environment *env)
{
    // we have not done rigrious exception handling, we leave onous error reduction on user
    QFile file(configFile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Error file opening the config file, Exiting...";
        return -1;
    }

    QTextStream in(&file);
    QStringList spaceSplitted;
    QRegularExpression REspace(" ");
    bool ok;

    QString line = in.readLine();   // puzzle size
    line = in.readLine();
    env->puzzleSize = line.toInt(&ok);
    env->startForward->array = new int [env->puzzleSize + 1];   env->startForward->array[env->puzzleSize] = env->puzzleSize;    // adding the plate
    env->goalForward->array = new int [env->puzzleSize + 1];    env->goalForward->array[env->puzzleSize] = env->puzzleSize;     // adding the plate

    line = in.readLine();   // read start
    line = in.readLine();
    spaceSplitted = line.split(REspace);
    for (int i=0; i<env->puzzleSize; i++)
    {
        if (!spaceSplitted.at(i).isEmpty())
            env->startForward->array[i] = spaceSplitted.at(i).toInt(&ok);
        else
        {
            qDebug() << "Error in reading start state in prepareEnvironment";
            return 1;
        }
    }

    line = in.readLine();   // read goal
    line = in.readLine();
    spaceSplitted = line.split(REspace);
    for (int i=0; i<env->puzzleSize; i++)
    {
        if (!spaceSplitted.at(i).isEmpty())
            env->goalForward->array[i] = spaceSplitted.at(i).toInt(&ok);
        else
        {
            qDebug() << "Error in reading start state in prepareEnvironment";
            return 1;
        }
    }

    file.close();
    return 0;   // success
}

void findSuccessor(PUZZLENODE *current, QList<PUZZLENODE *> *successors, searchEnv::Environment *env)
{
    PUZZLENODE *temp1 = NULL;
    int *arr = new int [env->puzzleSize + 1];   arr[env->puzzleSize] = env->puzzleSize;     // adding the plate
    QString hashKey;

    for (int flipSize=2; flipSize <= env->puzzleSize; flipSize++)
    {
        // flip
        getFlippedArray(arr, current->array, flipSize);
        hashKey = getHashKey(arr);
        if ( !(temp1 = env->hashTable.value(hashKey, false)) )
        {
            temp1 = new PUZZLENODE(arr, env->puzzleSize);
            env->hashTable.insert(hashKey, temp1);
        }
        successors->append(temp1);
#ifdef CAUTION
        temp1 = NULL;
#endif
    }
    delete [] arr;
}

inline qint64 findHeuristic_F(PUZZLENODE *current, searchEnv::Environment *env)
{
    //---------------------GAP Heuristic--------------------//      // goal automatically has heuristic value 0
    int sum = 0;
    //printArray(current->array);

    for (int i=0; i<env->puzzleSize; i++)
        if ( qAbs(current->array[i] - current->array[i+1]) > 1 )
            sum++;
    return sum;
}

inline qint64 findHeuristic_B(PUZZLENODE *current, searchEnv::Environment *env)
{
    //---------------------GAP Heuristic--------------------//      // goal automatically has heuristic value 0
    int sum = 0;
    //printArray(current->array);

    for (int i=0; i<env->puzzleSize; i++)
        if ( qAbs(current->array[i] - current->array[i+1]) > 1 )
            sum++;
    return sum;
}

inline void getFlippedArray(int *intoArr, int *fromArr, int flipSize)
{
    for (int i=0; i<flipSize; i++)
        intoArr[i] = fromArr[flipSize-1-i];
    for (int i=flipSize; i<GPuzzleSize; i++)
        intoArr[i] = fromArr[i];
}

inline QString getHashKey(int *arr)
{
    QString hashKey = "";
    for (int i=0; i<GPuzzleSize; i++)
        hashKey = hashKey + GSymbolList.at(arr[i]);
    return hashKey;
}

inline QString getHashKey(PUZZLENODE *current)
{
    QString hashKey = "";
    for (int i=0; i<GPuzzleSize; i++)
        hashKey = hashKey + GSymbolList.at(current->array[i]);
    return hashKey;
}

inline void initSymbolTable()
{
    for (int i=0; i<GPuzzleSize; i++)
        GSymbolList.append(QChar(45+i));    // can go upto 80 symbols
}

PUZZLENODE::puzzle(int *arr, int size):
    fForward(MY_INFINITY), gForward(MY_INFINITY), fBackward(MY_INFINITY), gBackward(MY_INFINITY), vForward(MY_INFINITY), vBackward(MY_INFINITY),
    heapIndexForward(-1),
    heapIndexBackward(-1),
    parentForward(NULL),
    parentBackward(NULL),
    visitedForward(false),
    visitedBackward(false),
    array(NULL)
{
    this->array = new int [size + 1];   this->array[size] = size;   // adding the plate
    for (int i=0; i<size; i++)
        this->array[i] = arr[i];
}

QString POINT::getPointString()
{
    return QString::number(this->x) + "," + QString::number(this->y);
}

void printArray(int *array, QTextStream *out)
{
    if (!out)   // QTextStream is NULL
    {
        QString line;
        for (int i=0; i<GPuzzleSize; i++)
        {
            if ( array[i] < 10 )
                line = line + "  " + QString::number(array[i]);
            else
                line = line + " " + QString::number(array[i]);
        }
        qDebug() << line << "       " << getHashKey(array) << endl;
    }
    else
    {
        QString line;
        for (int i=0; i<GPuzzleSize; i++)
        {
            if ( array[i] < 10 )
                line = line + "  " + QString::number(array[i]);
            else
                line = line + " " + QString::number(array[i]);
        }
        (*out) << line << "       " << getHashKey(array) << endl;
    }
}

void printPath(searchEnv::Environment *env)
{
    QList<PUZZLENODE *> printList;
    PUZZLENODE *temp = env->goalForward;

    for (; temp!=NULL; temp = temp->parentForward)
        printList.prepend(temp);

    qDebug() << "======================";
    qDebug() << "    SOLUTION PATH";
    qDebug() << "======================";

    QFile file("/home/peace/pancakePath.txt");
    if (!file.open(QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "Error while opening the result file: from printPath()";
        return;
    }
    QTextStream out(&file);

    for (int i=0; i<printList.size();i++)
    {
        out << "f= " << printList.at(i)->fForward << "     g= " << printList.at(i)->gForward
            << "  h= " << printList.at(i)->fForward - printList.at(i)->gForward << endl;
        if ( i != 0 )
        {
            int num1 = printList.at(i-1)->array[0], num2 = -1;
            for (int j=0; j<env->puzzleSize; j++)
                if (printList.at(i)->array[j] == num1)
                    num2 = j;
            out << "|||||||||||||||||||||||||||||||||||||||||||||     " << num2+1 << "    ";
        }
        printArray(printList.at(i)->array, &out);
    }
}

void printOpenlist(QList<PUZZLENODE*> *openlist)
{
    QString openlistStr = "";
    for (qint64 i=0; i<openlist->size(); i++)
    {
        openlistStr = openlistStr + QString::number(openlist->at(i)->fForward) + "  " +
                getHashKey(openlist->at(i)) + "  ";
    }
    qDebug() << "openlist: " << openlistStr;
}













int on_btnSearchARAstar_clicked()
{
    bool ok;
    bool signalUnfinishedSearch = false;
    QString configFile = GConfigFilePath;
    //int searchTypeInt = 5;
    int searchTypeInt = QCoreApplication::arguments().at(2).toInt(&ok);
    //QString resultPath = "/home/peace/Projects/Qt/searchalgos/pancakePuzzle/pancakePuzzleConsole/resources/results/";
    QString resultPath = QCoreApplication::arguments().at(3);

    searchEnv::Environment env;
    QString searchType = "";
    QString searchTypeAlphabet = "";

    if ( searchTypeInt == 1 )
    {
        searchType = "ARA*";
        searchTypeAlphabet = "a";
    }
    else if ( searchTypeInt == 2 )
    {
        searchType = "o1BiARA*";
        searchTypeAlphabet = "b";
    }
    else if ( searchTypeInt == 3 )
    {
        searchType = "o12BiARA*";
        searchTypeAlphabet = "c";
    }
    else if ( searchTypeInt == 4 )
    {
        //searchType = "o123BiARA*";
        searchType = "o12BiARA*NEW";
        searchTypeAlphabet = "d";
    }
    else if ( searchTypeInt == 5 )
    {
        //searchType = "o123BiARA*F";
        searchType = "o123BiARA*NEW";
        searchTypeAlphabet = "e";
    }
    else if ( searchTypeInt == 6 )
    {
        //searchType = "ro12BiARA*";
        searchType = "IKKA_ARA*";
        searchTypeAlphabet = "f";
    }

    env.startForward = new PUZZLENODE;
    env.goalForward = new PUZZLENODE;

    // Set start and goal for backward
    env.startBackward = env.goalForward;
    env.goalBackward = env.startForward;

    // set the start and goal states; prepare environment
    if ( prepareEnvironment(configFile, &env) )
    {
        qDebug() << "Error in prepareEnvironment, Exiting...";
        return 1;
    }
    GPuzzleSize = env.puzzleSize;    // GLOBAL SUBSTITUTION
    initSymbolTable();      // initialize the symbols

#ifndef CONFIGFILEGENERATION
    qDebug() << "Start state:";
    printArray(env.startForward->array, env.puzzleSize, env.puzzleSize);
    qDebug() << "Goal state:";
    printArray(env.goalForward->array, env.puzzleSize, env.puzzleSize);
#endif

    QString hashKey;
    // start node hashKey calculation
    hashKey = getHashKey(env.startForward);
    env.hashTable.insert(hashKey, env.startForward);
    // goal node hashKey calculation
    hashKey = getHashKey(env.goalForward);
    env.hashTable.insert(hashKey, env.goalForward);

    ARASTARPRINTINFORMATION printInfo[(qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR)];  // for number of ARA* executions
    for (int i = 0; i<(qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR); i++)
        printInfo[i].timeInstant = (i+1) * TIME_INCREMENT_FACTOR;

    if ( compareSearch( &signalUnfinishedSearch, searchType, &env, printInfo, DECREMENT ) )    // Failure
    {
        qDebug() << "\nSearch " + searchType + " failed! Couldn't find a path!";
        qDebug() << configFile;
        return 1;
    }

    for (int timecount = 0; timecount<((qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR)); timecount++)
    {
        if ( printInfo[timecount].solutionCost == MY_INFINITY )
        {
            if ( timecount != 0 )    // replace MY_INFINITY values
            {
                printInfo[timecount].bound.setFloat_p(printInfo[timecount-1].bound.float_p);
                printInfo[timecount].solutionCost = printInfo[timecount-1].solutionCost;
            }
        }
        QString fileName;

        // Writing to a path cost file
        fileName = "p" + searchTypeAlphabet + QString::number(printInfo[timecount].timeInstant);

        //qDebug() << "this is first result file path";
        //qDebug() << resultPath + fileName + ".csv";

        QFile file(resultPath + fileName + ".csv");
        if (!file.open(QIODevice::Append | QIODevice::Text))
        {
            qDebug() << "Error while opening the result file: from comapreARAstarBlockbasedSearchCCL";
            return 1;
        }
        QTextStream out1(&file);
        if ( !signalUnfinishedSearch )
            out1 << " " << QString::number( printInfo[timecount].solutionCost ) << endl;
        else
            out1 << " #######" << endl;
        file.close();

        // Writing to a bound file
        fileName = "b" + searchTypeAlphabet + QString::number(printInfo[timecount].timeInstant);
        file.setFileName(resultPath + fileName + ".csv");
        if (!file.open(QIODevice::Append | QIODevice::Text))
        {
            qDebug() << "Error while opening the result file: ";
            return 1;
        }
        QTextStream out2(&file);
        if ( !signalUnfinishedSearch )
            out2 << " " << QString::number(printInfo[timecount].bound.float_p) << endl;
        else
            out2 << " #######" << endl;
        file.close();
    }

    //printPath(&env);    // testing
    return 0;
}

int compareSearch(bool *signalUnfinishedSearch, QString searchType, Environment *env, ARASTARPRINTINFORMATION *printInfo, float decrement)
{
    /*QList<ARASTARPRINTINFORMATION *> printinfolist;
    for (int i = 0; i<(qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR); i++)
        printinfolist.append(&printInfo[i]);*/
    if (searchType == "ARA*")   // search 1
    {
        QTime t;
        t.start();

        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlist;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslist;

        PUZZLENODE *goal = env->goalForward;
        goal->parentForward = NULL;
        goal->gForward = MY_INFINITY;
        goal->fForward = goal->gForward;    // fvalueForward(goal, env, &_epsilon);

        PUZZLENODE *current = env->startForward;    // start node
        current->gForward = 0;
        current->fForward = current->gForward +  _epsilon.float_p * findHeuristic_F(current, env);  //+findLC_F(env->startForward,env));
        current->visitedForward = true;
        current->parentForward = NULL;
        minHeapInsertF(&openlist, current, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePathCCL(&epsilonDash, signalUnfinishedSearch, t, goal, &_epsilon, &openlist, &inconslist, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlist.size(); i++)
        {
            bufferInt = openlist.at(i)->gForward + findHeuristic_F(openlist.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslist.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslist.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslist.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(env->goalForward->gForward / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // merge openlist with inconslist
            QList<QString> keyList = inconslist.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslist.size()!=keyList.size() )
            {
                qDebug() << "FAILURE: merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyList.size(); i++)
            {
                //inconslist.value(keyList.at(i))->onClosedListForward = false;
                openlist.append(inconslist.value(keyList.at(i)));
                inconslist.remove(keyList.at(i));
            }
            for (qint64 i=0; i<openlist.size(); i++)     // update f values for new epsilon
                openlist.at(i)->fForward = openlist.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlist.at(i), env);

            buildMinHeapF(&openlist, env);
            //inconslist.clear();
#ifdef MYASSERT_VALIDATE_HEAP     // heap debug [12-June-15]
            if ( validateHeap(&openlist) )     // either heap inconstent or inconslist has some node
            {
                qDebug() << "FAILURE: returning from validateHeap in ARA* main(), abrupt termination";
                return 1;
            }
#endif

            if (improvePathCCL(&epsilonDash, signalUnfinishedSearch, t, goal, &_epsilon, &openlist, &inconslist, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlist.size(); i++)
            {
                bufferInt = openlist.at(i)->gForward + findHeuristic_F(openlist.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslist.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslist.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslist.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(env->goalForward->gForward / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

            /*if (printinfolist.at(0)->bound.int_p == 1)
                qDebug() << "stop here";*/
        }
        return 0;   // success
    }
    else if (searchType == "o1BiARA*")  // search 2
    {
        QTime t;
        t.start();

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        /*PUZZLENODE *goalForward = new PUZZLENODE;
        goalForward->ptMe = env->ptGoalForward;
        goalForward->parentForward = NULL;
        goalForward->gForward = MY_INFINITY;    // single most important line
        goalForward->fForward = fvalueForward(goalForward, env, &epsilon);
        env->grid[goalForward->ptMe.x()][goalForward->ptMe.y()].link = goalForward;*/

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward +  _epsilon.float_p * findHeuristic_F(currentForward, env);  //+findLC_F(env->startForward,env));
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePatho1BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                      costIncumbentSolution, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                openlistForward.append(inconslistForward.value(keyListF.at(i)));
                inconslistForward.remove(keyListF.at(i));
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                inconslistBackward.remove(keyListB.at(i));
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePatho1BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                          costIncumbentSolution, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    else if (searchType == "o12BiARA*") // search 3
    {
        QTime t;
        t.start();

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward + _epsilon.float_p * findHeuristic_F(currentForward, env);
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePatho12BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                       costIncumbentSolution, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                if ( inconslistForward.value(keyListF.at(i))->vForward > (qint64)(_epsilon.float_p*findHeuristic_B(inconslistForward.value(keyListF.at(i)), env)) )
                {
                    openlistForward.append(inconslistForward.value(keyListF.at(i)));
                    inconslistForward.remove(keyListF.at(i));
                }
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                if ( inconslistBackward.value(keyListB.at(i))->vBackward > (qint64)(_epsilon.float_p*findHeuristic_F(inconslistBackward.value(keyListB.at(i)), env)) )
                {
                    openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                    inconslistBackward.remove(keyListB.at(i));
                }
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePatho12BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                           costIncumbentSolution, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    else if (searchType == "o12BiARA*NEW")  // search 4
    {
        QTime t;
        t.start();

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward + _epsilon.float_p * findHeuristic_F(currentForward, env);
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePatho12BiARAstarNEW(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                       costIncumbentSolution, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                if ( inconslistForward.value(keyListF.at(i))->vForward > (qint64)(_epsilon.float_p*findHeuristic_B(inconslistForward.value(keyListF.at(i)), env)) )
                {
                    openlistForward.append(inconslistForward.value(keyListF.at(i)));
                    inconslistForward.remove(keyListF.at(i));
                }
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                if ( inconslistBackward.value(keyListB.at(i))->vBackward > (qint64)(_epsilon.float_p*findHeuristic_F(inconslistBackward.value(keyListB.at(i)), env)) )
                {
                    openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                    inconslistBackward.remove(keyListB.at(i));
                }
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePatho12BiARAstarNEW(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                           costIncumbentSolution, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    else if (searchType == "o123BiARA*NEW")  // search 5
    {
        QTime t;
        t.start();

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward + _epsilon.float_p * findHeuristic_F(currentForward, env);
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        qint64 HcorrectionForward = 0;
        qint64 minHerrNodeCountForward = 1;

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePatho123BiARAstarNEW(HcorrectionForward, minHerrNodeCountForward, &epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                       costIncumbentSolution, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env) + HcorrectionForward;
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env) + HcorrectionForward;
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                if ( inconslistForward.value(keyListF.at(i))->vForward > (qint64)(_epsilon.float_p*findHeuristic_B(inconslistForward.value(keyListF.at(i)), env)) )
                {
                    openlistForward.append(inconslistForward.value(keyListF.at(i)));
                    inconslistForward.remove(keyListF.at(i));
                }
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                if ( inconslistBackward.value(keyListB.at(i))->vBackward > (qint64)(_epsilon.float_p*findHeuristic_F(inconslistBackward.value(keyListB.at(i)), env)) )
                {
                    openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                    inconslistBackward.remove(keyListB.at(i));
                }
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePatho123BiARAstarNEW(HcorrectionForward, minHerrNodeCountForward, &epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                           costIncumbentSolution, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env) + HcorrectionForward;
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env) + HcorrectionForward;
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }

    else if (searchType == "o123BiARA*")
    {
        QTime t;
        t.start();

        qint64 HerrForward = 0, HerrBackward = 0;

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward + _epsilon.float_p * findHeuristic_F(currentForward, env);
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePatho123BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                        costIncumbentSolution, env, &HerrForward, &HerrBackward, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                if ( inconslistForward.value(keyListF.at(i))->vForward > (qint64)(_epsilon.float_p*findHeuristic_B(inconslistForward.value(keyListF.at(i)), env)) )
                {
                    openlistForward.append(inconslistForward.value(keyListF.at(i)));
                    inconslistForward.remove(keyListF.at(i));
                }
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                if ( inconslistBackward.value(keyListB.at(i))->vBackward > (qint64)(_epsilon.float_p*findHeuristic_F(inconslistBackward.value(keyListB.at(i)), env)) )
                {
                    openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                    inconslistBackward.remove(keyListB.at(i));
                }
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePatho123BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                            costIncumbentSolution, env, &HerrForward, &HerrBackward, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    else if (searchType == "o123BiARA*F")
    {
        QTime t;
        t.start();

        qint64 HerrForward = 0, HerrBackward = 0;

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward + _epsilon.float_p * findHeuristic_F(currentForward, env);
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        PUZZLENODE *minHerrNodeForward = currentBackward, *minHerrNodeBackward = currentForward;
        HerrForward = minHerrNodeForward->gBackward - findHeuristic_F(minHerrNodeForward, env);
        HerrBackward = minHerrNodeBackward->gForward - findHeuristic_B(minHerrNodeBackward, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePatho123BiARAstarFCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                         costIncumbentSolution, env, &HerrForward, &HerrBackward, &minHerrNodeForward, &minHerrNodeBackward,
                                         printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                if ( inconslistForward.value(keyListF.at(i))->vForward > (qint64)(_epsilon.float_p*findHeuristic_B(inconslistForward.value(keyListF.at(i)), env)) )
                {
                    openlistForward.append(inconslistForward.value(keyListF.at(i)));
                    inconslistForward.remove(keyListF.at(i));
                }
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                if ( inconslistBackward.value(keyListB.at(i))->vBackward > (qint64)(_epsilon.float_p*findHeuristic_F(inconslistBackward.value(keyListB.at(i)), env)) )
                {
                    openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                    inconslistBackward.remove(keyListB.at(i));
                }
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePatho123BiARAstarFCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                             costIncumbentSolution, env, &HerrForward, &HerrBackward, &minHerrNodeForward, &minHerrNodeBackward,
                                             printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    else if (searchType == "ro12BiARA*")
    {
        QTime t;
        t.start();

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward + _epsilon.float_p * findHeuristic_F(currentForward, env);
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward + _epsilon.float_p * findHeuristic_B(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePathro12BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                        costIncumbentSolution, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                if ( inconslistForward.value(keyListF.at(i))->vForward > (qint64)(_epsilon.float_p*findHeuristic_B(inconslistForward.value(keyListF.at(i)), env)) )
                {
                    //env->grid[inconslistForward.value(keyListF.at(i))->ptMe.x()][inconslistForward.value(keyListF.at(i))->ptMe.y()].onClosedListForward = false;
                    openlistForward.append(inconslistForward.value(keyListF.at(i)));
                    inconslistForward.remove(keyListF.at(i));
                }
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                if ( inconslistBackward.value(keyListB.at(i))->vBackward > (qint64)(_epsilon.float_p*findHeuristic_F(inconslistBackward.value(keyListB.at(i)), env)) )
                {
                    //env->grid[inconslistBackward.value(keyListB.at(i))->ptMe.x()][inconslistBackward.value(keyListB.at(i))->ptMe.y()].onClosedListBackward = false;
                    openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                    inconslistBackward.remove(keyListB.at(i));
                }
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);

            if (improvePathro12BiARAstarCCL(&epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                            costIncumbentSolution, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)tempInt));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    else if (searchType == "IKKA_ARA*")
    {
        QTime t;
        t.start();

        qint64 costIncumbentSolution = MY_INFINITY;
        MYFLOAT _epsilon; _epsilon.setFloat_p(ARAstar_MAX_EPSILON);      // weight
        env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

        QList<PUZZLENODE *> openlistForward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistForward;

        /*PUZZLENODE *goalForward = new PUZZLENODE;
        goalForward->ptMe = env->ptGoalForward;
        goalForward->parentForward = NULL;
        goalForward->gForward = MY_INFINITY;    // single most important line
        goalForward->fForward = fvalueForward(goalForward, env, &epsilon);
        env->grid[goalForward->ptMe.x()][goalForward->ptMe.y()].link = goalForward;*/

        PUZZLENODE *currentForward = env->startForward;    // start node
        currentForward->gForward = 0;
        currentForward->fForward = currentForward->gForward +  _epsilon.float_p * findHeuristic_F(currentForward, env);  //+findLC_F(env->startForward,env));
        currentForward->visitedForward = true;
        currentForward->parentForward = NULL;
        minHeapInsertF(&openlistForward, currentForward, env);

        QList<PUZZLENODE *> openlistBackward;    // priority Queue
        QHash<QString, PUZZLENODE*> inconslistBackward;

        PUZZLENODE *currentBackward = env->startBackward;    // start node
        currentBackward->gBackward = 0;
        currentBackward->fBackward = currentBackward->gBackward - findHeuristic_F(currentBackward, env);
        currentBackward->visitedBackward = true;
        currentBackward->parentBackward = NULL;
        minHeapInsertB(&openlistBackward, currentBackward, env);

        qint64 HcorrectionForward = 0;
        QHash<QString, bool> closedlistBackward;

        MYFLOAT epsilonDash(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p), optepsilon(_epsilon.int_p, _epsilon.frac_p, _epsilon.float_p);
        if (improvePath_IKKA(&closedlistBackward, HcorrectionForward, &epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                      costIncumbentSolution, env, printInfo))
        {
            qDebug() << "FAILURE: returning from improvePath, before while()";
            return 1;
        }
        // find MIN epsilon
        qint64 tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
        for (qint64 i=0; i<openlistForward.size(); i++)
        {
            bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        /*for (qint64 i=0; i<openlistBackward.size(); i++)
        {
            bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }*/
        QList<QString> tempKeyList = inconslistForward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }
        /*tempKeyList = inconslistBackward.uniqueKeys();
        for (qint64 i=0; i<tempKeyList.size(); i++)
        {
            bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
            if ( tempInt > bufferInt )
                tempInt = bufferInt;
        }*/
        optepsilon.setFloat_p((float)(costIncumbentSolution / (float)(tempInt + HcorrectionForward)));
        epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );

        while( canRunAnotherARAstarIteration(epsilonDash) )  // while(goal node is not expanded)
        {
            _epsilon.setFloat_p(_epsilon.float_p - decrement);
            env->epsilon.setFloat_p(_epsilon.float_p);         // for debugging heap

            // ---------FORWARD MERGE----------//
            QList<QString> keyListF = inconslistForward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistForward.size() != keyListF.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListF.size(); i++)
            {
                openlistForward.append(inconslistForward.value(keyListF.at(i)));
                inconslistForward.remove(keyListF.at(i));
            }
            for (qint64 i =0; i<openlistForward.size(); i++)   // update f values for new epsilon in inconslistForward
                openlistForward.at(i)->fForward = openlistForward.at(i)->gForward + _epsilon.float_p * findHeuristic_F(openlistForward.at(i), env);
            buildMinHeapF(&openlistForward, env);

            // ---------BACKWARD MERGE----------//
            /*QList<QString> keyListB = inconslistBackward.uniqueKeys();
#ifdef MYASSERT     // heap debug [12-June-15]
            if ( inconslistBackward.size() != keyListB.size() )
            {
                qDebug() << "FAILURE: in o1BiARA*, merging openlist with inconslist, there is some repeatition in inconslist, abrupt termination";
                return 1;
            }
#endif
            for (qint64 i=0; i<keyListB.size(); i++)
            {
                openlistBackward.append(inconslistBackward.value(keyListB.at(i)));
                inconslistBackward.remove(keyListB.at(i));
            }
            for (qint64 i =0; i<openlistBackward.size(); i++)   // update f values for new epsilon in inconslistBackward
                openlistBackward.at(i)->fBackward = openlistBackward.at(i)->gBackward + _epsilon.float_p * findHeuristic_B(openlistBackward.at(i), env);
            buildMinHeapB(&openlistBackward, env);*/

            if (improvePath_IKKA(&closedlistBackward, HcorrectionForward, &epsilonDash, signalUnfinishedSearch, t, &_epsilon, &openlistForward, &inconslistForward, &openlistBackward, &inconslistBackward,
                                          costIncumbentSolution, env, printInfo))
            {
                qDebug() << "FAILURE: returning from improvePath";
                return 1;
            }
            // find MIN epsilon
            tempInt = MY_INFINITY, bufferInt = MY_INFINITY;
            for (qint64 i=0; i<openlistForward.size(); i++)
            {
                bufferInt = openlistForward.at(i)->gForward + findHeuristic_F(openlistForward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            /*for (qint64 i=0; i<openlistBackward.size(); i++)
            {
                bufferInt = openlistBackward.at(i)->gBackward + findHeuristic_B(openlistBackward.at(i), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }*/
            tempKeyList = inconslistForward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistForward.value(tempKeyList.at(i))->gForward + findHeuristic_F(inconslistForward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }
            /*tempKeyList = inconslistBackward.uniqueKeys();
            for (qint64 i=0; i<tempKeyList.size(); i++)
            {
                bufferInt = inconslistBackward.value(tempKeyList.at(i))->gBackward + findHeuristic_B(inconslistBackward.value(tempKeyList.at(i)), env);
                if ( tempInt > bufferInt )
                    tempInt = bufferInt;
            }*/
            optepsilon.setFloat_p((float)(costIncumbentSolution / (float)(tempInt + HcorrectionForward)));
            epsilonDash.setFloat_p( (float)findMinMYFLOAT(_epsilon, optepsilon) );
        }
        return 0;   // success
    }
    return 1;
}

bool canRunAnotherARAstarIteration(MYFLOAT epsilonDash)
{
    if (epsilonDash.int_p > ARAstar_MIN_EPSILON)
        return true;
    else if (epsilonDash.int_p == ARAstar_MIN_EPSILON && epsilonDash.frac_p > 0)
        return true;
    else
        return false;
}

int improvePathCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, PUZZLENODE *goal, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlist, QHash<QString, PUZZLENODE*> *inconslist,
                   Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    QHash<QString, bool> closedlist;
    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalue = 0;

    QString hashKey = "";

#ifdef  CAUTION_NO
    while(openlist->at(0)->ptMe != goal->ptMe)
        //while(goal->f >= openlist->at(0)->f && openlist->at(0)->ptMe != goal->ptMe)
    {
        if (!(current = extractMinF(openlist, env)))   // take the first node from the OPEN
        {
            qDebug() << "FAILURE: openlist returned NULL";
            return 1;
        }
#else
    while(goal->fForward > openlist->at(0)->fForward)   // conflict
    {
        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "ARA* Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = goal->gForward;
        }

        if ( !(current = extractMinF(openlist, env)) )   // take the first node from the OPEN
        {
            qDebug() << "FAILURE in improvePATH(): openlist returned NULL";
            return 1;
        }
#endif

#ifdef MYASSERT
        if ( (previousFvalue > current->fForward) && epsilon->int_p == 1 && epsilon->frac_p == 0 )
        {
            qDebug() << "Assertion on non-decreasing fValue failed!";
            qDebug() << "expanded node number " << numberOfNodesExpandedForward;
            printArray(current->array);
            return 1;
        }
        previousFvalue = current->fForward;
#endif
        numberOfNodesExpandedForward++;     // no re-expansion condition checking as we are using closed list

        //expandlist.push_back(current->ptMe);  // temp
        hashKey = getHashKey(current);
        if ( !closedlist.value(hashKey, false) )
            closedlist.insert(hashKey, true);
        else
        {
            qDebug() << hashKey;
            qDebug() << "Error in closed list assignment in improvePath()";
            printArray(current->array);
            return 1;
        }

        QList<PUZZLENODE*> successors;
        findSuccessor( current, &successors, env );

        if (!successors.size())
            continue;
        else        // set ptParent for each child
        {
            while(successors.size())
            {
                child = successors.takeLast();
                if (!child->visitedForward)
                    child->gForward = MY_INFINITY;
                if ( child->gForward > current->gForward + PANCAKE_COST )       // cost to any child is 1 or sqrt(2)
                {
                    child->gForward = current->gForward + PANCAKE_COST;
                    child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                    child->parentForward = current;
                    child->visitedForward = true;
#ifdef MYASSERT
                    if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                    {
                        qDebug() << "Great error, child-current f-value comparision... Terminating...";
                        return 1;
                    }
#endif
                    hashKey = getHashKey(child);
                    if ( !closedlist.value(hashKey, false) )
                        minHeapInsertF(openlist, child, env);
                    else    // insert into INCONS LIST
                    {
                        // check to avoid multiple insertions
                        if ( !inconslist->value(hashKey, NULL) )
                            inconslist->insert(hashKey, child);
                    }
#ifdef MYASSERT_VALIDATE_HEAP
                    if (epsilon->int_p == 1 && epsilon->frac_p == 0 && numberOfNodesExpandedForward > 1000000)
                    {
                        validateHeap(openlist, current, &openlistcopy, &decreaseKeyNodeChain);     // heap debug [12-June-15]
                        qDebug() << numberOfNodesExpandedForward;   // heap debug [12-June-15]
                        decreaseKeyNodeChain.clear();
                        openlistcopy.clear();
                        for ( int i=0; i<openlist->size(); i++)
                            openlistcopy.append(openlist->at(i));
                    }
#endif
                }
            }
        }

    }   // end while
    //qDebug() << "iteration exit at " << (int)(t.elapsed()/TIME_INCREMENT_FACTOR) << "  bound = " << epsilon->float_p;       // testing
    //qDebug() << numberOfNodesExpandedForward;       // testing
    return 0;
}

int improvePatho1BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                              QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                              Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    //QList<QPoint> expandlist;   // temp
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;
    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    while( costIncumbentSolution > qMax(openlistForward->at(0)->fForward, openlistBackward->at(0)->fBackward) )
    {
        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }

        if ( numberOfNodesExpandedForward < numberOfNodesExpandedBackward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist forward returned NULL";
                return 1;
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
    }   // end while
    return 0;
}

int improvePatho12BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;

    // Fixing bug of repetition of values in openlist for optimization 2
    QList<QString> keyListF = inconslistForward->uniqueKeys();
    if ( inconslistForward->size() != keyListF.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListF.size(); i++)
        closedlistForward.insert(keyListF.at(i), true);
    QList<QString> keyListB = inconslistBackward->uniqueKeys();
    if ( inconslistBackward->size() != keyListB.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListB.size(); i++)
        closedlistBackward.insert(keyListB.at(i), true);

    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    while( costIncumbentSolution > qMax(openlistForward->at(0)->fForward, openlistBackward->at(0)->fBackward) )
    {
        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }

        if ( numberOfNodesExpandedForward < numberOfNodesExpandedBackward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist forward returned NULL";
                return 1;
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListForward = true;

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vForward = current->gForward;      // for second optimization

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vBackward = current->gBackward;      // for second optimization

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
    }   // end while
    return 0;
}

int improvePatho12BiARAstarNEW(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;

    // Fixing bug of repetition of values in openlist for optimization 2
    QList<QString> keyListF = inconslistForward->uniqueKeys();
    if ( inconslistForward->size() != keyListF.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListF.size(); i++)
        closedlistForward.insert(keyListF.at(i), true);
    QList<QString> keyListB = inconslistBackward->uniqueKeys();
    if ( inconslistBackward->size() != keyListB.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListB.size(); i++)
        closedlistBackward.insert(keyListB.at(i), true);

    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    bool moveForward = false, singleSearch = false;

    while( costIncumbentSolution > qMax(openlistForward->at(0)->fForward, openlistBackward->at(0)->fBackward) )
    {
        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }

        if ( moveForward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                if (openlistBackward->size() == 0)
                {
                    qDebug() << "FAILURE: openlist forward returned NULL";
                    return 1;
                }
                else        // Permanently move the backwards search
                {
                    singleSearch = true;
                    moveForward = false;
                    continue;
                }
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListForward = true;

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;
            moveForward = false;

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vForward = current->gForward;      // for second optimization

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( closedlistBackward.value(hashKey, false) )     // improvement 2
                            continue;

                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;
            if ( !singleSearch )
                moveForward = true;

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vBackward = current->gBackward;      // for second optimization

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
    }   // end while
    return 0;
}

int improvePatho123BiARAstarNEW(qint64 &HcorrectionForward, qint64 &minHerrNodeCountForward, MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                               QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                               Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;

    // Fixing bug of repetition of values in openlist for optimization 2
    QList<QString> keyListF = inconslistForward->uniqueKeys();
    if ( inconslistForward->size() != keyListF.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListF.size(); i++)
        closedlistForward.insert(keyListF.at(i), true);
    QList<QString> keyListB = inconslistBackward->uniqueKeys();
    if ( inconslistBackward->size() != keyListB.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListB.size(); i++)
        closedlistBackward.insert(keyListB.at(i), true);

    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    bool moveForward = false, singleSearch = false;

    while( costIncumbentSolution > qMax(openlistForward->at(0)->gForward + (qint64)(epsilon->float_p * (findHeuristic_F(openlistForward->at(0), env) + HcorrectionForward)),
                                        openlistBackward->at(0)->fBackward) )
    {
        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }

        if ( moveForward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                if (openlistBackward->size() == 0)
                {
                    qDebug() << "FAILURE: openlist forward returned NULL";
                    return 1;
                }
                else        // Permanently move the backwards search
                {
                    singleSearch = true;
                    moveForward = false;
                    continue;
                }
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListForward = true;

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;
            moveForward = false;

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vForward = current->gForward;      // for second optimization

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( closedlistBackward.value(hashKey, false) )     // improvement 2
                            continue;

                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;
            if ( !singleSearch )
                moveForward = true;

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vBackward = current->gBackward;      // for second optimization
            if ( current->gBackward - findHeuristic_F(current, env) == minHerrNodeCountForward )
                minHerrNodeCountForward--;

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }

            // check forward heuristic correction
            if ( minHerrNodeCountForward <= 0 && openlistBackward->size() )
            {
                qint64 tempmin = MY_INFINITY, buffer = MY_INFINITY;
                for (qint64 i=0; i<openlistBackward->size(); i++)
                {
                    buffer = openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i), env);
                    if ( buffer < tempmin )
                    {
                        tempmin = buffer;
                        minHerrNodeCountForward = 1;
                    }
                    else if ( buffer == tempmin )
                        minHerrNodeCountForward++;
                }
                HcorrectionForward = tempmin;
            }
        }
    }   // end while
    return 0;
}








int improvePatho123BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                                QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                                Environment *env, qint64 *HerrForward, qint64 *HerrBackward, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;

    // Fixing bug of repetition of values in openlist for optimization 2
    QList<QString> keyListF = inconslistForward->uniqueKeys();
    if ( inconslistForward->size() != keyListF.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListF.size(); i++)
        closedlistForward.insert(keyListF.at(i), true);
    QList<QString> keyListB = inconslistBackward->uniqueKeys();
    if ( inconslistBackward->size() != keyListB.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListB.size(); i++)
        closedlistBackward.insert(keyListB.at(i), true);

    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    while( costIncumbentSolution > qMax(openlistForward->at(0)->gForward + (qint64)(epsilon->float_p * (findHeuristic_F(openlistForward->at(0), env) +
                                                                                                        (*HerrForward))), openlistBackward->at(0)->gBackward + (qint64)(epsilon->float_p * (findHeuristic_B(openlistBackward->at(0), env) +
                                                                                                                                                                                            (*HerrBackward)))) )
    {
        if ( numberOfNodesExpandedForward < numberOfNodesExpandedBackward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist forward returned NULL";
                return 1;
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListForward = true;

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vForward = current->gForward;      // for second optimization

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            if (openlistBackward->size())
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistBackward->size(); i++)
                                {
                                    if ( openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i), env) < tempmin )
                                        tempmin = openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i),env);
                                }
                                (*HerrForward) = tempmin;
                            }
                            if (openlistForward->size())
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistForward->size(); i++)
                                {
                                    if ( openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i), env) < tempmin )
                                        tempmin = openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i),env);
                                }
                                (*HerrBackward) = tempmin;
                            }
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vBackward = current->gBackward;      // for second optimization

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            if (openlistBackward->size())
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistBackward->size(); i++)
                                {
                                    if ( openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i), env) < tempmin )
                                        tempmin = openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i),env);
                                }
                                (*HerrForward) = tempmin;
                            }
                            if (openlistForward->size())
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistForward->size(); i++)
                                {
                                    if ( openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i), env) < tempmin )
                                        tempmin = openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i),env);
                                }
                                (*HerrBackward) = tempmin;
                            }
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }
        }

        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }
    }   // end while
    return 0;
}

int improvePatho123BiARAstarFCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                                 QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                                 Environment *env, qint64 *HerrForward, qint64 *HerrBackward, PUZZLENODE **ptrMinHerrNodeForward,
                                 PUZZLENODE **ptrMinHerrNodeBackward, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;

    // Fixing bug of repetition of values in openlist for optimization 2
    QList<QString> keyListF = inconslistForward->uniqueKeys();
    if ( inconslistForward->size() != keyListF.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListF.size(); i++)
        closedlistForward.insert(keyListF.at(i), true);
    QList<QString> keyListB = inconslistBackward->uniqueKeys();
    if ( inconslistBackward->size() != keyListB.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListB.size(); i++)
        closedlistBackward.insert(keyListB.at(i), true);

    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    while( costIncumbentSolution > qMax(openlistForward->at(0)->gForward + (qint64)(epsilon->float_p * (findHeuristic_F(openlistForward->at(0), env) +
                                                                                                        (*HerrForward))), openlistBackward->at(0)->gBackward + (qint64)(epsilon->float_p * (findHeuristic_B(openlistBackward->at(0), env) +
                                                                                                                                                                                            (*HerrBackward)))) )
    {
        if ( numberOfNodesExpandedForward < numberOfNodesExpandedBackward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist forward returned NULL";
                return 1;
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListForward = true;

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vForward = current->gForward;      // for second optimization
            if (current == (*ptrMinHerrNodeForward))
                *ptrMinHerrNodeForward = NULL;

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            if (openlistBackward->size() && (*ptrMinHerrNodeForward) == NULL)
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistBackward->size(); i++)
                                {
                                    if ( openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i), env) < tempmin )
                                    {
                                        tempmin = openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i),env);
                                        *ptrMinHerrNodeForward = openlistBackward->at(i);
                                    }
                                }
                                (*HerrForward) = tempmin;
                            }
                            if (openlistForward->size() && (*ptrMinHerrNodeBackward) == NULL)
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistForward->size(); i++)
                                {
                                    if ( openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i), env) < tempmin )
                                    {
                                        tempmin = openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i),env);
                                        *ptrMinHerrNodeBackward = openlistForward->at(i);
                                    }
                                }
                                (*HerrBackward) = tempmin;
                            }
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vBackward = current->gBackward;      // for second optimization
            if (current == (*ptrMinHerrNodeBackward))
                *ptrMinHerrNodeBackward = NULL;

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            if (openlistBackward->size() && (*ptrMinHerrNodeForward) == NULL)
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistBackward->size(); i++)
                                {
                                    if ( openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i), env) < tempmin )
                                    {
                                        tempmin = openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i),env);
                                        *ptrMinHerrNodeForward = openlistBackward->at(i);
                                    }
                                }
                                (*HerrForward) = tempmin;
                            }
                            if (openlistForward->size() && (*ptrMinHerrNodeBackward) == NULL)
                            {
                                qint64 tempmin = MY_INFINITY;
                                for (qint64 i=0; i<openlistForward->size(); i++)
                                {
                                    if ( openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i), env) < tempmin )
                                    {
                                        tempmin = openlistForward->at(i)->gForward - findHeuristic_B(openlistForward->at(i),env);
                                        *ptrMinHerrNodeBackward = openlistForward->at(i);
                                    }
                                }
                                (*HerrBackward) = tempmin;
                            }
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }
        }

        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }
    }   // end while
    return 0;
}

int improvePathro12BiARAstarCCL(MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                                QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                                Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    qint64 numberOfNodesExpandedForward = 0;
    qint64 numberOfNodesExpandedBackward = 0;
    QHash<QString, bool> closedlistForward;
    QHash<QString, bool> closedlistBackward;

    // Fixing bug of repetition of values in openlist for optimization 2
    QList<QString> keyListF = inconslistForward->uniqueKeys();
    if ( inconslistForward->size() != keyListF.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListF.size(); i++)
        closedlistForward.insert(keyListF.at(i), true);
    QList<QString> keyListB = inconslistBackward->uniqueKeys();
    if ( inconslistBackward->size() != keyListB.size() )
    {
        qDebug() << "FAILURE: INCONS to CLOSED assignment";
        return 1;
    }
    for (qint64 i=0; i<keyListB.size(); i++)
        closedlistBackward.insert(keyListB.at(i), true);

    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;
    qint64 previousFvalueBackward = 0;

    QString hashKey = "";

    // ***********************************************SENSING***********************************************
#define SENSING_ARRAY_MAX_LIMIT 10
    qint64 avgHvalueForward = MY_INFINITY, avgHvalueBackward = MY_INFINITY;
    qint64 hValueArrayForward[SENSING_ARRAY_MAX_LIMIT] = {0}, hValueArrayBackward[SENSING_ARRAY_MAX_LIMIT] = {0};
    int indexCounterForward = 0, indexCounterBackward = 0;
    bool shortStuckForward = false, shortStuckBackward = false;
    qint64 longStuckForward = 0, longStuckBackward = 0;
    bool moveForward = true;
    // will measure newly expansions in stuck state, no less than 10% of the other side
    qint64 freshExpansionForward = 1, freshExpansionBackward = 1;
    bool randModeForward = false, randModeBackward = false;
    // ***********************************************END SENSING***********************************************

    while( costIncumbentSolution > qMax(openlistForward->at(0)->fForward, openlistBackward->at(0)->fBackward) )
    {
        // ****************************************SENSING****************************************
        if (  avgHvalueBackward<0 || avgHvalueForward<0 )
            qDebug() << "invalid exp";
        /* vesion 1
        if (!shortStuckForward)
            moveForward = true;
        else if (!shortStuckBackward)
            moveForward = false;
        else if (avgHvalueForward > avgHvalueBackward)
            moveForward = true;
        else
            moveForward = false;*/
        // version 2
        if (longStuckForward<longStuckBackward)
            moveForward = true;
        else
            moveForward = false;
        // ****************************************END SENSING****************************************

        //if ( numberOfNodesExpandedForward < numberOfNodesExpandedBackward )   // Expand Forward
        if ( moveForward )      // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist forward returned NULL";
                return 1;
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListForward = true;

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            numberOfNodesExpandedForward++;

            // ****************************************SENSING*******************************************
            if (indexCounterForward==SENSING_ARRAY_MAX_LIMIT)
            {
                double sumForward = 0;
                for (int i=0; i<indexCounterForward; i++)
                    sumForward += hValueArrayForward[i];
                qint64 newAvg = sumForward/indexCounterForward;
                longStuckForward += newAvg - avgHvalueForward;
                avgHvalueForward = newAvg;
                indexCounterForward = 0;
                if (longStuckForward<0)
                    longStuckForward = 0;
            }
            hValueArrayForward[indexCounterForward++] = findHeuristic_F(current, env);
            freshExpansionForward++;
            // ******************************************END SENSING*******************************************

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vForward = current->gForward;      // for second optimization

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;
#endif
            numberOfNodesExpandedBackward++;

            // ***********************************************SENSING**************************************************
            if (indexCounterBackward==SENSING_ARRAY_MAX_LIMIT)
            {
                double sumBackward = 0;
                for (int i=0; i<indexCounterBackward; i++)
                    sumBackward += hValueArrayBackward[i];
                qint64 newAvg = sumBackward/indexCounterBackward;
                longStuckBackward += newAvg - avgHvalueBackward;
                avgHvalueBackward = newAvg;
                indexCounterBackward = 0;
                if (longStuckBackward<0)
                    longStuckBackward = 0;
            }
            hValueArrayBackward[indexCounterBackward++] = findHeuristic_B(current, env);
            // ***********************************************END SENSING***********************************************

            hashKey = getHashKey(current);
            if ( !closedlistBackward.value(hashKey, false) )
                closedlistBackward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            current->vBackward = current->gBackward;      // for second optimization

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward + epsilon->float_p * findHeuristic_B(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( !closedlistBackward.value(hashKey, false) )
                            minHeapInsertB(openlistBackward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }
                    }
                }
            }
        }

        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }
    }   // end while
    return 0;
}

int improvePath_IKKA(QHash<QString, bool> *closedlistBackward, qint64 &HcorrectionForward, MYFLOAT *epsilonDash, bool *signalUnfinishedSearch, QTime &t, const MYFLOAT *epsilon, QList<PUZZLENODE *> *openlistForward, QHash<QString, PUZZLENODE*> *inconslistForward,
                              QList<PUZZLENODE *> *openlistBackward, QHash<QString, PUZZLENODE*> *inconslistBackward, qint64 &costIncumbentSolution,
                              Environment *env, ARASTARPRINTINFORMATION *printInfo)
{
    //QList<QPoint> expandlist;   // temp
    qint64 numberOfNodesExpandedForward = 0;
    QHash<QString, bool> closedlistForward;
    PUZZLENODE *current=NULL, *child=NULL;
    qint64 previousFvalueForward = 0;

    QString hashKey = "";

    float KKAddParameter = 0.1;
    bool moveForward = false, singleSearch = false;
    qint64 expLimitBackward = 10;
    qint64 tempExpansionCountForward = 0, tempExpansionCountBackward = 0;

    while( costIncumbentSolution > openlistForward->at(0)->gForward + (qint64)(epsilon->float_p * (findHeuristic_F(openlistForward->at(0), env)
                                                                                                   + HcorrectionForward)) )
    {
        qint64 timeElapsed = t.elapsed();
        if (timeElapsed >= MAX_POSSIBLE_TIME_MACRO)
        {
            if ( epsilon->int_p == ARAstar_MAX_EPSILON )    // it means that first ARA* iteration is still unfinished
            {
                (*signalUnfinishedSearch) = true;
                qDebug() << "bidir Max epoch reached, Solution doesn't exist for this config file! Exiting...";
            }
            return 0;
        }
        else
        {
            int timecount = (qint64)(timeElapsed/TIME_INCREMENT_FACTOR);
            printInfo[timecount].bound.setFloat_p( findMinMYFLOAT(*epsilon, *epsilonDash) );
            printInfo[timecount].solutionCost = costIncumbentSolution;
        }

        if ( moveForward )   // Expand Forward
        {
#ifdef  CAUTION
            if (!(current = extractMinF(openlistForward, env)))   // take the first node from the OPEN
            {
                if (openlistBackward->size() == 0)
                {
                    qDebug() << "FAILURE: openlist forward returned NULL";
                    return 1;
                }
                else        // Permanently move the backwards search
                {
                    singleSearch = true;
                    moveForward = false;
                    continue;
                }
            }
#else
            current = extractMinF(openlistForward, env);   // take the first node from the OPEN
#endif

#ifdef MYASSERT
            if ( previousFvalueForward > current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedForward;
                printArray(current->array);
                return 1;
            }
            previousFvalueForward = current->fForward;
#endif
            tempExpansionCountForward++;
            if (tempExpansionCountForward == (qint64)(expLimitBackward/KKAddParameter))
            {
                tempExpansionCountForward = 0;
                moveForward = false;
                expLimitBackward *= 2;
            }

            hashKey = getHashKey(current);
            if ( !closedlistForward.value(hashKey, false) )
                closedlistForward.insert(hashKey, true);
            else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list forward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }

            QList<PUZZLENODE*> successors;
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedForward)
                        child->gForward = MY_INFINITY;
                    if ( child->gForward > current->gForward + PANCAKE_COST )
                    {
                        child->gForward = current->gForward + PANCAKE_COST;
                        child->fForward = child->gForward + epsilon->float_p * findHeuristic_F(child, env);
                        child->parentForward = current;
                        child->visitedForward = true;

#ifdef MYASSERT
                        if ( child->fForward < current->fForward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListBackward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistForward.at(0)->fForward, openlistBackward.at(0)->fBackward);
                        }

                        hashKey = getHashKey(child);
                        if ( closedlistBackward->value(hashKey, false) )     // improvement 2
                            continue;

                        if ( !closedlistForward.value(hashKey, false) )
                            minHeapInsertF(openlistForward, child, env);
                        else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistForward->value(hashKey, NULL) )
                                inconslistForward->insert(hashKey, child);
                        }
                    }
                }
            }
        }
        else        // Expand Backward
        {
#ifdef  CAUTION
            if (!(current = extractMinB(openlistBackward, env)))   // take the first node from the OPEN
            {
                qDebug() << "FAILURE: openlist backward returned NULL";
                return 1;
            }
#else
            current = extractMinB(openlistBackward, env);   // take the first node from the OPEN
#endif
            //env->grid[current->ptMe.x()][current->ptMe.y()].onClosedListBackward = true;

#ifdef MYASSERT
            /*if ( previousFvalueBackward > current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
            {
                qDebug() << "Assertion on non-decreasing fValue failed!";
                qDebug() << "expanded node number " << numberOfNodesExpandedBackward;
                printArray(current->array);
                return 1;
            }
            previousFvalueBackward = current->fBackward;*/
#endif
            hashKey = getHashKey(current);
            if ( !closedlistBackward->value(hashKey, false) )
                closedlistBackward->insert(hashKey, true);
            /*else
            {
                qDebug() << hashKey;
                qDebug() << "Error in closed list Backward assignment in improvePatho1BiARAstar()";
                printArray(current->array);
                return 1;
            }*/

            QList<PUZZLENODE*> successors;     // vector of children of current
            findSuccessor( current, &successors, env );

            if (!successors.size())
                continue;
            else        // set ptParent for each child
            {
                while(successors.size())
                {
                    child = successors.takeLast();
                    if (!child->visitedBackward)
                        child->gBackward = MY_INFINITY;
                    if ( child->gBackward > current->gBackward + PANCAKE_COST )
                    {
                        child->gBackward = current->gBackward + PANCAKE_COST;
                        child->fBackward = child->gBackward - findHeuristic_F(child, env);
                        child->parentBackward = current;
                        child->visitedBackward = true;

#ifdef MYASSERT
                        /*if ( child->fBackward < current->fBackward && epsilon->int_p == 1 && epsilon->frac_p == 0 )
                        {
                            qDebug() << "Great error, child-current f-value comparision... Terminating...";
                            return 1;
                        }*/
#endif
                        if ( /*env->grid[child->ptMe.x()][child->ptMe.y()].onClosedListForward &&*/
                             costIncumbentSolution > child->gForward + child->gBackward )    // current already expanded by backward
                        {
                            costIncumbentSolution = child->gForward + child->gBackward;
                            //qDebug() << "cost" << costIncumbentSolution;
                            //qDebug() << "qmax" << qMax(openlistBackward.at(0)->fBackward, openlistBackward.at(0)->fBackward);
                        }

                        //hashKey = getHashKey(child);
                        //if ( !closedlistBackward.value(hashKey, false) )
                        minHeapInsertB(openlistBackward, child, env);
                        /*else    // insert into INCONS LIST
                        {
                            // check to avoid multiple insertions
                            if ( !inconslistBackward->value(hashKey, NULL) )
                                inconslistBackward->insert(hashKey, child);
                        }*/
                    }
                }
            }

            if (!singleSearch)
            {
                tempExpansionCountBackward++;
                if (tempExpansionCountBackward == expLimitBackward)
                {
                    tempExpansionCountBackward = 0;
                    moveForward = true;

                    if (openlistBackward->size())
                    {
                        qint64 tempmin = MY_INFINITY, buffer = MY_INFINITY;
                        for (qint64 i=0; i<openlistBackward->size(); i++)
                        {
                            buffer = openlistBackward->at(i)->gBackward - findHeuristic_F(openlistBackward->at(i), env);
                            if ( buffer < tempmin )
                                tempmin = buffer;
                        }
                        if (HcorrectionForward > tempmin)   // ASSERTION
                        {
                            qDebug() << "Error in heuristic correction calculation!! Decrease in Heuristic correction.";
                            return 1;
                        }
                        HcorrectionForward = tempmin;
                    }
                }
            }
        }
    }   // end while
    return 0;
}













double findMinMYFLOAT(MYFLOAT num1, MYFLOAT num2)
{
    if (num1.int_p > num2.int_p)
        return num2.float_p;
    else if ( num1.int_p == num2.int_p && num1.frac_p > num2.frac_p )
        return num2.float_p;
    else
        return num1.float_p;
}

int getRandomNumber(int low, int high)
{
    return qrand() % ((high + 1) - low) + low;
}

void generateConfigFiles()
{
    // Create seed for the random
    QTime time = QTime::currentTime();
    qsrand((uint)time.msec());

    int TOTAL_FILES = 100, PUZZLE_LENGTH = 70;      // PUZZLE_LENGTH should be >= 6
    int TOTAL_SWAPS = 1000;
    int firstSwapPos = 7;
    QString path = "/home/peace/Projects/Qt/searchalgos/pancakePuzzle/pancakePuzzleConsole/resources/" + QString::number(PUZZLE_LENGTH) + "puz/";

    for (int i=1; i<=TOTAL_FILES;)
    {
        QList<int> pickList;
        for (int j=0; j<PUZZLE_LENGTH; j++)
            pickList.append(j);

        // PUZZLE_LENGTH times swaps
        for (int j=0; j<TOTAL_SWAPS; j++)
        {
            if ( firstSwapPos >= PUZZLE_LENGTH/2 )      // Mutual Exclusion
                pickList.swap( getRandomNumber(0, PUZZLE_LENGTH/2 - 1), firstSwapPos );
            else
                pickList.swap(getRandomNumber(PUZZLE_LENGTH/2, PUZZLE_LENGTH-1), firstSwapPos);
            firstSwapPos = getRandomNumber(0, PUZZLE_LENGTH-1);
        }

        QList<int> permutationList;
        for (;pickList.size();)
            permutationList.append( pickList.takeAt(getRandomNumber(0, pickList.size()-1)) );
        firstSwapPos = permutationList.last();

        // Check heuristic value, must be equal to the PUZZLE_SIZE
        permutationList.append(PUZZLE_LENGTH);      // append plate

        int sum = 0;
        for (int j=0; j<PUZZLE_LENGTH; j++)
            if ( qAbs(permutationList.at(j) - permutationList.at(j+1)) > 1 )
                sum++;

        if ( sum == PUZZLE_LENGTH )
            permutationList.removeLast();
        else
            continue;

        // Write permutationList to the file
        QString fileName = "env" + QString::number(i);
        QFile file(path + fileName + ".cfg");
        if (!file.open(QIODevice::Append | QIODevice::Text))
        {
            qDebug() << "Error while opening the result file: from comapreARAstarBlockbasedSearchCCL";
            return;
        }
        QTextStream out(&file);

        out << "puzzleSize" << endl;
        out << QString::number(PUZZLE_LENGTH) << endl;
        out << "start" << endl;
        for (int j=0; j<permutationList.size()-1; j++)
            out << permutationList.at(j) << " ";
        out << permutationList.last() << endl;
        out << "end" << endl;
        for (int j=0; j<PUZZLE_LENGTH-1; j++)
            out << j << " ";
        out << PUZZLE_LENGTH-1 << endl << endl;

        file.close();
        qDebug() << "File  " << i << "  " << fileName << " written!";
        i++;
    }
}










int getMean(QList<int> tempValuesList)
{
    qint64 count = 0;
    qint64 sum=0;
    for (;tempValuesList.size();)
    {
        sum += tempValuesList.takeFirst();
        count++;
    }
    return (sum/count);
}

int getStdDev(qint64 mean, QList<int> tempValuesList)
{
    qint64 count = 0;
    qint64 sum=0;
    for (;tempValuesList.size();)
    {
        sum += qPow( (tempValuesList.takeFirst()-mean),2 );
        count++;
    }
    count--;
    return ( qSqrt(sum/count) );
}

void findMeanStdDev()
{
    QString resultPath = "/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/t24puzzle/reseultFiles/";
    bool ok;

    QList<QString> valueTypeList;   valueTypeList << "p" << "t" << "e";
    QList<QString> searchTypeList; searchTypeList << "a" << "b" << "c" << "d" << "e" << "f";
    QList<float> epsilonList;
    for (float i=ARAstar_MIN_EPSILON; i<=ARAstar_MAX_EPSILON; i += DECREMENT)
        epsilonList.append(i);

    for (int searchTypeIdx=0; searchTypeIdx<searchTypeList.size(); searchTypeIdx++)
    {
        for (int weightIdx=0; weightIdx<epsilonList.size(); weightIdx++)     // searchType
        {
            QList<int> intersectionList;
            QFile isecfile("/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/24puzzle/reseultFiles/intersectionDetails/w"
                           + QString::number(epsilonList.at(weightIdx)) + ".csv");
            if (!isecfile.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                qDebug() << "Error while opening the result file: for reading";
                return;
            }
            QTextStream isecin(&isecfile);
            QStringList spaceSplitted;
            QRegularExpression respace(" ");
            QString isecline;
            // read line
            for (int i=0; i<50; i++)    // for npuzzle
            {
                isecline = isecin.readLine();
                spaceSplitted = isecline.split(respace);
                int tempNum = spaceSplitted.last().toInt(&ok);
                if ( tempNum )
                    intersectionList.append(i);
                if (isecline == "")
                    qDebug() << "Error in input file, empty line at record " << (i+1);
            }
            isecfile.close();

            for (int valueTypeIdx=0; valueTypeIdx<valueTypeList.size(); valueTypeIdx++)   // path time expansion
            {
                QList<int> tempValuesList;
                int mean = -1, stddev = -1;
                bool ok;
                QString line = "";

                QString fileName = valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx) + QString::number(epsilonList.at(weightIdx));     // READ FILE

                QFile file(resultPath + fileName + ".csv");
                qDebug() << fileName + ".csv";

                if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: for reading";
                    return;
                }
                QTextStream in(&file);
                QStringList spaceSplitted;
                QRegularExpression respace(" ");
                // read line
                for (int i=0; i<51; i++)    // for npuzzle
                {
                    if (i < 50)
                    {
                        line = in.readLine();
                        spaceSplitted = line.split(respace);
                        if ( spaceSplitted.last() == "#######" )
                            qDebug() << "Error in input file, ####### line at record " << (i+1);
                        if (line == "")
                            qDebug() << "Error in input file, empty line at record " << (i+1);
                        int tempNum = spaceSplitted.last().toInt(&ok);
                        if (tempNum==MY_INFINITY)
                            qDebug() << "Error in input file, MY_INFINITY line at record " << (i+1);
                        if ( intersectionList.contains(i) )
                            tempValuesList.append(tempNum);
                    }
                    else
                    {
                        line = in.readLine();
                        if ( line != "")
                            qDebug() << "Error in input file, more than 50 lines";
                    }
                }
                file.close();

                if ( tempValuesList.size() != intersectionList.size() )
                    qDebug() << "Error tempValueList and intersectionList have different sizes";

                mean = getMean(tempValuesList);
                stddev = getStdDev(mean, tempValuesList);

                // WRITE MEAN
                file.setFileName(resultPath + "res" + "/mean/" + valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx) + ".mat");
                if ( !file.exists() )
                {
                    if (!file.open(QIODevice::Append | QIODevice::Text))
                    {
                        qDebug() << "Error while opening the result file: ";
                        return;
                    }
                    QTextStream out(&file);
                    line = "# Created by Octave 3.8.1, Fri Apr 03 02:26:59 2015 IST <peace@himanshulin>";
                    out << line << endl;
                    line = "# name: " + valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx);
                    out << line << endl;
                    line = "# type: matrix";
                    out << line << endl;
                    line = "# rows: 1";
                    out << line << endl;
                    line = "# columns: " + QString::number(epsilonList.size());
                    out << line << endl;
                    line = "";
                    out.flush();
                    file.close();
                }
                if (!file.open(QIODevice::Append | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: ";
                    return;
                }
                QTextStream out1(&file);
                line = " " + QString::number(mean);
                out1 << line;
                file.close();

                // WRITE STD DEV
                file.setFileName(resultPath + "res" + "/stddev/" + valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx) + ".mat");
                if ( !file.exists() )
                {
                    if (!file.open(QIODevice::Append | QIODevice::Text))
                    {
                        qDebug() << "Error while opening the result file: ";
                        return;
                    }
                    QTextStream out(&file);
                    line = "# Created by Octave 3.8.1, Fri Apr 03 02:26:59 2015 IST <peace@himanshulin>";
                    out << line << endl;
                    line = "# name: " + valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx);
                    out << line << endl;
                    line = "# type: matrix";
                    out << line << endl;
                    line = "# rows: 1";
                    out << line << endl;
                    line = "# columns: " + QString::number(epsilonList.size());
                    out << line << endl;
                    line = "";
                    out.flush();
                    file.close();
                }
                if (!file.open(QIODevice::Append | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: ";
                    return;
                }
                QTextStream out2(&file);
                line = " " + QString::number(stddev);
                out2 << line;
                file.close();
            }
        }
    }
    qDebug() << "All files written safely!";
}

void findMeanStdDevARAstar()
{
    QString resultPath = "/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/t24puzzle/reseultFiles/";
    bool ok;

    QList<QString> valueTypeList;   valueTypeList << "p" << "b";
    QList<QString> searchTypeList; searchTypeList << "a" << "b" << "c" << "d" << "e" << "f";

    for (int searchTypeIdx=0; searchTypeIdx<searchTypeList.size(); searchTypeIdx++)
    {
        for (int timecount = 0; timecount<((qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR)); timecount++)
        {
            QList<int> intersectionList;
            QFile isecfile("/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/24puzzle/reseultFiles/intersectionDetails/w.csv");
            if (!isecfile.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                qDebug() << "Error while opening the result file: for reading";
                return;
            }
            QTextStream isecin(&isecfile);
            QStringList spaceSplitted;
            QRegularExpression respace(" ");
            QString isecline;
            // read line
            for (int i=0; i<TOTAL_NUMBER_OF_CONFIG_FILES; i++)    // for npuzzle
            {
                isecline = isecin.readLine();
                spaceSplitted = isecline.split(respace);
                int tempNum = spaceSplitted.last().toInt(&ok);
                if ( tempNum )
                    intersectionList.append(i);
                if (isecline == "")
                    qDebug() << "Error in input file while reading intersection list, empty line at record " << (i+1);
            }
            isecfile.close();

            for (int valueTypeIdx=0; valueTypeIdx<valueTypeList.size(); valueTypeIdx++)   // path time expansion
            {
                QList<int> tempValuesList;
                int mean = -1, stddev = -1;
                bool ok;
                QString line = "";

                QString fileName = valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx) + QString::number((timecount+1)*TIME_INCREMENT_FACTOR);     // READ FILE

                QFile file(resultPath + fileName + ".csv");
                qDebug() << fileName + ".csv";

                if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: for reading";
                    return;
                }
                QTextStream in(&file);
                QStringList spaceSplitted;
                QRegularExpression respace(" ");
                // read line
                for (int i=0; i<=TOTAL_NUMBER_OF_CONFIG_FILES; i++)    // for npuzzle
                {
                    if (i < TOTAL_NUMBER_OF_CONFIG_FILES)
                    {
                        line = in.readLine();
                        spaceSplitted = line.split(respace);
                        int tempNum = spaceSplitted.last().toInt(&ok);
                        if ( intersectionList.contains(i) )
                        {
                            if ( spaceSplitted.last() == "#######" )
                            {
                                qDebug() << "Error in input file, ####### line at record " << (i+1);
                                return;
                            }
                            if (line == "")
                            {
                                qDebug() << "Error in input file, empty line at record " << (i+1);
                                return;
                            }
                            if (tempNum==MY_INFINITY)
                            {
                                qDebug() << "Major error in input file, MY_INFINITY line at record " << (i+1);
                                continue;
                            }
                            tempValuesList.append(tempNum);
                        }
                    }
                    else
                    {
                        line = in.readLine();
                        if ( line != "")
                            qDebug() << "Error in input file, more than 50 lines";
                    }
                }
                file.close();

                if ( tempValuesList.size() != intersectionList.size() )
                    qDebug() << "Error tempValueList and intersectionList have different sizes";

                mean = getMean(tempValuesList);
                stddev = getStdDev(mean, tempValuesList);

                // WRITE MEAN
                file.setFileName(resultPath + "res" + "/mean/" + valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx) + ".mat");
                if ( !file.exists() )
                {
                    if (!file.open(QIODevice::Append | QIODevice::Text))
                    {
                        qDebug() << "Error while opening the result file: ";
                        return;
                    }
                    QTextStream out(&file);
                    out << "# Created by Octave 3.8.1, Fri Apr 03 02:26:59 2015 IST <peace@himanshulin>" << endl;
                    out << "# name: " << valueTypeList.at(valueTypeIdx) << searchTypeList.at(searchTypeIdx) << endl;
                    out << "# type: matrix" << endl;
                    out << "# rows: 1" << endl;
                    out <<"# columns: " << QString::number((qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR)) << endl;
                    file.close();
                }
                if (!file.open(QIODevice::Append | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: ";
                    return;
                }
                QTextStream out1(&file);
                out1 << " " << QString::number(mean);
                file.close();

                // WRITE STD DEV
                file.setFileName(resultPath + "res" + "/stddev/" + valueTypeList.at(valueTypeIdx) + searchTypeList.at(searchTypeIdx) + ".mat");
                if ( !file.exists() )
                {
                    if (!file.open(QIODevice::Append | QIODevice::Text))
                    {
                        qDebug() << "Error while opening the result file: ";
                        return;
                    }
                    QTextStream out(&file);
                    out << "# Created by Octave 3.8.1, Fri Apr 03 02:26:59 2015 IST <peace@himanshulin>" << endl;
                    out << "# name: " << valueTypeList.at(valueTypeIdx) << searchTypeList.at(searchTypeIdx) << endl;
                    out << "# type: matrix" << endl;
                    out << "# rows: 1" << endl;
                    out << "# columns: " << QString::number((qint64)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR)) << endl;
                    file.close();
                }
                if (!file.open(QIODevice::Append | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: ";
                    return;
                }
                QTextStream out2(&file);
                out2 << " " << QString::number(stddev);
                file.close();
            }
        }
    }
    qDebug() << "All files written safely!";
}

void findWeightWiseIntersection()
{
    QString resultPath = "/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/t24puzzle/reseultFiles/";
    QString line = "";

    QList<QString> searchTypeList; searchTypeList << "a" << "b" << "c" << "d" << "e" << "f";

    //for (int timecount = 0; timecount<((int)((MAX_POSSIBLE_TIME_MACRO - TIME_INCREMENT_FACTOR) / TIME_INCREMENT_FACTOR)); timecount++)
    for (int timecount = 0; timecount<1; timecount++)
    {
        QList<bool> intersectionList;     // 0 - searchType a, 1 - b, 2 -c, 3 - d, 4 - e
        for (int i=0; i<TOTAL_NUMBER_OF_CONFIG_FILES; i++)
            intersectionList.append(true);      // true for an intersection across all 5 searchTypes
        for (int searchTypeIdx=0; searchTypeIdx<searchTypeList.size(); searchTypeIdx++)
        {
            int percentageCount = 0;
            QString fileName = "p" + searchTypeList.at(searchTypeIdx) + QString::number((timecount+1)*TIME_INCREMENT_FACTOR);     // READ FILE

            QFile file(resultPath + fileName + ".csv");
            qDebug() << fileName + ".csv";
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                qDebug() << "Error while opening the result file: for reading";
                return;
            }
            QTextStream in(&file);
            QStringList spaceSplitted;
            QRegularExpression respace(" ");
            // read line
            for (int i=0; i<TOTAL_NUMBER_OF_CONFIG_FILES; i++)    // for npuzzle
            {
                line = in.readLine();
                spaceSplitted = line.split(respace);
                if ( spaceSplitted.last() == "#######" )
                    intersectionList.replace(i, false);
                else
                    percentageCount++;
            }
            file.close();


            file.setFileName(resultPath + "intersectionDetails/" + "p" + searchTypeList.at(searchTypeIdx) + ".mat");
            if ( !file.exists() )
            {
                if (!file.open(QIODevice::Append | QIODevice::Text))
                {
                    qDebug() << "Error while opening the result file: ";
                    return;
                }
                QTextStream out(&file);
                out << "# Created by Octave 3.8.1, Fri Apr 03 02:26:59 2015 IST <peace@himanshulin>" << endl;
                out << "# name: " << "p" << searchTypeList.at(searchTypeIdx) << endl;
                out << "# type: matrix" << endl;
                out << "# rows: 1" << endl;
                out << "# columns: " << "1" << endl;
                file.close();
            }
            if (!file.open(QIODevice::Append | QIODevice::Text))
            {
                qDebug() << "Error while opening the result file: ";
                return;
            }
            QTextStream out(&file);
            out << " " << QString::number((qint64)(percentageCount * 100 / TOTAL_NUMBER_OF_CONFIG_FILES));
            file.close();
        }

        QFile file(resultPath + "intersectionDetails/" + "w" + ".csv");
        if (!file.open(QIODevice::Append | QIODevice::Text))
        {
            qDebug() << "Error while opening the result file: ";
            return;
        }
        QTextStream out(&file);
        for (int i=0; i<TOTAL_NUMBER_OF_CONFIG_FILES; i++)
        {
            if ( intersectionList.at(i) )
                out << " " << "1" << endl;
            else
                out << " " << "0" << endl;
        }
        out << endl << endl << " This file contains the weight wise intersection of all 5 search algorithms. 1 if each search algorithm has run for that instance otherwise 0.";
        file.close();
    }
    qDebug() << endl << endl << "All files written safely!";
}

void printOctaveQueries()
{
    QString valueType1 = "t"; // "p" "t" "e"
    QList<QString> searchTypeList; searchTypeList << "a" << "b" << "c" << "d" << "e" << "f";
    QList<QString> searchTypeColorList; searchTypeColorList << "r" << "g" << "y" << "b" << "k" << "c";
    QString xLabel = "weight",
            yLabel,
            title = ("npuzzle"),
            xlim = "[0, 12]",
            legend = "'Red-ARA*', 'Green-o1BiARA*', 'Yellow-o12BiARA*', 'Blue-o123BiARA*', 'Black-o123BiARA*F', 'Cyan-ro12BiARA*'";
    if (valueType1 == "p")
        yLabel = "path length";
    else if (valueType1 == "t")
        yLabel = "time taken";
    else if (valueType1 == "e")
        yLabel = "nodes expanded";

    QFile file("/home/peace/outputforoctave.txt");
    if (!file.open(QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "Error while opening the result file: ";
        return;
    }
    QTextStream out1(&file);

    out1 << "x = linspace (2, 10, 5)" << ";" << endl;

    for (int i=0; i<searchTypeList.size(); i++)
        out1 << "load " << valueType1 << searchTypeList.at(i) << ";" << endl;

    QString tempString = "";

    tempString += "figure; plot(";
    for (int i=0; i<searchTypeList.size(); i++)
    {
        tempString += "x," + valueType1 + searchTypeList.at(i) + ",'+-" + searchTypeColorList.at(i) + "',";
        //figure; plot(x,pa5,'*-g',x,pa10,'+-g',x,pa15,'o-g',x,pb5,'*-r',x,pb10,'+-r',x,pb15,'o-r');
    }
    out1 << tempString.mid(0, tempString.size()-1) << ")" << ";" << endl;

    out1 << "legend(" << legend << ")" << ";" << endl;
    out1 << "xlim(" << xlim << ")" << ";" << endl;
    out1 << "title('" << title << "')" << ";" << endl;
    out1 << "xlabel('" << xLabel << "')" << ";" << endl;
    out1 << "ylabel('" << yLabel << "')" << ";" << endl;
    out1 << "grid on;" << endl << endl << endl;
    file.close();

    qDebug() << "output written to octave file";
}

void copyFiles()    // copy from Korf benchmarks
{
    int PUZZ_SIZE = 4;
    for (int fileCount=1; fileCount<=50; fileCount++)
    {
        // we have not done rigrious exception handling, we leave onus of error reduction on user
        QFile file("/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/15puzzle/puz15_ori/" + QString::number(fileCount) + ".txt");
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qDebug() << "Error file opening the config file, Exiting...";
            return;
        }

        QTextStream in(&file);
        QStringList spaceSplitted;
        QRegularExpression REspace(" ");

        QString line = "";
        // Writing to a path file
        QFile file2("/home/peace/Documents/PdD/Planning/MYpaper/1/results/npuzzle/15puzzle/puz15/env" + QString::number(fileCount) + ".cfg");
        if (!file2.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qDebug() << "Error while opening the result file: ";
            return;
        }
        QTextStream out2(&file2);
        out2 << "puzzleSize" << endl;
        out2 << QString::number(PUZZ_SIZE) << endl;
        out2 << "start" << endl;


        line = in.readLine();   // puzzle start
        spaceSplitted = line.split(REspace);
        int counter = 0;
        for ( int i=0; counter!=spaceSplitted.size(); i++)      // removing blanks
        {
            if (spaceSplitted.at(counter).isEmpty())
                spaceSplitted.removeAt(counter);
            else
                counter++;
        }
        counter = 0;
        while (spaceSplitted.size()-1)
        {
            if ( counter < PUZZ_SIZE-1 )
            {
                line = spaceSplitted.takeFirst();
                if (line=="0")
                    line = "";
                out2 << line << " ";
                counter++;
            }
            else if ( counter < PUZZ_SIZE )
            {
                line = spaceSplitted.takeFirst();
                if (line=="0")
                    line = "";
                out2 << line;
                counter++;
            }
            else
            {
                counter = 0;
                out2 << endl;
            }
        }

        out2 << endl << "end" << endl;
        if ( PUZZ_SIZE == 4 )
        {
            out2 << " 1 2 3" << endl;
            out2 << "4 5 6 7" << endl;
            out2 << "8 9 10 11" << endl;
            out2 << "12 13 14 15" << endl << endl;
        }
        else if ( PUZZ_SIZE == 5 )
        {
            out2 << " 1 2 3 4" << endl;
            out2 << "5 6 7 8 9" << endl;
            out2 << "10 11 12 13 14" << endl;
            out2 << "15 16 17 18 19" << endl;
            out2 << "20 21 22 23 24" << endl << endl;
        }

        out2 << spaceSplitted.takeFirst();

        file2.close();
        file.close();
        qDebug() << "File " + QString::number(fileCount) + " written.";
    }
}












