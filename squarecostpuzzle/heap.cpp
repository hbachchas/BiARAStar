
#include "structures.h"
#include "heap.h"

#define LEFT(i)     (2*i+1)
#define RIGHT(i)    (2*i+2)
#define PARENT(i)   (qFloor((i-1)/2))

using namespace searchEnv;

//@function     testing function
void testor(void)
{

    qDebug() << LEFT(2);
    qDebug() << RIGHT(2);
    qDebug() << PARENT(5);
    qDebug() << PARENT(6);

    QList<QPoint> openlist;
    openlist.push_back(QPoint(1,0));
    openlist.push_back(QPoint(0,1));
    openlist.push_back(QPoint(2,1));
    openlist.push_back(QPoint(2,3));
    openlist.push_back(QPoint(1,1));

    for (qint64 i =0; i< openlist.size(); i++)
        qDebug() << openlist.at(i);

    for (qint64 i=0; i< openlist.size(); i++)
    {
        for(qint64 j=i+1; j< openlist.size(); j++)
        {
            if (openlist.at(i).x()>openlist.at(j).x())
                openlist.swap(i,j);
            else if ((openlist.at(i).x()==openlist.at(j).x()) && openlist.at(i).y()>openlist.at(j).y())
                openlist.swap(i,j);

        }
    }
    qDebug() << "-----------------";
    for (qint64 i =0; i< openlist.size(); i++)
        qDebug() << openlist.at(i);

}

void writeOutput2File(QPoint *pt)
{
    QString configFilePath = "/home/peace/qtConsole.out";
    QFile file(configFilePath);
    if (!file.open(QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "Error file opening the config file, Exiting... from composeSearchSpaceFile";
        return;
    }

    QTextStream out(&file);
    QString line = "";

    // line1
    line = "FH: (" + QString::number(pt->x()) + ", " + QString::number(pt->y()) + ")";
    out << line << endl;

    out.flush();
    file.close();
}

//@function     insert current in openlist heap     2*O(lg n)
void minHeapInsertF(QList<PUZZLENODE*> *openlist, PUZZLENODE* current, Environment *env)
{
    qint64 pos = current->heapIndexForward;
    if (pos > -1)       // if node already exist
        heapDecreaseKeyF(openlist, pos, env);     // O(lg n)
    else
    {
        openlist->append(current);      // O(lg n)
        current->heapIndexForward = openlist->size()-1;
        heapDecreaseKeyF(openlist, openlist->size()-1, env);      // O(lg n)
    }
}

//@function     decrease key of an element at position pos      O(lg n)
void heapDecreaseKeyF(QList<PUZZLENODE*> *openlist, qint64 pos, Environment *env)
{
    qint64 key = openlist->at(pos)->fForward;
    //env->grid[openlist->at(pos)->ptMe.x()][openlist->at(pos)->ptMe.y()].heapIndex = pos;
    for(; pos > 0 && openlist->at(PARENT(pos))->fForward > key; pos = PARENT(pos))
    {
        openlist->swap(pos, PARENT(pos));
        openlist->at(PARENT(pos))->heapIndexForward = PARENT(pos);
        openlist->at(pos)->heapIndexForward = pos;
        //env->grid[openlist->at(PARENT(pos))->ptMe.x()][openlist->at(PARENT(pos))->ptMe.y()].heapIndexForward = PARENT(pos);
        //env->grid[openlist->at(pos)->ptMe.x()][openlist->at(pos)->ptMe.y()].heapIndexForward = pos;
    }
}

//@function     build min heap      O(n)
void buildMinHeapF(QList<PUZZLENODE*> *openlist, Environment *env)
{
    if (openlist->size())
    {
        // let everyone know its index
        for (qint64 i=0; i<openlist->size(); i++)
            openlist->at(i)->heapIndexForward = i;

        for(qint64 i = PARENT(openlist->size()-1); i>=0; --i)
            minHeapifyF(openlist, i, env);
    }
}

//@function     extract minimum     O(lg n)
PUZZLENODE* extractMinF(QList<PUZZLENODE*> *openlist, Environment *env)
{
    if (!openlist->size())
        return NULL;
    else
    {
        // not adjusting heapIndex of the node extracted, because it will not be used again in case of ASTAr
        openlist->swap(0, openlist->size()-1);      // swap first with last
        //env->grid[openlist->at(0)->ptMe.x()][openlist->at(0)->ptMe.y()].heapIndexForward = 0;
        openlist->at(0)->heapIndexForward = 0;
        PUZZLENODE *temp = openlist->takeLast();

        // minHeapify at position 0
        minHeapifyF(openlist, 0, env);
        temp->heapIndexForward = -1;        // needed in case of ARASTAR
#ifdef DEBUG_HEAP
        writeOutput2File(&(temp->ptMe));
#endif
        return temp;
    }
}

//@function     min Heapify openlist    O(lg n)
void minHeapifyF(QList<PUZZLENODE*> *openlist, qint64 i, Environment *env)
{
    qint64 smallest = -1;
    while(1)
    {
        if (LEFT(i) < openlist->size() && openlist->at(LEFT(i))->fForward < openlist->at(i)->fForward)
            smallest = LEFT(i);
        else
            smallest = i;
        if (RIGHT(i) < openlist->size() && openlist->at(RIGHT(i))->fForward < openlist->at(smallest)->fForward)
            smallest = RIGHT(i);
        if (smallest != i)
        {
            openlist->swap(i, smallest);
            //env->grid[openlist->at(smallest)->ptMe.x()][openlist->at(smallest)->ptMe.y()].heapIndexForward = smallest;
            openlist->at(smallest)->heapIndexForward = smallest;
            //env->grid[openlist->at(i)->ptMe.x()][openlist->at(i)->ptMe.y()].heapIndexForward = i;
            openlist->at(i)->heapIndexForward = i;
            i = smallest;
        }
        else
            break;
    }
}









//@function     insert current in openlist heap     2*O(lg n)
void minHeapInsertB(QList<PUZZLENODE*> *openlist, PUZZLENODE* current, Environment *env)
{
    qint64 pos = current->heapIndexBackward;
    if (pos > -1)       // if node already exist
        heapDecreaseKeyB(openlist, pos, env);     // O(lg n)
    else
    {
        openlist->append(current);      // O(lg n)
        current->heapIndexBackward = openlist->size()-1;
        heapDecreaseKeyB(openlist, openlist->size()-1, env);      // O(lg n)
    }
}

//@function     decrease key of an element at position pos      O(lg n)
void heapDecreaseKeyB(QList<PUZZLENODE*> *openlist, qint64 pos, Environment *env)
{
    qint64 key = openlist->at(pos)->fBackward;
    //env->grid[openlist->at(pos)->ptMe.x()][openlist->at(pos)->ptMe.y()].heapIndex = pos;
    for(; pos > 0 && openlist->at(PARENT(pos))->fBackward > key; pos = PARENT(pos))
    {
        openlist->swap(pos, PARENT(pos));
        openlist->at(PARENT(pos))->heapIndexBackward = PARENT(pos);
        openlist->at(pos)->heapIndexBackward = pos;
        //env->grid[openlist->at(PARENT(pos))->ptMe.x()][openlist->at(PARENT(pos))->ptMe.y()].heapIndexForward = PARENT(pos);
        //env->grid[openlist->at(pos)->ptMe.x()][openlist->at(pos)->ptMe.y()].heapIndexForward = pos;
    }
}

//@function     build min heap      O(n)
void buildMinHeapB(QList<PUZZLENODE*> *openlist, Environment *env)
{
    if (openlist->size())
    {
        // let everyone know its index
        for (qint64 i=0; i<openlist->size(); i++)
            openlist->at(i)->heapIndexBackward = i;

        for(qint64 i = PARENT(openlist->size()-1); i>=0; --i)
            minHeapifyB(openlist, i, env);
    }
}

//@function     extract minimum     O(lg n)
PUZZLENODE* extractMinB(QList<PUZZLENODE*> *openlist, Environment *env)
{
    if (!openlist->size())
        return NULL;
    else
    {
        // not adjusting heapIndex of the node extracted, because it will not be used again in case of ASTAr
        openlist->swap(0, openlist->size()-1);      // swap first with last
        //env->grid[openlist->at(0)->ptMe.x()][openlist->at(0)->ptMe.y()].heapIndexForward = 0;
        openlist->at(0)->heapIndexBackward = 0;
        PUZZLENODE *temp = openlist->takeLast();

        // minHeapify at position 0
        minHeapifyB(openlist, 0, env);
        //env->grid[temp->ptMe.x()][temp->ptMe.y()].heapIndexForward = -1;   // needed in case of ARASTAR
        temp->heapIndexBackward = -1;
#ifdef DEBUG_HEAP
        writeOutput2File(&(temp->ptMe));
#endif
        return temp;
    }
}

//@function     min Heapify openlist    O(lg n)
void minHeapifyB(QList<PUZZLENODE*> *openlist, qint64 i, Environment *env)
{
    qint64 smallest = -1;
    while(1)
    {
        if (LEFT(i) < openlist->size() && openlist->at(LEFT(i))->fBackward < openlist->at(i)->fBackward)
            smallest = LEFT(i);
        else
            smallest = i;
        if (RIGHT(i) < openlist->size() && openlist->at(RIGHT(i))->fBackward < openlist->at(smallest)->fBackward)
            smallest = RIGHT(i);
        if (smallest != i)
        {
            openlist->swap(i, smallest);
            //env->grid[openlist->at(smallest)->ptMe.x()][openlist->at(smallest)->ptMe.y()].heapIndexForward = smallest;
            openlist->at(smallest)->heapIndexBackward = smallest;
            //env->grid[openlist->at(i)->ptMe.x()][openlist->at(i)->ptMe.y()].heapIndexForward = i;
            openlist->at(i)->heapIndexBackward = i;
            i = smallest;
        }
        else
            break;
    }
}




