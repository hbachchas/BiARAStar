
#include "structures.h"
#include "functions.h"

#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    if (argc<1)     // puzzleDim fileNo
    {
        QTextStream(stdout) << ">>> Not enough arguments. Please enter config Path followed by image Path" << endl;
        QTextStream(stdout) << ">>> Terminating... from main" << endl;
        return 0;
    }
    else
        callMeFromCMD();
    //return a.exec();
}








