## BiARAstar
to compile any of the projects:

1. extract gcc_64.tar.bz2 in a folder

2. go to the project directory and delete any existing .user files, if any

3. ../gcc_64/bin/qmake -o Makefile project.pro

4. make

This will generate the executable.




####About the Project.

1. on_btnSearchARAstar_clicked() is the entry function

2. there are five algorithms

 1. `ARA*` - basic ARA*
 2. `o1BiARA*` - basic bidirectional ARA*
 3. `o12BiARA*` - bidirectional ARA* with improvement: pruning when merging INCONS with OPEN
 4. `o12BiARA*NEW` - bidirectional ARA* with previous improvement + improvement 2
 5. `o123BiARA*NEW` - bidirectional ARA* with previous improvements + Heuristic correction

3. Please provide three command line arguments:
 1. configuration file path
 2. search type: a number between 1-5
 3. result path: path to store the results

4. in `structures.h` edit MAX_POSSIBLE_TIME_MACRO to set the time duration for execution, default is 20 seconds

5. Following two files are generated in results folder for every second:
 1. pa10.csv - 'p' stands for solution cost, for search type 'a' or 1, at the 10th second
 2. ba10.csv - 'b' stands for bound, for search type 'a' or 1, at the 10th second

