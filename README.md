#  Modified Linear Tangent Guidance

## Project demo

You can find the project demo [here.](https://youtu.be/K8VO1M5nfRU) Also to know more see the PDF in the repository.

## Test it on KSP using KRPC C++ and KOS

Put the craft file at its place and the kos program in the boot folder(depends on your version). In ksp add the kos boot program in the centaur of the Atlas V 542 and go for the launch.

For the C++ program move in to the directory where the rest of the files are located and open up a terminal and type the following.
```
g++ -o guidance openclose.cpp -std=c++11 -lkrpc -lprotobuf closedloopmath.cpp
```
And then to run it...
```
./guidance <krpc server ip> <height of orbit in meters> <time to orbit initial guess>
```
