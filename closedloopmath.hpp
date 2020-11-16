#pragma once

extern double hT;
extern double vx,vy,H,Mass,fuelinStage,F,I,M2,F2,I2;
extern int stage;
extern double A,B,T;

extern double vT,om,g;

double integrate(double (*f)(double),unsigned int n);
void simultaneous(double a[9],double b[3],double *X);
extern double t1,t2;
void t12();
double acc(double t);
double theta(double t);
double tanA(double t);
double tanB(double t);
double ax(double t);
double ay(double t);
double f3(double t);
double axB(double t);
double ayB(double t);
double f3B(double t);
double axA(double t);
double ayA(double t);
double f3A(double t);

void calculate();