#include "closedloopmath.hpp"
#include <cmath>
using namespace std;

//(Asin(w*t)+Bcos(w*t))/(Acos(w*t)-B*sin(w*t)+1)

double hT;
double vx,vy,H,Mass,fuelinStage,F,I,M2,F2,I2;
int stage;//stage=0  rd 180 a<44.145   stage=1 rd180 acc=44.145      stage=2 rl10

double vT,A,B,T,om,g;

//atan(A+t*(B+om*A*A)+t*t*om*A*(3*B/2+om*A*A))          use as your equation

double integrate(double (*f)(double),unsigned int n){//The numerical Integral Solver. usage:integral(function,intervals)..........The function is verified to work correct
	double sum=0,dt=T/n;
	for(double t=0;t<T;t+=dt)sum+=(*f)(t)*dt;
	sum+=sum+((*f)(T)-(*f)(0))*dt;
	sum/=2;
	return sum;
}
void simultaneous(double a[9],double b[3],double *X){//Verified to work correct
	double den=*(a+0)*(*(a+4)**(a+8)-*(a+5)**(a+7))-*(a+1)*(*(a+3)**(a+8)-*(a+5)**(a+6))+*(a+2)*(*(a+3)**(a+7)-*(a+4)**(a+6));
	*X=(*(b+0)*(*(a+4)**(a+8)-*(a+5)**(a+7))-*(a+1)*(*(b+1)**(a+8)-*(a+5)**(b+2))+*(a+2)*(*(b+1)**(a+7)-*(a+4)**(b+2)))/den;
	*(X+1)=(*(a+0)*(*(b+1)**(a+8)-*(a+5)**(b+2))-*(b+0)*(*(a+3)**(a+8)-*(a+5)**(a+6))+*(a+2)*(*(a+3)**(b+2)-*(b+1)**(a+6)))/den;
	*(X+2)=(*(a+0)*(*(a+4)**(b+2)-*(b+1)**(a+7))-*(a+1)*(*(a+3)**(b+2)-*(b+1)**(a+6))+*(b+0)*(*(a+3)**(a+7)-*(a+4)**(a+6)))/den;
}
double t1,t2;//t1= time of stage1, t2=time of second stage-2.5 sec. the 2.5 sec is time for the second stage engine to start.
void t12(){//calculate t1,t2......Tested
	if(stage==2){t1=t2=-500;}
	else if(stage==1){t1=-500;t2=I*9.80665*log(Mass/(Mass-fuelinStage))/44.145;}
	else{
		t1=I*9.80665*(Mass/F-1/44.145);
		t2=t1+I*9.80665*log(Mass/(Mass-fuelinStage+F*t1/I/9.80665))/44.145;
	}
}
double acc(double t){//The Accleration Of The Ship......Tested
	if(t<t1 || stage==2) return F/(Mass-F*t/I/9.80665);
	else if(t<t2)return 44.145;
	else if(t>(t2+2.5))return F2/(M2-F2*(t-t2-2.5)/I2/9.80665);
	else return 0;
}



double theta(double t){
	return atan(A+t*(B+om*A*A)+t*t*om*A*(3*B/2+om*A*A));
}
double tanA(double t){
	return 1+2*A*om*t+3*om*(B/2+om*A*A)*t*t;
}
double tanB(double t){
	return t+3*om*A*t*t/2;
}

double ax(double t){
	return acc(t)*cos(theta(t))-vx*vy/(6371000+H);
}
double ay(double t){
	return acc(t)*sin(theta(t))+0.2*vT*vT/(6371000+hT)+0.8*vx*vx/(6371000+H)-g;
}
double f3(double t){
	return -t*ay(t);
}
double axB(double t){
	double sth=sin(theta(t));
	return acc(t)*(sth*sth*sth-sth)*tanB(t);
}
double ayB(double t){
	double cth=cos(theta(t));
	return acc(t)*cth*cth*cth*tanB(t);
}
double f3B(double t){
	return -t*ayB(t);
}
double axA(double t){
	double sth=sin(theta(t));
	return acc(t)*(sth*sth*sth-sth)*tanA(t);
}
double ayA(double t){
	double cth=cos(theta(t));
	return acc(t)*cth*cth*cth*tanA(t);
}
double f3A(double t){
	return -t*ayA(t);
}
void calculate(){
	vT=sqrt(398600441800000/(6371000+hT));
	om=vx/(6371000+H)+vT/(6371000+hT);
	g=398600441800000/(6371000+H)/(6371000+H);
	t12();
	double coeff[9],dV[3]={1000,1000,1000},dels[3],chvx=vT-vx,chvy=-vy,chh=hT-H;
	while(abs(dV[0]/chvx)>0.001 || abs(dV[1]/chvy)>0.001 || abs(dV[2]/chh)>0.001){
		dV[0]=chvx-integrate(ax,100000);dV[1]=chvy-integrate(ay,100000);dV[2]=chh-integrate(f3,100000);
		coeff[0]=ax(T);coeff[1]=integrate(axA,100000);coeff[2]=integrate(axB,100000);
		coeff[3]=ay(T);coeff[4]=integrate(ayA,100000);coeff[5]=integrate(ayB,100000);
		coeff[6]=f3(T);coeff[7]=integrate(f3A,100000);coeff[8]=integrate(f3B,100000);
		simultaneous(coeff,dV,dels);
		T+=dels[0];A+=dels[1];B+=dels[2];
	}
}

// int main(){
// 	cout<<"here\n";
// 	stage=0;
// 	hT=300000;vx=780;vy=920;H=42000;Mass=249000;fuelinStage=199000;F=4152000;I=337.8;M2=23146;F2=198200;I2=450.5;
// 	//vx=5100;vy=100;H=250000;Mass=23146;fuelinStage=20000;F=198200;I=450.5;
// 	//vx=3500;vy=500;H=150000;fuelinStage=44053;Mass=94053;
// 	A=-0.30;B=0.15;T=500;
// 	calculate();
// 	cout<<A<<","<<B<<","<<T<<","<<om;
// }