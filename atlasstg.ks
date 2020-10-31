function thv{
	set accl to ship:partstagged("rd180")[0]:maxthrust/ship:mass.
	if accl<=0 return 1.
	else{
		set val to (44.145/accl-0.47)/0.53.
		if val>1 return 1.
		else return val.
	}
}
//sas on.
wait until throttle = 1.
wait 3.
stage.
wait until stage:ready and ship:partstagged("rd180")[0]:thrust>=0.9*ship:maxthrust.
stage.
lock mach to 10*sqrt(ship:q/ship:sensors:pres/0.7).
wait until mach>=0.8.
lock throttle to 0.5+(1-mach)*2.5.
set oldq to ship:q.
until ship:q<oldq{
	set oldq to ship:q.
	if throttle<=0.5
	lock throttle to 0.5.
}
set oldq1 to mach.
set thrt to throttle.
lock throttle to thrt+(1-thrt)*(mach-oldq1)/(1.6-oldq1).
wait until mach>=1.6.
lock throttle to 1.
wait until ship:partstagged("srb")[0]:resources[0]:amount<=0.
stage.
set flgt to time:seconds+10000.
when ship:q<=0.00005*oldq then{
	stage.
	set flgt to time:seconds+5.
}
when time:seconds>=flgt then stage.
lock throttle to thv().
until ship:partstagged("rd180")[0]:thrust<=0.
lock throttle to 1.
stage.
rcs on.
set ship:control:fore to 1.
wait 2.
stage.
wait until ship:partstagged("rl10")[0]:thrust>=0.9*ship:partstagged("rl10")[0]:maxthrust.
set ship:control:fore to 0.
set ship:control:neutralize to true.
wait until 0.