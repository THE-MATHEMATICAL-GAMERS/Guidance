function thv{
	set accl to ship:partstagged("rd180")[0]:maxthrust/ship:mass.
	if accl<=0 return 1.
	else{
		set val to (44.145/accl-0.47)/0.53.
		if val>1 return 1.
		else return val.
	}
}
wait until throttle = 1.
wait 2.
lock throttle to 1.
stage.
wait until stage:ready and ship:partstagged("rd180")[0]:thrust>=0.9*ship:maxthrust.
stage.
lock mach to 10*sqrt(ship:q/ship:sensors:pres/0.7).
wait until mach>=0.8.
print 0.5+(1-mach)*2.5.
lock throttle to 0.5+(1-mach)*2.5.
set oldq to ship:q.
until ship:q<oldq{
	set oldq to ship:q.
	if throttle<=0.5
	lock throttle to 0.5.
}
set oldq1 to mach.
set thrt to throttle.
lock throttle to thrt+(1-thrt)*(mach-oldq1)/(1.9-oldq1).
wait until mach>=1.9.
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
unlock throttle.
stage.
rcs on.
set ship:control:fore to 1.
wait 2.5.
stage.
wait until ship:partstagged("rl10")[0]:thrust>=0.9*ship:partstagged("rl10")[0]:maxthrust.
set ship:control:fore to 0.
set ship:control:neutralize to true.
// wait until ship:partstagged("rl10")[0]:thrust <= 0.
// set orbH to 300000.
// lock vx to sqrt(ship:velocity:orbit:mag*ship:velocity:orbit:mag-ship:verticalspeed*ship:verticalspeed).
// set vT to sqrt(398600441800000/(6371000+orbH)).
// lock tth to arctan(-ship:verticalspeed/(vT-vx)).
// lock steering to heading(arccos(-0.47882041*sin(ship:geoposition:lng+80.599756418184)),tth).
// set ship:control:fore to 1.
// wait until (ship:periapsis+ship:apoapsis)>=2*orbH.
// set ship:control:fore to 0.