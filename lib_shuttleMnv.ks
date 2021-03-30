//lib_shuttle_mnv.ks
@lazyGlobal off.

function shuttleSteerDir {
    parameter steerDir.

    local thrustAng is round(vang(ship:facing:forevector,centerOfThrust()),2).
    local rotationAxis is vcrs(ship:facing,centerOfThrust).

    local newDir is angleAxis(thrustAng,rotationAxis)*steerDir.

    return newDir.
}

function centerOfThrust {
    local eList is list().
    local avgPos is V(0,0,0).
    local maxT is 0.
	list engines in eList.
	
	for eng in eList {
		set avgPos to avgPos + -eng:POSITION * eng:MAXTHRUST.
		set maxT to maxT + eng:MAXTHRUST.
	}
	SET avgPos to avgPos / maxT.
	
return avgPos.
}

function shuttle_Node {
    //Uses the Shuttle Steer function to execute a maneuver node.
    parameter n is nextnode.

    local dv0 is n:deltav.

    local maxAcc is ship:maxThrust/ship:mass.

    local burnDuration is dv0:mag/maxAcc.

    local done is false.

    lock throttle to 0.

    clearscreen.

    until n:eta < (burnDuration/2)+300 {
        print "Burn ETA: " + round(n:eta - burnDuration/2,2) at(0,0).
        wait 0.
    }

    rcs on.
    sas off.
    lock steering to shuttleSteerDir(lookdirup(dv0:vec,-up:forevector)).

    until n:eta < (burnDuration/2) {
        print "Burn ETA: " + round(n:eta - burnDuration/2,2) at(0,0).
        wait 0.
    }

    rcs off.
    lock throttle to 1.
    until done {
        if vdot(dv0, n:deltav) < 0 {set done to true.}
        if n:deltav:mag < 0.1 {set done to true.}
    }
    lock throttle to 0.
    lock steering to shuttleSteerDir(lookdirup(prograde:forevector,-up:forevector)).
    rcs on.
}
