//lib_shuttle_deorbit.ks
@lazyGlobal off.

function deorbit_maneuver {
    //Calculate a deorbit maneuver given a target landing site [geocoordinates], an earliest burn time [s], a desired entry flight path angle [deg], 
    //a desired entry interface downrange distance [km], and a maximum crossrange distance [km]
    parameter site.
    parameter t_earliest_burn is 600.   //600 seconds from now
    parameter fpa is -3.                //-3 degrees (Not used right now)
    parameter EI_dist is 1500000.       //1500km (Not used right now)
    parameter xrange_max is 100000.     //100km

    set config:ipu to 2000.
    clearvecdraws().
    local site is location_constants:kerbin:runway_09_start.
    local vec_site is V(0,0,0).
    local vec_proj is V(0,0,0).
    local vec_ship is V(0,0,0).
    local vec_eitgt is angleaxis(EI_dist/(constant:pi*2*ship:body:radius)*360,-orbitBinormal(SHIP))*vec_proj:normalized*(ship:body:radius+ship:body:atm:height).
    local central_ang is 0.
    local normal_ang is 0.
    //local tgt_ang is 190.
    local tgt_ang is 200.   //FAR
    //local tgt_ang is 182. //Stock Atmosphere
    local xrange is ship:body:radius*2*constant:pi.
    local dxrange is abs(xrange - xrange_max).
    local dang is abs(central_ang-tgt_ang).
    local vec_site_draw is vecdraw(kerbin:position,vec_site-kerbin:position,white,"LandingSite",1.5,TRUE).
    local vec_proj_draw is vecdraw(kerbin:position,vec_proj-kerbin:position,yellow,"Projection",1.5,TRUE).
    local vec_ship_draw is vecdraw(kerbin:position,vec_ship-kerbin:position,blue,"Ship",1.0,TRUE).
    local vec_eitgt_draw is vecdraw(kerbin:position,vec_eitgt,red,"Entry Interface",1.0,TRUE).
    clearscreen.
    local rm is 1.
    //local t_adv is (ship:body:rotationperiod / 4) + time:seconds. //One quarter of a day ahead
    local t_adv is time:seconds + t_earliest_burn.
    local delta_tof is central_ang/360 * (ship:orbit:period).

    //Propogate forward until landing site is within crossrange distance
    until rm = 0 {
        if rm = 1{
            set delta_tof to central_ang/360 * (ship:orbit:period).
    	    set vec_site to ground_positionAt(site,t_adv + delta_tof)-kerbin:position.
    	    set vec_proj to project_vec_to_orbit(vec_site).
    	    set vec_ship to positionAt(ship,t_adv)-kerbin:position.
            set vec_eitgt to angleaxis(EI_dist/(constant:pi*2*ship:body:radius)*360,-orbitBinormal(ship))*vec_proj:normalized*(ship:body:radius+ship:body:atm:height).
    	    set vec_site_draw:start to kerbin:position.
    	    set vec_site_draw:vec to vec_site.
    	    set vec_proj_draw:start to kerbin:position.
    	    set vec_proj_draw:vec to vec_proj.
    	    set vec_ship_draw:start to kerbin:position.
    	    set vec_ship_draw:vec to vec_ship.
            set vec_eitgt_draw:start to kerbin:position.
            set vec_eitgt_draw:vec to vec_eitgt.

    	    set central_ang to vang_360(vec_ship,vec_proj,orbitBinormal(ship)).
    	    set normal_ang to vang(vec_proj,vec_site).
    	    set xrange to 2*ship:body:radius*(normal_ang*constant:pi/360).
    	    print "Central Angle: " + round(central_ang,2) + " deg    " at(0,0).
    	    print "Normal Angle: " + round(normal_ang,2) + " deg    " at(0,1).
    	    print "Crossrange Distance: " + round(xrange/1000,1) + " km    " at(0,2).
    	    print "Time til t_adv: " + round(t_adv-time:seconds,1) + " s    " at(0,3).

            set dxrange to abs(xrange - xrange_max).
            set dang to abs(central_ang-tgt_ang).

            if xrange > xrange_max {
    	        set t_adv to t_adv + max(1*dxrange/1000,1).
            } else {
                //set rm to 2.
                if dang < 0.01 {
                    set rm to 0.
                } else {
                    set t_adv to t_adv + max(1*dang/10,0.1).
                }
            }
        } else if rm = 2 {
            //Save previous results and display
            //Present option of advancing an orbit and trying again
        }
    }

    //local vec_eitgt is angleaxis(-300/(constant:pi*2*ship:body:radius)*360,orbitBinormal(SHIP))*vec_proj:normalized*(ship:body:radius+ship:body:atm:height).
    //local vec_eitgt_draw is vecdraw(kerbin:position,vec_eitgt,red,"Entry Interface",1.0,TRUE).


    set vec_site_draw:start to kerbin:position.
    set vec_site_draw:vec to vec_site.
    set vec_proj_draw:start to kerbin:position.
    set vec_proj_draw:vec to vec_proj.
    set vec_ship_draw:start to kerbin:position.
    set vec_ship_draw:vec to vec_ship.

    //kuniverse:pause().

    //Target a 45km periapsis for reentry
    local r1 is ship:altitude + ship:body:radius.
    local r2 is 45000 + ship:body:radius.

    local dv is sqrt(ship:body:mu/(r1)) * (sqrt((2*r2)/(r1+r2))-1).

    local mnv is node(t_adv, 0, 0, dv).
    clearVecDraws().
    return mnv.
}

function ground_positionAt {
    //Return the vector corresponding to a set of geocoordinates at a certain UT, give the body's rotation
    parameter coord.
    parameter t.
    parameter bod is ship:body.

    local vec_coord is coord:position-bod:position.

    local vec_coord_rotated is angleAxis(((t-time:seconds)/bod:rotationperiod)*360,bod:angularvel)*vec_coord + bod:position.

    return vec_coord_rotated.
}

function project_coord_orbit {
    //Project a set of ground coordinates onto ship's orbit
    parameter coord.

    local vec_coord is coord:position.

    local vec_proj is vxcl(orbitBinormal,vec_coord).

    return vec_proj.
}

function project_vec_to_orbit {
    //Project a vector onto the plane of an orbit
    parameter vec.
    parameter ves is ship.

    local vec_proj is vxcl(orbitBinormal(ves),vec).

    return vec_proj.
}

function central_angle {
    //Calculate the central angle between two vectors in an Earth-Centered Fixed coordinate system
    parameter vec1.
    parameter vec2.
    parameter bod is ship:body.

    local cang is vang(vec1-bod:position,vec2-bod:position).

    return cang.
}

function crossrange {
    //Calculate the crossrange distance given a central angle and a parent body
    parameter cang.
    parameter bod is ship:body.

    local xrange is bod:radius * (cang*constant:pi/360).

    return xrange.
}

function vang_360 {
    //Calculates the angle from v1 to v2 in the range of 0-360 degrees about a given axis of rotation vnorm
    parameter v1.
    parameter v2.
    parameter vnorm.
    
    local angle is vang(v1,v2).

    if vCrs(v1,v2)*vnorm < 0 {
        set angle to 360 - angle.
    } 

    return angle.
}

function shuttle_Deorbit_Exec_Node {
    //Uses the Shuttle Steer function to execute a maneuver node.
    parameter n is nextnode.

    local dv0 is n:deltav.

    local maxAcc is ship:maxThrust/ship:mass.

    local burnDuration is dv0:mag/maxAcc.

    local done is false.

    lock throttle to 0.

    clearscreen.

    until n:eta < (burnDuration/2)+65 {
        print "Burn ETA: " + round(n:eta - burnDuration/2,2) + " s     " at(0,0).
        wait 0.
    }
    retractAntenna().
    until n:eta < (burnDuration/2)+55 {
        print "Burn ETA: " + round(n:eta - burnDuration/2,2) + " s     " at(0,0).
        wait 0.
    }

	bays off.
    rcs on.
    sas off.
    lock steering to shuttleSteerDir(lookdirup(n:deltav:vec,-up:forevector)).

    until n:eta < (burnDuration/2) {
        print "Burn ETA: " + round(n:eta - burnDuration/2,2) + " s     " at(0,0).
        wait 0.
    }

    rcs off.
    lock throttle to 1.
    until done {
        set burnDuration to n:deltav:mag/(ship:maxThrust/ship:mass).
        if vdot(dv0, n:deltav) < 0 {set done to true.}
        if n:deltav:mag < 0.1 {set done to true.}
        print "Estimated Burn Remaining: " + round(burnDuration,2) + " s     " at(0,0).
    }
    lock throttle to 0.
    lock steering to shuttleSteerDir(lookdirup(prograde:forevector,-up:forevector)).
    rcs on.
    remove(n).
}

function entry_Guidance {
    parameter site.

    //Initialize Guidance Variables
    local t0 is time:seconds.
    local tlast is t0.
    local tnew is t0.
    local psi is site_Bearing(site).    //Bearing of your landing site from your velocity vector
    local xrange_max is 1.5.            //Maximum crossrange angle [deg]. This value is increased to 3 later in the function
    lock downrange to site:distance.    //Gives distance remaining to landing site.

    //Initialize variables for aerodynamic force calculation (Not really used in this version)
    local vold is ship:velocity:surface.
    local vnew is ship:velocity:surface.
    local totalAcc is V(0,0,0).
    local grav is body:mu * body:position / body:position:mag^3.
    local aeroAcc is totalAcc - grav.
    local dragVec is vproj(aeroAcc,ship:velocity:surface:normalized).
    local liftVec is vxcl(ship:velocity:surface:normalized,aeroAcc).

    ////////////////// Stock Atmosphere + JNSQ /////////////////////////////////
    ////Load target altitude data for interpolation
    //local breaks is list(60073.7112884398,126308.397694500,222828.868142166,349381.007403719,505636.369048762,683323.045557369,869596.922445097,1076363.75621087).
    //local coefs is list(list(1.38401373019906e-12,-7.04183988494879e-07,0.194124422103601,17513.2470891399),
    //                    list(1.38401373019906e-12,-4.29174842290630e-07,0.119056755360983,27683.8965633609),
    //                    list(-2.40519965493889e-13,-2.84178732561022e-08,0.0748896911829879,36421.5422673970),
    //                    list(2.37363110984958e-13,-1.19732821761202e-07,0.0561409037954621,44956.3848923044),
    //                    list(5.13093447008527e-14,-8.46504551676076e-09,0.0361092996818208,51710.8985324916),
    //                    list(8.40276900220255e-15,1.88859152844264e-08,0.0379609493971663,58147.6234402573),
    //                    list(8.40276900220255e-15,2.35815643603232e-08,0.0458715314722445,65928.3691454693)).
    //
    //local Alttgt is ship:body:atm:height.

    /////////////////// FAR atmosphere + JNSQ ////////////////////////////////////
    //Load target altitude data for interpolation
    local breaks is list(60035.5860346500,159865.418436400,290893.346469670,486097.522570790,728008.281124721,1004314.13489264,1297125.17694497,1588438.37890076).
    local coefs is list(list(6.004786e-14,-1.624274e-07,1.318023e-01,1.572787e+04),
                        list(6.004786e-14,-1.444425e-07,1.011654e-01,2.732734e+04),
                        list(6.251381e-14,-1.208369e-07,6.640383e-02,3.823872e+04),
                        list(1.467775e-13,-8.418384e-08,2.633460e-02,4.706778e+04),
                        list(2.413119e-14,2.236479e-08,1.137605e-02,5.059051e+04),
                        list(-1.667849e-14,4.235815e-08,2.925097e-02,5.594645e+04),
                        list(-1.667849e-14,2.771392e-08,4.975946e-02,6.771779e+04)).
    
    local Alttgt is ship:body:atm:height.

    //Initialize Roll Control PID
    //local rollTgtPID is PIDloop(-50,-1,-50,-5,40,0).
    //local rollTgtPID is PIDloop(-0.05,-0.001,-0.01,-5,70,0).
    local rollTgtPID is PIDloop(-0.1,-0.0001,-0.01,-5,70,0).
    local TEtgt is 5.8e+10.
    lock spTE to (ship:altitude * 9.81 + 1/2 * ship:velocity:surface:mag^2) / 1000^2.
    set rollTgtPID:setpoint to 8.

    //Increase 
    when downrange < 500000 then {
        set xrange_max to 3.
    }
    when ship:velocity:surface:mag < 1200 then {
        rcs off.
    }

    //Initial Bank angle is in the direction of the landing site
    local rollSign is -1.
    if psi < 0 {set rollSign to 1.}.

    //Initialize steering variables
    local rollAng is 0.
    local tgtRollAng is 0.
    local pitchAng is 40.

    set steeringManager:rollcontrolanglerange to 180.
    lock rolldir to angleAxis(rollAng,ship:velocity:surface)*lookdirup(ship:velocity:surface,up:vector).
	lock steerdir to angleAxis(pitchAng,-rolldir:starvector)*rolldir.

	lock steering to steerdir.

    clearscreen.

    until ship:velocity:surface:mag < 900 { //Entry Guidance
        wait 0.
        set tnew to time:seconds.
        //set dt to tnew - tlast.
        //set hnew to ship:altitude.
        //set hdotnew to (hnew - hlast)/dt.
        //set hdotdot to (hdotnew - hdotold)/dt.
        //set hlast to hnew.

        //set vnew to ship:velocity:surface.
        //set totalAcc to (vnew - vold)/dt.
        //set grav to body:mu * body:position / body:position:mag^3.
        //set aeroAcc to totalAcc - grav.
        //set dragVec to vproj(aeroAcc,ship:velocity:surface:normalized).
        //set liftVec to vxcl(ship:velocity:surface:normalized,aeroAcc).

        //set TETgt to 5.7e-14*(downrange)^3 - 1.3e-07*(downrange)^2 + 0.11*(downrange) + 6.9e+03.
        //set TETgt to -2.5e-18*(downrange)^3 + 3.9e-12*(downrange)^2 + 3.7e-06*(downrange) + 1.9.
        //set rollTgtPID:setpoint to TETgt.

        //Interpolate for target altitude
        set Alttgt to interp_cubic_spline(downrange,breaks,coefs).
        set rollTgtPID:setpoint to Alttgt.

        //Handle Roll Reversals
        set psi to site_Bearing(site).
        if abs(psi) > xrange_max {
            if psi > 0 {
                set rollSign to -1.
            } else {
                set rollSign to 1.
            }
        }

        //Control Roll Angle
        if time:seconds - t0 > 10 {
            //set tgtRollAng to rollSign * 35.
            //set tgtRollAng to rollSign * (20 + rollTgtPID:update(time:seconds,spTE)).
            set tgtRollAng to rollSign * (20 + rollTgtPID:update(time:seconds,ship:altitude)).
            //if ship:velocity:surface:mag < 3000 {
            //    set tgtRollAng to rollSign * 5.
            //}
        }

        //Smooth roll angle transition to avoid loss of control
        if abs(tgtRollAng - rollAng) > 0.01 {
            set rollAng to rollAng + ((tgtRollAng - rollAng)/abs(tgtRollAng - rollAng))*min(0.3,abs(tgtRollAng - rollAng)).
        }

        //Handle Pitch Schedule (FAR)
        if ship:velocity:surface:mag < 1800 and ship:velocity:surface:mag > 1400 {
            set pitchAng to 40 - (1800 - ship:velocity:surface:mag)/(1800 - 1400) * 35.
        }

        ////Handle Pitch Schedule (Stock)
        //if ship:velocity:surface:mag < 3000 and ship:velocity:surface:mag > 1400 {
        //    set pitchAng to 40 - (3000 - ship:velocity:surface:mag)/(3000 - 1400) * 35.
        //}

        print "Bearing:             " + round(psi,2) + "    " at(0,0).
        print "Distance:            " + round(downrange/1000,2) + "    " at(0,1).
        //print "Target Total Energy: " + round(TEtgt,2) + "    " at(0,2).
        //print "Currr. Total Energy: " + round(spTE,2) + "    " at(0,3).
        //print "Drag Acceleration:  " + round(dragVec:mag,2) + "    "at(0,3).
        //print "Lift Acceleration:  " + round(liftVec:mag,2) + "    "at(0,4).
        print "Target Altitude:     " + round(Alttgt,2) + "    " at(0,2).
        print "Currrent Altitude:   " + round(ship:altitude,2) + "    " at(0,3).
        print "Altitude Error:      " + round(ship:altitude-Alttgt,2) + "    " at(0,4).
        print "Target Roll:         " + round(rollAng,2) + "    " at(0,5).
        print "Commanded Roll:      " + round(tgtRollAng,2) + "    " at(0,6).
        print "Commanded Pitch:     " + round(pitchAng,2) + "    " at(0,7).

    }
}

function geo_Distance {
    //Function to calculate the distance between two sets of geocoordinates at a specified altitude
    parameter cord1.
    parameter cord2.
    parameter h is 0.
    parameter bod is ship:body.

    local vector1 is cord1:altitudeposition(h) - bod:position.
    local vector2 is cord2:altitudeposition(h) - bod:position.
    local theta is vang(vector1, vector2). // angle between the vectors.
    local circDist is theta * constant:degtorad * bod:radius. // distance of the circumference arc between the two vectors on a circle of the body's radius.

    return circDist.
}

function vector_Heading {
    //Calculates the heading of a vector
    //Example: get the heading of your velocity vector instead of your ship:facing vector

    parameter input_vector.

    local east_unit_vec is  vcrs(up:vector, north:vector).

    local east_vel is vdot(input_vector, east_unit_vec). 

    local north_vel is vdot(input_vector, north:vector).

    local compass is arctan2(east_vel, north_vel).

    if compass < 0 {
        set compass to compass + 360.
    }

  return compass.
}

function site_Bearing {
    //Returns the bearing to the target site based on the velocity vector, not the ship's attitude
    parameter site.
    return site:heading - vector_Heading(ship:velocity:surface).
}

function vproj {
	//Projection of vector a onto vector b
	parameter a.
	parameter b.

	return vdot(a,b)/(b:mag)^2 * b.
}

function interp_cubic_spline {
    //Interpolate data using a piecewise cubic spline
    //
    //Uses the closest polynomial for extrapolation
    //Takes inputs in the form of two lists and your current point to interpolate for:
    //  breaks = list of breakpoints (i.e. the supplied x-values for the piecewise spline)
    //  coefs  = list of lists of the parameters [a,b,c,d] for a cubic polynomial of the form:
    //  x      = current data point to interpolate a value for
    //
    //                          f(x) = a(x)^3 + b(x)^2 + c(x) + d
    //  
    //  There should be one fewer lists of coefficients than break points. Any more than that will be ignored

    parameter x is 0.5.
    parameter breaks is list(0,1).
    parameter coefs is list(list(1,1,1,1)).

    local x1 is breaks[0].
    local coef is coefs[0].

    local it is breaks:iterator.
    it:next.

    until x <= it:value {
        it:next().
        set x1 to breaks[it:index - 1].
        set coef to coefs[it:index - 1].
        if not it:next {break.}
    }

    local a is coef[0].
    local b is coef[1].
    local c is coef[2].
    local d is coef[3].

    return a*(x-x1)^3 + b*(x-x1)^2 + c*(x-x1) + d.
}
