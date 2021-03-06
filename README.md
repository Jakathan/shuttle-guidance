# shuttle-guidance
A collection of kOS scripts for deorbit and reentry guidance for the Stock Shuttle Orbiter Kit for Kerbal Space Program

So far this project includes two files:
lib_shuttle_mnv.ks and lib_shuttle_deorbit.ks

## lib_shuttle_mnv.ks
Contains a few functions to handle the offset thrust vector found on the shuttle:
### centerOfThrust()
Calculates the thrust-weighted average position of all active engines. This is mainly used to feed into the next function:

### shuttleSteerDir(\<direction>) 
Takes an input direction, and then uses centerOfThrust() to calculate the direction the shuttle should face so that the thrust vector is oriented through the center of mass toward the input direction. This is useful for OMS burns and can completely replace the need to switch the cockpit control angle.

 ### shuttle_Node(\<node>)
A simple maneuver node execution function that uses the offset steering direction calculated in the previous function to execute an input maneuver node. It defaults to executing the next maneuver node. There are plans to improve this function, but for now it works well enough.
  
  
## lib_shuttle_deorbit.ks
This is the current main focus of this project. It contains the functions necessary to plan a deorbit burn to target a specific landing site, and is based partly on the actual [Space Shuttle's guidance computer algorithms](https://core.ac.uk/download/pdf/80647106.pdf).

### deorbit_maneuver(\<site coordinates>,\<earliest burn time>,\<EI fpa>,\<EI distance>,\<max crossrange>)
Takes several input parameters and calculates a deorbit burn that will ensure the shuttle's reentry path is sufficiently close to the input coordinates that aerodynamics can be used to maneuver to the landing site during the entry guidance. As of right now, only the target landing site, earliest burn time, and maximum crossrange do anything of note, but there are plans to switch to entry interface targeting instead of periapsis and burn anomaly targeting in the future.

### entry_Guidance(\<site coordinates>)
Performs entry guidance all the way until the shuttle is at 900 m/s. This function is still very much a work in progress, but the general idea is that it uses an altitude vs. downrange cubic spline interpolation in order to calculate a target altitude for a roll angle PID controller. This spline table was generated by flying a "nominal" flight and then using MATLAB to fit a spline to the recorded data. The current code is based on FAR aerodynamics using the JNSQ planet system, though a sample table for stock atmosphere with JNSQ is included as well. When the maximum crossrange angle is exceeded, a roll reversal is commanded by flipping the sign of the target roll angle. The commanded roll angle is limited to 0.3 degrees from the current roll angle to prevent loss of control. Pitch angle is held at 40 degrees for most of reentry, but is ramped down to 5 degrees as you approach the end of entry guidance for transition into terminal area guidance, which is not yet implemented in this project.

All other functions found in this file are used by the previous two functions.

The spline interpolation function is definitely not quite right (I think it pulls from the spline ahead of where it should) but it works well enough (tm) for now.

## Example Usage

```
runOncePath("0:/library/lib_shuttle_deorbit.ks").
runOncePath("0:/library/lib_shuttle_mnv.ks").

local site is latlng(<coordinates of KSC>)

local deorbit_mnv is deorbit_maneuver(site).
add(deorbit_mnv).

shuttle_Node().

wait until altitude < ship:body:atm:height.

entry_Guidance(site).

//Your chosen approach & landing script goes here :)
```
