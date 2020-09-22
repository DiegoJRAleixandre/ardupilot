The Vehicle will start by default in stabilize mode. As long as the vehicle receives PWM, in this case from Sbus in Herelink air transmitter, it switches to the value of the flight mode channel. 

For this particular use case we will use the change mode from the tablet, which goes through mavlink telemetry and not from RC channels. So in order for the copter to start in the mode we want
we need to set the flight mode correspondent to the initial Herelink output pwm flight mode channel to the mode we want to start with.

We want to start in manual mode ( loiter ), because simple and super simple modes require the copter to be on the selected mode prior to arming. It doesn't matter if we switch to guided and take off
in guided mode, but before that the copter must be in loiter simple or loiter super simple in order for it to catch the location and orientation and thus perform simple and super simple mode correctly.
If regular loiter ( no simple nor super simple ) is chosen we may start in whatever flight mode as this does not apply.

TODO - Explain pwm_out options

TODO - Explain spiral options



----- SUGGESTIONS FOR FUTURE DEVELOPMENTS -----

We think it could be interesting for your application to display over the vehicle icon in the map some kind of indication of the portion of terrain contained in the field of view of the camera, that changes
accordingly with each camera angle. In case you have the camera pointing downwards we could as well adjust that fov displaying taking as reference the altitude of takeoff. 