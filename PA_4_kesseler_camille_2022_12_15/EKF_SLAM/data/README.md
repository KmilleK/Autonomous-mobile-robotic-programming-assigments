### Dataset information


############################################################
### sensor_data.dat

if {DATA_KEY=="ODOMETRY"}
ODOMETRY {ROTATION1} {TRANSLATION} {ROTATION2}

if {DATA_KEY=="SENSOR"}
SENSOR {LANDMRK_ID} {RANGE} {BEARING(in rad)}
############################################################


############################################################
### world.dat

{LANDMARK_ID} {x} {y}
############################################################