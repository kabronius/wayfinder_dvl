# List of coordinate system
uint8 BEAM = 0
uint8 XYZ = 1
uint8 SHIP = 2
uint8 EARTH = 3

uint8 coordinate_system

std_msgs/Header header

#Velocity
geometry_msgs/Vector3 velocity #Velocity in m/s
float64 velocity_error #Error vel in m/s

#Beams range in meters
float64 range_beam1
float64 range_beam2
float64 range_beam3
float64 range_beam4

float64 mean_range #Mean range in meters

float64 speed_of_sound #m/s









