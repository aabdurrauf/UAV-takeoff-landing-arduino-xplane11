import xpc

client = xpc.XPlaneConnect()
try:
    # If X-Plane does not respond to the request, a timeout error will be raised.
    client.getDREF("sim/test/test_float")
except:
    print("Error establishing connection to X-Plane.")

print("Connection established-xplane")

# CANNOT WRITE DATAREF LAT LONG IN XPLANE
# def set_lat_long(lat=None, long=None):
#     if type(lat) == int or type(lat) == float:
#         try:
#             client.sendDREF('sim/flightmodel/position/latitude', lat)
#         except:
#             client.clearBuffer()
#             print("Error setting the latitude.")

#     if type(long) == int or type(long) == float:
#         try:
#             client.sendDREF('sim/flightmodel/position/longitude', long)
#         except:
#             client.clearBuffer()
#             print("Error setting the longitude.") 

def set_coordinate(x=None, y=None, z=None):
    if type(x) == int or type(x) == float:
        try:
            client.sendDREF('sim/flightmodel/position/local_x', x)
        except:
            client.clearBuffer()
            print("Error setting the x coordinate.")

    if type(y) == int or type(y) == float:
        try:
            client.sendDREF('sim/flightmodel/position/local_y', y)
        except:
            client.clearBuffer()
            print("Error setting the y coordinate.") 
            
    if type(z) == int or type(z) == float:
        try:
            client.sendDREF('sim/flightmodel/position/local_z', z)
        except:
            client.clearBuffer()
            print("Error setting the z coordinate.")    

def shift_coordinate(x=None, y=None, z=None):
        if type(x) == int or type(x) == float:
            set_coordinate(x=client.getDREF(
                'sim/flightmodel/position/local_x')[0] + x)
        
        if type(y) == int or type(y) == float:
            set_coordinate(y=client.getDREF(
                'sim/flightmodel/position/local_y')[0] + y)
                
        if type(z) == int or type(z) == float:
            set_coordinate(z=client.getDREF(
                'sim/flightmodel/position/local_z')[0] + z)

# shift_coordinate(z=0, x=30, y=0)
