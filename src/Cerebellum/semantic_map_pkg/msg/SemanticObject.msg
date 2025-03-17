string category  # category of the object, one of the prompt list
uint32 count  # number of points in the object
float32[] x  # x coordinates of the points, in map frame
float32[] y  # y coordinates of the points
float32[] z  # z coordinates of the points
uint32[] rgb  # rgb values of the points
float32 confidence  # confidence of the object
sensor_msgs/Image image  # rgb image of the object
