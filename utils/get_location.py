#!/usr/bin/env python

current_position = [None, None] # [latitude, longitude]

def get_current_position():
    # Try to use Novatel GNSS location if available
    # If not available, fallback to geocoder
    try:
        import rospy
        from novatel_oem7_msgs.msg import BESTGNSSPOS
        ROS_AVAILABLE = True
    except ImportError:
        ROS_AVAILABLE = False

    global current_position
    current_position = [None, None] # reset the position
    method = None

    # Read GNSS location from ROS topic if available
    if ROS_AVAILABLE:
        method = "GNSS"
        gnss_location_listener()
    if current_position[0] is None or current_position[1] is None:
        # Fallback to geocoder if ROS or novatel_oem7_msgs is not available
        try:
            import geocoder
            g = geocoder.ip('me')
            if g.latlng:
                current_position = g.latlng
                method = "Geocoder"
            else:
                print("Geocoder did not return a location.")
        except ImportError:
            print("Geocoder module is not installed, and ROS is not available.")
    return current_position, method


def gnss_location_callback(data):
    global current_position
    current_position[0] = data.lat
    current_position[1] = data.lon
    print("Latitude: %f Longitude: %f" % (current_position[0], current_position[1]))
    # Signal to stop listening after receiving the first message
    global listener_active
    listener_active = False


def gnss_location_listener():
    global listener_active
    listener_active = True
    rospy.init_node('gnss_pos_listener', anonymous=True, disable_signals=True)
    rospy.Subscriber("/novatel/oem7/bestgnsspos", BESTGNSSPOS, gnss_location_callback)
    
    # Instead of rospy.spin(), use a while loop to control the execution
    i = 0
    while listener_active and not rospy.is_shutdown():
        rospy.sleep(0.1)  # Sleep to prevent excessive CPU usage
        i += 1
        if i > 3:
            return
        

if __name__ == '__main__':
    for i in range(5):
        position = get_current_position()
        print("Current position: ", position)