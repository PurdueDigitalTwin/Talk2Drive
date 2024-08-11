import requests
import xml.etree.ElementTree as ET


# API Keys
# OPENWEATHER_API_KEY = ''
# TOMTOM_API_KEY = ''


# weather information through Openweather API
def get_weather_info(position):
    # Define the API endpoint and the API key
    
    url = "https://api.openweathermap.org/data/2.5/weather"
    parameters = {
        'lat': position[0],
        'lon': position[1],
        'appid': OPENWEATHER_API_KEY
    }

    # Make the HTTP request to the OpenWeatherMap API
    response = requests.get(url, params=parameters)

    # Check if the request was successful
    if response.status_code == 200:
        data = response.json()
        # Extract the necessary information
        main = data['weather'][0]['main']
        description = data['weather'][0]['description']
        city = data['name']
        country = data['sys']['country']
        
        # Format the extracted information into a readable string
        result = f"We are currently in {city}, {country}. The weather is {description} ({main})."
        return result
    else:
        # Return an error message if something goes wrong
        return "The current weather information is not available."


# map information (such as road type and speed limits) through OpenStreetMap API
def get_speed_limit(position):
    # use the current position to get the speed limit
    import overpy
    open_street_map_api = overpy.Overpass()
    result = open_street_map_api.query(f"way(around:100,{position[0]},{position[1]})[maxspeed];out;")

    # parse the result and extract the speed limit information
    speed_limits = []
    street_names = []
    road_types = []
    for way in result.ways:
        speed_limit =  way.tags.get("maxspeed")
        street_name = way.tags.get("name")
        road_type = way.tags.get("highway")
        if speed_limit is not None:
            speed_limits.append(speed_limit)
            street_names.append(street_name)
            road_types.append(road_type)
    
    # format the extracted information into a readable string.
    # check if the speed limit is available, if not return an error message
    if len(speed_limits) == 0:
        return "The speed limit information is currently not available."
    else:
        # get the speed limit with the most occurrences and the corresponding street name and road type
        speed_limit = max(set(speed_limits), key = speed_limits.count)
        street_name = street_names[speed_limits.index(speed_limit)]
        road_type = road_types[speed_limits.index(speed_limit)]
        speed_limit_kmh = int(speed_limit.split()[0]) * 1.60934
        speed_limit = f"{speed_limit} ({speed_limit_kmh:.2f} km/h)"
        result = f"We are currently on street {street_name}, where the speed limit is {speed_limit} and the road type is {road_type}."
        return result


# traffic information through TomTom API
def get_traffic_info(position):
    # Define the API endpoint
    url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/xml"
    parameters = {
        'key': TOMTOM_API_KEY,
        'point': f"{position[0]},{position[1]}",
        'unit': 'mph'
    }

    # Make the HTTP request to the TomTom API
    response = requests.get(url, params=parameters)

    # Check if the request was successful
    if response.status_code == 200:
        # Parse the XML response
        root = ET.fromstring(response.content)
        
        # Extract the current speed and free flow speed from the XML data
        current_speed = root.find('currentSpeed').text
        free_flow_speed = root.find('freeFlowSpeed').text
        
        # Format the extracted information into a readable string
        result = f"Current traffic flow speed is {current_speed} mph, and the free flow speed is {free_flow_speed} mph."
        return result
    else:
        # Return an error message if something goes wrong
        return "Failed to retrieve the traffic information."
    

def get_driving_context():
    # Get the current position
    from utils.get_location import get_current_position
    position, locating_method = get_current_position()
    if locating_method == 'GNSS':
        locating_method_info = "The current position was obtained using GPS, so the information is accurate."
    else:
        locating_method_info = "The current position was obtained using IP geolocation, so the street name, speed and traffic information may not be accurate."
    # Get the weather information
    weather_info = get_weather_info(position)
    # Get the speed limit information
    speed_limit_info = get_speed_limit(position)
    # Get the traffic information
    traffic_info = get_traffic_info(position)
    
    # Combine the information into a single string
    driving_context = f"{locating_method_info}\n{weather_info}\n{speed_limit_info}\n{traffic_info}"
    return driving_context


if __name__ == '__main__':
    print(get_driving_context())
