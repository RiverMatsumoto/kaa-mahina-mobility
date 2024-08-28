
State machine description and required clients:

### State machine

#### Global state used
- 2d array to store gps coordinates in a grid pattern
    - give initial point and grows north and east for rows and columns respectively

#### States
- navigate state
    - action client
- measure state
    - subscribe timer client for 100 seconds
        - emits time elapsed event
    - service client calls the ml service to return a grid point
        - use blackboard for the gps coordinates

Latitude offset = 2 / 111,139 (move upwards 2 meters)
Longitude offset = 2 / (111,139 * cos(initial latitude)) (move east 2 meters)
