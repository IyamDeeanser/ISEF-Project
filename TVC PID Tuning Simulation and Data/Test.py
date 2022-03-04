simTime = 0
simLength = 5000
timeInterval = 5
thrustCurve = [
[0, 0],
[0.063, 2.127],
[0.118, 4.407],
[0.158, 8.359],
[0.228, 13.68],
[0.34, 20.82],
[0.386, 26.75],
[0.425, 25.38],
[0.481, 22.19],
[0.583, 17.93],
[0.883, 16.11],
[1.191, 14.59],
[1.364, 15.35],
[1.569, 15.65],
[1.727, 14.74],
[2.0, 14.28],
[2.39, 13.68],
[2.68, 13.08],
[2.96, 13.07],
[3.25, 13.05],
[3.35, 13.0],
[3.39, 7.3],
[3.4, 0.0],
[5.1, 0.0],
]
def func_thrust(Time):
    global thrust
    index = 0
    Time = Time/1000
    for item in thrustCurve:
        if item[0] < Time:
            index += 1
        else:
            break
    point1 = thrustCurve[index-1]
    point2 = thrustCurve[index]
    slope = (point1[1]-point2[1])/(point1[0]-point2[0])
    thrust = slope * (Time-point1[0]) + point1[1]
    #print(thrust, Time)
    return thrust
    
while simTime<simLength: #actual simulation is here
    simTime += timeInterval
    thrust = func_thrust(simTime)
    print(simTime,thrust)