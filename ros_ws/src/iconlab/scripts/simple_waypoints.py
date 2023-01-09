"""Single CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np

from pycrazyswarm import Crazyswarm


Z = 1.0
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 1.0

n = 20

x = np.linspace(0,3.14,n)
y = 0.5*np.sin(2*x)

print(x)
print(y)

WAYPOINTS = np.zeros([2*n-1,3])

for i in range(n-1):
	WAYPOINTS[i,0] = x[i]-1.57
	WAYPOINTS[i,1] = 0
	WAYPOINTS[i,2] = 1+y[i]
	WAYPOINTS[2*n-2-i,0] = x[i]-1.57
	WAYPOINTS[2*n-2-i,1] = 0
	WAYPOINTS[2*n-2-i,2] = 1-y[i]

WAYPOINTS[n-1,0] = x[n-1]-1.57
WAYPOINTS[n-1,1] = 0
WAYPOINTS[n-1,2] = 1+y[n-1]

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)
    
    cf.goTo([-1.57,0,1], yaw=0.0, duration=4.0)
    timeHelper.sleep(5.0)

    for p in WAYPOINTS:
        cf.goTo(cf.initialPosition + p, yaw=0.0, duration=GOTO_DURATION)
        timeHelper.sleep(0.3)
    
    timeHelper.sleep(3)
        
    #cf.goTo(cf.initialPosition, yaw=0.0, duration=3.0)

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()
