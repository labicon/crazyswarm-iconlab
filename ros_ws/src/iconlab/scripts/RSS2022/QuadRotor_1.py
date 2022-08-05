import numpy as np

from pycrazyswarm import Crazyswarm

#import julia
#jl = julia.Julia(compiled_modules=False)
#from julia import Main
#X = Main.include('QuadRotor_1.jl')
#X = X.T

X = np.loadtxt("Waypoints.csv",delimiter=',')
print(X.shape)
X[:,0] = -X[:,0]
X[:,1] = -X[:,1]

Z = 1.5
TAKEOFF_DURATION = 5
GOTO_DURATION = 3
WAYPOINTS = X


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 0.22)

    for p in WAYPOINTS:
        cf.goTo(cf.initialPosition + p, yaw=0.0, duration=GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION + )

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 0.22)


if __name__ == "__main__":
    main()
