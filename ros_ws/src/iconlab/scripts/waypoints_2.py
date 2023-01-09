import numpy as np 

n = 20

x = np.linspace(0,3.14,n)
y = 0.5*np.sin(2*x)

print(x)
print(y)

WAYPOINTS = np.zeros([2*n-1,3])

for i in range(n-1):
	WAYPOINTS[i,0] = x[i]
	WAYPOINTS[i,1] = 0
	WAYPOINTS[i,2] = 1+y[i]
	WAYPOINTS[2*n-2-i,0] = x[i]
	WAYPOINTS[2*n-2-i,1] = 0
	WAYPOINTS[2*n-2-i,2] = 1-y[i]

WAYPOINTS[n-1,0] = x[n-1]
WAYPOINTS[n-1,1] = 0
WAYPOINTS[n-1,2] = 1+y[n-1]

print(WAYPOINTS)
