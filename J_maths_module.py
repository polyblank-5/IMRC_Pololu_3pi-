import math

#taken directly from Summer school code 
# Define Bezier curve functions
def bezier(p, t):
    return (1-t)**3*p[0] + 3*t*(1-t)**2*p[1]+3*t**2*(1-t)*p[2]+t**3*p[3]

def bezier_d(p, t):
    return 3*(1-t)**2 *(p[1] - p[0]) + 6*(1-t)*t*(p[2] - p[1]) + 3*t**2*(p[3] - p[2])

def bezier_dd(p, t):
    return 6*(1-t)*(p[2]-2*p[1]+p[0])+6*t*(p[3]-2*p[2]+p[1])



# input: x, y and derivatives; output: [x,y,theta], v, omega
def diff_flatness(self, x, y, x_dot, y_dot, x_ddot, y_ddot):
    theta = math.atan2(y_dot, x_dot)
    v = math.sqrt(y_dot**2 + x_dot**2)
    omega = (x_dot*y_ddot - y_dot*x_ddot) / (x_dot**2 + y_dot**2)
    return [x, y, theta], v, omega


#round down to the nth decimal place
def round_down(n, decimals=0):
    multiplier = 10**decimals
    return math.floor(n * multiplier) / multiplier




