import numpy as np
import math

class ArmPMT:
    def __init__(self, theta=0, phi=0):

        #PMT information
        self.R0 = 325                     # PMT's radius of curvature in    θ<t1 (θ:zenith angle)
        self.R1 = 225                     # PMT's radius of curvature in t1<θ<t2
        self.R2 = 125                     # PMT's radius of curvature in t2<θ<=π/2
        self.RPMT = 254

        #lc : x offset of centres
        #hc : z offset of centres

        self.lc0 = 0
        self.lc1 = 38.6
        self.lc2 = 129

        self.U1 = math.asin(self.lc1/(self.R0-self.R1))                 # radius changes at this angle / angle between C0 and C1
        self.Uchange1 = self.U1
        self.U12 = math.asin((self.lc2-self.lc1)/(self.R1-self.R2))     # angle between C1 and C2

        self.hc0 = 0
        self.hc1 = (self.R0-self.R1)*math.cos(self.U1)             # hc1 = (R0-R1)cos(U1) = 92.25 = 92.24987804869988
        self.hc2 = (self.R0-self.R1)*math.cos(self.U1) + (self.R1-self.R2)*math.cos(self.U12)

        self.U2 = math.atan(self.lc2/self.hc2)             # angle between C0 and C2
        self.Uchange2 = math.atan((self.lc1+self.R1*math.sin(self.U12))/(self.hc1+self.R1*math.cos(self.U12)))             # radius changes at this angle 
        print(math.degrees(self.Uchange2))
        self.Urob_change = math.radians(42)                # angle where the robot must change its configuration
        self.Uend = math.atan(self.RPMT/self.hc2)

        self.r01 = math.sqrt(self.lc1**2 + self.hc1**2) #distance from C0 to C1
        self.r02 = math.sqrt(self.lc2**2 + self.hc2**2) #distance from C0 to C2

        self.d = 90 # the distance from the light to the surface of PMT
        self.h = 690 # the distance between base of the robot arm (origin) and the top of the PMT

    def quad_solve(a, b, c):
        disc = b**2 - 4*a*c
        x1 = (-b + math.sqrt(disc))/(2*a)
        x2 = (-b - math.sqrt(disc))/(2*a)
        return (x1, x2)
    
    def pitch_of(self, theta):
        theta = math.degrees(theta)
        return -(90-theta)
    
    def get_cart(self, theta):
        '''
        Takes in a desired zenith angle and calculates the required cartesian
        coordinates (x, y, z, roll, pitch, yaw)
        '''

        theta = math.radians(theta)
        beta1 = theta - self.U1
        beta2 = theta - self.U2

        # Check if the zenith angle chosen has crossed any radial changes
        if theta<=self.U1:
            print("Use R0")
            r = self.R0

            # Identify the (x, y, z) of the TCP
            x = (r+self.d)*math.sin(theta)
            y = 0
            z = (self.h+r)-(r+self.d)*math.cos(theta)

            # Identify the (roll, pitch, yaw) of the TCP
            roll = -180
            pitch = self.pitch_of(theta)
            yaw = 0

        elif self.U1<theta<=self.Uchange2:
            print("Use R1")

            ohm = math.asin((self.r01/self.R1)*math.sin(beta1))
            gamma = theta + ohm

            x = self.lc1 + (self.R1+self.d)*math.sin(gamma)
            y = 0
            z = (self.h+self.R0) - (self.hc1 + (self.R1+self.d)*math.cos(gamma))

            # Check if the zenith angle is such that the robot needs to change
            # its configuration
            if theta <= self.Urob_change:
                roll = -180
                pitch = self.pitch_of(gamma)
                yaw = 0

            else:
                roll = 0
                pitch = self.pitch_of(gamma)
                yaw = 0

        elif self.Uchange2<theta<self.Uend:
            print("Use R2")

            ohm = math.asin((self.r02/self.R2)*math.sin(beta2))
            gamma = theta + ohm

            x = self.lc2 + (self.R2+self.d)*math.sin(gamma)
            y = 0
            z = (self.h+self.R0) - (self.hc2 + (self.R2+self.d)*math.cos(gamma))

            roll = 0
            pitch = self.pitch_of(gamma)
            yaw = 0

        else:
            print("Please select angle between 0 and 61.89181")
            return

        return [-x, y, z, roll, pitch, yaw]
    

    def rotate_base(self, phi, joint_target):
        '''
        Taking in a azimuthal angle and the joint angles decided from inverse kinematics,
        change the angle of the first joint thus rotating the base of the robot
        '''

        joint_target[0] = (round(joint_target[0])+phi)%360

        return joint_target
