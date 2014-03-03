from ode import (
    World, GeomPlane, Space, JointGroup, collide, ContactJoint
)

from visual import rate, box, scene
from quad import Quad
from utils import set_wii_remote

from flight import Flight

import cwiid


class Simulator(object):

    def __init__(self, Stable, wm, quad_pos=(0, 0, 0)):
        self.world = World()
        self.world.setGravity((0, -9.81, 0))
        self.space = Space()
        self.contactgroup = JointGroup()
        self.wm = wm

        # draw floor
        self.floor = GeomPlane(self.space, (0, 1, 0), -1)
        box(pos=(0, -1, 0), width=2, height=0.01, length=2)

        # Draw Quad
        self.quad = Quad(self.world, self.space, pos=quad_pos)

        # Init Algorithm
        self.stable = Stable(self.quad)

        # stop autoscale of the camera
        scene.autoscale = False

        # Initial noise
        #quad.motors[2].addForce((0.5, 1, 0))

    # Collision callback
    def near_callback(self, args, geom1, geom2):
        """Callback function for the collide() method.
        This function checks if the given geoms do collide and
        creates contact joints if they do.
        """
        # Check if the objects do collide
        contacts = collide(geom1, geom2)

        # Create contact joints
        world, contactgroup = args
        for c in contacts:
            c.setBounce(0.2)
            c.setMu(5000)
            j = ContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())


    def step(self):
        # Control Quad if controls are enabled
        res = self.wii_remote()

        # Detect collisions
        self.space.collide((self.world, self.contactgroup), self.near_callback)
        # run algorithm
        self.stable.step()
        self.world.step(self.dt)
        self.contactgroup.empty()

        # point camera to quad
        scene.center = self.quad.pos
        self.print_log()
        return res

    def wii_remote(self):
        if self.wm:
            if self.wm.state['buttons'] & cwiid.BTN_LEFT:
                self.stable.command_yaw = 1
            if self.wm.state['buttons'] & cwiid.BTN_RIGHT:
                self.stable.command_yaw = -1
            if self.wm.state['buttons'] & cwiid.BTN_1:
                self.stable.command_acceleration = 1
            if self.wm.state['buttons'] & cwiid.BTN_2:
                self.stable.command_acceleration = -1
            if self.wm.state['buttons'] & cwiid.BTN_A and self.wm.state['acc']:
                p = (wm.state['acc'][1] - 125.) / 125.
                r = (wm.state['acc'][0] - 125.) / 125.
                self.stable.command_pitch = -p
                self.stable.command_roll = r
            if self.wm.state['buttons'] & cwiid.BTN_B and self.wm.state['acc']:
                a = (wm.state['acc'][1] - 125.) / 125.
                self.stable.command_acceleration = a * 10
            if self.wm.state['buttons'] & cwiid.BTN_HOME:
                self.wm.close()
                return True
            if not self.wm.state['buttons']:
                self.stable.command_pitch = 0
                self.stable.command_roll = 0
                self.stable.command_yaw = 0
                self.stable.command_acceleration = 0
        return False

    def print_log(self):
        print "%1.2fsec: pos=(%6.3f, %6.3f, %6.3f) vel=(%6.3f, %6.3f, %6.3f) pry=(%6.3f, %6.3f, %6.3f)" % \
              (self.total_time, self.quad.pos[0], self.quad.pos[1], self.quad.pos[2],
                        self.quad.vel[0], self.quad.vel[1], self.quad.vel[2],
                        self.quad.pitch, self.quad.roll, self.quad.yaw)
        print "power=(%6.3f, %6.3f, %6.3f, %6.3f)" % \
            (self.quad.motors_force[0], self.quad.motors_force[1],
             self.quad.motors_force[2], self.quad.motors_force[3])

    def simulate(self):
        # Do the simulation...
        self.total_time = 0.0
        self.dt = 0.04
        self.stop = False
        while not self.stop:
            rate(self.dt*1000)
            self.step()
            self.total_time += self.dt


if __name__ == '__main__':
    # enable wii remote?
    wm = set_wii_remote()
    # Init simulattion
    sim = Simulator(Flight, wm)
    sim.simulate()

