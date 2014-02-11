from ode import World
from visual import rate, box
from pid import PID
from quad import Quad
from utils import set_wii_remote

class Stable(object):

    def __init__(self, quad, max_force=10, min_force=0.02):
        self.quad = quad
        # set max and min forces
        self.max_force = max_force
        self.min_force = min_force
        # Aceleration set points
        self.set_point_aceleration = 0
        self.aceleration = PID(4, 0.8, 2)
        self.aceleration.setPoint(self.set_point_aceleration)
        # Pitch set point
        self.set_point_pitch = 0
        self.pitch = PID(0.02, 0.001, 0.10)
        self.pitch.setPoint(self.set_point_pitch)
        # Roll set point
        self.set_point_roll = 0
        self.roll = PID(0.02, 0.001, 0.10)
        self.roll.setPoint(self.set_point_roll)
        # Yaw set point
        self.set_point_yaw = 0
        self.yaw = PID(0.2, 0.001, 1)
        self.yaw.setPoint(self.set_point_yaw)

    def set_aceleration(self, aceleration):
        if self.set_point_aceleration != aceleration:
            self.set_point_aceleration = aceleration
            self.aceleration(aceleration)

    def set_pitch(self, pitch):
        if self.set_point_pitch != pitch:
            self.set_point_pitch = pitch
            self.pitch.setPoint(self.set_point_pitch)

    def set_roll(self, roll):
        if self.set_point_roll != roll:
            self.set_point_roll = roll
            self.roll.setPoint(self.set_point_roll)

    def set_yaw(self, yaw):
        if self.set_point_yaw != yaw:
            self.set_point_yaw = yaw
            self.yaw.setPoint(self.set_point_yaw)

    def update(self):
        self.quad.pos
        forces = []
        # aceleration
        aforce = self.aceleration.update(self.quad.vel[1])
        forces = [aforce, aforce, aforce, aforce]
        # pitch
        pforce = self.pitch.update(self.quad.pitch)
        forces[0] += pforce
        forces[1] -= pforce
        # roll
        rforce = self.roll.update(self.quad.roll)
        forces[3] += rforce
        forces[2] -= rforce
        # yaw
        yforce = self.yaw.update(self.quad.yaw)
        forces[0] += yforce
        forces[1] += yforce
        forces[2] -= yforce
        forces[3] -= yforce

        # Update forces in min and max values
        for i, force in enumerate(forces):
            force = max(self.min_force, force)
            force = min(self.max_force, force)
            forces[i] = force
        # set values to quad
        self.quad.motors_force = forces

    def step(self):
        self.update()
        self.quad.step()


if __name__ == '__main__':
    # enable wii remote?
    wm = set_wii_remote()
    if wm:
        import cwiid

    world = World()
    world.setGravity((0, -9.81, 0))

    # draw floor
    floor = box(pos=(0, -1, 0), width=1, height=0.01)

    quad = Quad(world, pos=(0, 1, 0))
    stable = Stable(quad)

    # Initial noise
    quad.motors[2].addForce((0.5, 1, 0))

    # Do the simulation...
    total_time = 0.0
    dt = 0.04
    while True:
        rate(dt*1000)

        # Control Quad if controls are enabled
        if wm:
            if wm.state['buttons'] & cwiid.BTN_UP:
                stable.set_roll(1)
            if wm.state['buttons'] & cwiid.BTN_DOWN:
                stable.set_roll(-1)
            if wm.state['buttons'] & cwiid.BTN_LEFT:
                stable.set_pitch(1)
            if wm.state['buttons'] & cwiid.BTN_RIGHT:
                stable.set_pitch(-1)
            if wm.state['buttons'] & cwiid.BTN_RIGHT:
                stable.set_pitch(-1)
            if wm.state['buttons'] & cwiid.BTN_1:
                stable.set_altitude(-0.5)
            if wm.state['buttons'] & cwiid.BTN_2:
                stable.set_altitude(1)
            if wm.state['buttons'] & cwiid.BTN_A and wm.state['acc']:
                # apply noise to motors directly
                n1 = (wm.state['acc'][0] - 125.) / 125.
                n2 = (wm.state['acc'][1] - 125.) / 125.
                quad.motors[0].addForce((0, n1, 0))
                quad.motors[2].addForce((0, n2, 0))
                print 'noise', n1, n2
            if wm.state['buttons'] & cwiid.BTN_HOME:
                wm.close()
                break
            if not wm.state['buttons']:
                stable.set_aceleration(0)
                stable.set_pitch(0)
                stable.set_roll(0)
                stable.set_way(0)

        print "%1.2fsec: pos=(%6.3f, %6.3f, %6.3f) vel=(%6.3f, %6.3f, %6.3f) pry=(%6.3f, %6.3f, %6.3f)" % \
              (total_time, quad.pos[0], quad.pos[1], quad.pos[2],
                        quad.vel[0], quad.vel[1], quad.vel[2],
                        quad.pitch, quad.roll, quad.yaw)
        print "power=(%6.3f, %6.3f, %6.3f, %6.3f)" % (quad.motors_force[0], quad.motors_force[1], quad.motors_force[2], quad.motors_force[3])

        stable.step()
        world.step(dt)

        total_time+=dt
