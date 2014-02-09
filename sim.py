from ode import World
from visual import rate, box
from pid import PID
from quad import Quad
from utils import set_wii_remote

class Stable(object):
    P = 4
    I = 0.8
    D = 2

    def __init__(self, quad, set_points=(0., 0., 0., 0.), max_force=10, min_force=0.02):
        self.quad = quad
        # Main set points and pids
        self.set_point_altitude = 0
        self.set_points = set_points
        self.pids = [PID(self.P, self.I, self.D) for p in range(4)]
        for i, pid in enumerate(self.pids):
            pid.setPoint(self.set_points[i])
        self.max_force = max_force
        self.min_force = min_force
        # Pitch set point
        self.set_point_pitch = 0
        self.pitch = PID(0.02, 0.001, 0.10)
        self.pitch.setPoint(self.set_point_pitch)
        # Roll set point
        self.set_point_roll = 0
        self.roll = PID(0.02, 0.001, 0.10)
        self.roll.setPoint(self.set_point_roll)

    def set_pitch(self, pitch):
        if self.set_point_pitch != pitch:
            self.set_point_pitch = pitch
            self.pitch.setPoint(self.set_point_pitch)

    def set_roll(self, roll):
        if self.set_point_roll != roll:
            self.set_point_roll = roll
            self.roll.setPoint(self.set_point_roll)

    def set_altitude(self, altitude):
        if self.set_point_altitude != altitude:
            self.set_point_altitude = altitude
            for pid in self.pids:
                pid.setPoint(altitude)

    def update(self):
        self.quad.pos
        forces = []
        for i, pid in enumerate(self.pids):
            force = pid.update(self.quad.vel[1])
            force = max(self.min_force, force)
            force = min(self.max_force, force)
            forces.append(force)
        # pitch
        pforce = self.pitch.update(self.quad.pitch)
        pforce = max(-2, pforce)
        pforce = min(2, pforce)
        forces[0] += pforce
        # roll
        rforce = self.roll.update(self.quad.roll)
        rforce = max(-2, rforce)
        rforce = min(2, rforce)
        forces[3] += rforce
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
                stable.set_altitude(0)
                stable.set_pitch(0)
                stable.set_roll(0)

        print "%1.2fsec: pos=(%6.3f, %6.3f, %6.3f) vel=(%6.3f, %6.3f, %6.3f) pry=(%6.3f, %6.3f, %6.3f)" % \
              (total_time, quad.pos[0], quad.pos[1], quad.pos[2],
                        quad.vel[0], quad.vel[1], quad.vel[2],
                        quad.pitch, quad.roll, quad.yaw)
        print "power=(%6.3f, %6.3f, %6.3f, %6.3f)" % (quad.motors_force[0], quad.motors_force[1], quad.motors_force[2], quad.motors_force[3])

        stable.step()
        world.step(dt)

        total_time+=dt
