from pid import PID
from utils import set_wii_remote
from visual import box
import cwiid

from sim import Simulator

class Roll(object):
    def __init__(self, quad):
        self.quad = quad
        self.command_roll = 0
        self.roll = PID(0.02, 0.001, 0.1)
        self.roll.setPoint(self.command_roll)

    def step(self):
        print self.command_roll
        if self.roll.getPoint() != self.command_roll:
            self.roll.setPoint(self.command_roll)
        pforce = self.roll.update(self.quad.roll)
        self.quad.motors_force = (0, 0, -pforce, pforce)
        self.quad.step()


class SimBalance(Simulator):

    def __init__(self, Stable, wm, quad_pos=(0, -0.6, 0)):
        super(SimBalance, self).__init__(Stable, wm, quad_pos=quad_pos)
        # Fix some axis
        self.quad.center.static()
        self.quad.motors[0].static()
        self.quad.motors[1].static()
        # Draw the anchors
        box(pos=(0, -0.8, 0), width=0.1, height=0.4, length=0.1)
        box(pos=(0, -0.8, self.quad.size/2.), width=0.1, height=0.4, length=0.1)
        box(pos=(0, -0.8, -self.quad.size/2.), width=0.1, height=0.4, length=0.1)
        # add some noise
        self.quad.motors[2].addForce((0, 2, 0))

    def wii_remote(self):
       if self.wm:
            if self.wm.state['buttons'] & cwiid.BTN_LEFT:
                self.stable.command_roll += 1
            if self.wm.state['buttons'] & cwiid.BTN_RIGHT:
                self.stable.command_roll -= 1
            if self.wm.state['buttons'] & cwiid.BTN_A and self.wm.state['acc']:
                r = (wm.state['acc'][0] - 125.)
                self.stable.command_roll = -r


if __name__ == '__main__':
    # enable wii remote?
    wm = set_wii_remote()
    # Init simulattion
    sim = SimBalance(Roll, wm)
    sim.simulate()

