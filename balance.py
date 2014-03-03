from pid import PID
from utils import set_wii_remote
from visual import box

from sim import Simulator

class Roll(object):
    def __init__(self, quad):
        self.quad = quad
        self.roll = PID(0.02, 0.001, 0.10)

    def step(self):
        pforce = self.roll.update(self.quad.roll)
        self.quad.motors_force = (0, 0, -pforce, pforce)
        self.quad.step()

class Balance(Simulator):

    def __init__(self, Stable, wm, quad_pos=(0, -0.8, 0)):
        super(Balance, self).__init__(Stable, wm, quad_pos=quad_pos)
        # Fix some axis
        self.quad.center.static()
        self.quad.motors[0].static()
        self.quad.motors[1].static()
        # Draw the anchors
        box(pos=(0, -0.9, 0), width=0.1, height=0.2, length=0.1)
        box(pos=(0, -0.9, self.quad.size/2.), width=0.1, height=0.2, length=0.1)
        box(pos=(0, -0.9, -self.quad.size/2.), width=0.1, height=0.2, length=0.1)
        # add some noise
        self.quad.motors[2].addForce((0, 2, 0))

if __name__ == '__main__':
    # enable wii remote?
    wm = set_wii_remote()
    # Init simulattion
    sim = Balance(Roll, wm)
    sim.simulate()

