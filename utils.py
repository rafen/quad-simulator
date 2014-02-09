from ode import Body, Mass, BallJoint, environment
from visual import sphere


class Sphere(object):
    def __init__(self, world, pos, size, mass):
        self.world = world
        self.pos = pos
        # Create a body inside the world
        self.body = Body(world)
        M = Mass()
        M.setSphere(2500.0, size)
        M.mass = mass
        self.body.setMass(M)
        self.body.setPosition(pos)
        # draw body
        self.rbody = sphere(pos=pos, radius=size)

    def getPosition(self):
        return self.body.getPosition()

    def getLinearVel(self):
        return self.body.getLinearVel()

    def addForce(self, force):
        return self.body.addForce(force)

    def static(self):
        self.sj = BallJoint(self.world)
        self.sj.attach(self.body, environment)
        self.sj.setAnchor(self.pos)

    def joint(self, body):
        self.j = BallJoint(self.world)
        self.j.attach(self.body, body.body)
        self.j.setAnchor( body.pos )

    def draw(self):
        self.rbody.pos = self.getPosition()


def yn_choice(message, default='y'):
    choices = 'Y/n' if default.lower() in ('y', 'yes') else 'y/N'
    choice = raw_input("%s (%s) " % (message, choices))
    values = ('y', 'yes', '') if default == 'y' else ('y', 'yes')
    return choice.strip().lower() in values

def set_wii_remote():
    if yn_choice('Enable Wii remote?', 'n'):
        # import WiiMote
        import cwiid
        print 'Press button 1 + 2 on your Wii Remote...'
        wm = cwiid.Wiimote()
        wm.rpt_mode = cwiid.RPT_ACC | cwiid.RPT_NUNCHUK | cwiid.RPT_BTN
    #    wm.rpt_mode = cwiid.RPT_BTN
        print 'Wii Remote connected...'
        return wm
    else:
        return None


if __name__ == '__main__':
    wm = set_wii_remote()
    if wm:
        import cwiid
        print 'Press A button to exit'
        while not wm.state['buttons'] & cwiid.BTN_A:
            print wm.state
        wm.close()
