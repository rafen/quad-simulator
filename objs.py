from ode import (
    Body, Mass, BallJoint, environment,
    GeomSphere
)
from visual import sphere


class Sphere(object):
    def __init__(self, world, space, pos, size, mass, color=(0.7, 0.7, 0.7)):
        self.world = world
        self.pos = pos
        # Create a body inside the world
        self.body = Body(world)
        M = Mass()
        M.setSphere(2500, size)
        M.mass = mass
        self.body.setMass(M)
        self.body.setPosition(pos)
        if space:
            # setect colisions
            self.geom = GeomSphere(space, size)
            self.geom.setBody(self.body)
        # draw body
        self.rbody = sphere(pos=pos, radius=size, color=color)

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
        self.pos = self.rbody.pos = self.getPosition()
