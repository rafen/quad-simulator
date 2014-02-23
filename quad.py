from random import random
from ode import (
    World, Body, Mass, BallJoint, environment,
    GeomSphere, GeomPlane, GeomBox, Space,
    JointGroup, collide, ContactJoint
)
from visual import sphere, rate, vector, norm, degrees, box


class Quad(object):
    def __init__(self, world, space, pos=(0, 0, 0), size=1, motors_force=(0, 0, 0, 0)):
        self.world = world
        self.pos = pos
        x, y, z = pos
        self.size = size
        self.motors_force = motors_force
        self.motors_noise = [random()/100 for m in range(4)]

        # Create the quad inside the world

        # Motors
        motor_size = size / 10.
        motor_weight = 0.07
        arms_distance = size / 2.
        motor_front_color = (0, 0, 1) # color blue
        motor_roll_color = (0, 1, 0) # color green
        self.motors = [
            Sphere(world, space, (x, y, z-arms_distance), motor_size, motor_weight, motor_front_color),
            Sphere(world, space, (x, y, z+arms_distance), motor_size, motor_weight),
            Sphere(world, space, (x-arms_distance, y, z), motor_size, motor_weight, motor_roll_color),
            Sphere(world, space, (x+arms_distance, y, z), motor_size, motor_weight),
        ]

        # Center
        center_size = motor_size * 1.5
        center_weight = 0.5
        self.center = Sphere(world, space, (x, y, z), center_size, center_weight)

        # Stick parts together
        self.motors[0].joint(self.motors[1])
        self.motors[0].joint(self.center)
        self.motors[2].joint(self.motors[3])
        self.motors[2].joint(self.center)
        self.motors[0].joint(self.motors[2])
        self.motors[1].joint(self.motors[3])

        # set util vectors
        self._set_plane_vectors()

    def draw(self):
        # draw motors
        [m.draw() for m in self.motors]
        self.center.draw()

    def step(self):
        # update force
        self.update_motors()
        # update pos
        self.pos = self.center.getPosition()
        # draw quad
        self.draw()

    def _set_plane_vectors(self):
        cx, cy, cz = self.center.getPosition()
        x, y, z = self.motors[0].getPosition()
        self.v1 = norm(vector(x-cx, y-cy, z-cz))
        x, y, z = self.motors[2].getPosition()
        self.v2 = norm(vector(x-cx, y-cy, z-cz))
        # Calculate pitch and roll
        self.pitch = 90 - degrees(self.v1.diff_angle(vector(0, 1, 0)))
        self.roll = degrees(self.v2.diff_angle(vector(0, 1, 0))) - 90
        # calculate yaw, (this works if the quad is close to a horizontal position)
        if self.v1.z <= 0:
            self.yaw = degrees(self.v1.diff_angle(vector(1, 0, 0)))
        else:
            self.yaw = 360 - degrees(self.v1.diff_angle(vector(1, 0, 0)))

    def update_motors(self):
        # apply noise to motors to emulate reality
        m_noise = self.motors_noise

        # force of motor should be orthogonal to the quad
        self._set_plane_vectors()
        v = self.v1.cross(self.v2)
        for i, f in enumerate(self.motors_force):
            self.motors[i].addForce(v*(f+m_noise[i]))

        # Add rotational fore to motors
        rotational_factor = 0.1
        rforce = self.v2*(self.motors_force[0]+m_noise[0])*rotational_factor
        self.motors[0].addForce(rforce)
        rforce = -self.v2*(self.motors_force[1]+m_noise[1])*rotational_factor
        self.motors[1].addForce(rforce)
        rforce = self.v1*(self.motors_force[2]+m_noise[2])*rotational_factor
        self.motors[2].addForce(rforce)
        rforce = -self.v1*(self.motors_force[3]+m_noise[3])*rotational_factor
        self.motors[3].addForce(rforce)


    @property
    def vel(self):
        return self.center.getLinearVel()

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



######################################
# Simulation test

if __name__ == '__main__':
    world = World()
    world.setGravity((0, -9.81, 0))
    world.setERP(0.8)
    world.setCFM(1E-5)
    space = Space()
    contactgroup = JointGroup()


    # Set environment
    floor = GeomPlane(space, (0, 1, 0), -1)
    box(pos=(0, -1, 0), width=2, height=0.01, length=2)

    quad = Quad(world, space, pos=(0, -0.5, 0))

    # set initial state of motors
    # quad.motors_noise = (0, 0, 0, 0)
    quad.motors_force = (1.92, 1.92, 1.92, 1.92)


    # Collision callback
    def near_callback(args, geom1, geom2):
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

    # Do the simulation...
    total_time = 0.0
    dt = 0.04
    while True:
        rate(40)

        print "%1.2fsec: pos=(%6.3f, %6.3f, %6.3f) pry=(%6.3f, %6.3f, %6.3f)" % \
                (total_time, quad.pos[0], quad.pos[1], quad.pos[2],
                quad.pitch, quad.roll, quad.yaw)
        print "power=(%6.3f, %6.3f, %6.3f, %6.3f)" % (quad.motors_force[0], quad.motors_force[1], quad.motors_force[2], quad.motors_force[3])

        # Detect collisions and create contact joints
        space.collide((world, contactgroup), near_callback)
        quad.step()
        world.step(dt)
        contactgroup.empty()

        total_time+=dt
