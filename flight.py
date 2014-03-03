from pid import PID

class Flight(object):

    def __init__(self, quad, max_force=10, min_force=0.02):
        self.quad = quad
        # set max and min forces
        self.max_force = max_force
        self.min_force = min_force
        # commands
        self.command_pitch = 0
        self.command_roll = 0
        self.command_yaw = 0
        self.command_acceleration = 0

        # acceleration set points
        self.set_point_acceleration = 0
        self.acceleration = PID(4, 0.8, 2)
        self.acceleration.setPoint(self.set_point_acceleration)
        # Pitch set point
        self.set_point_pitch = 0
        self.pitch = PID(0.02, 0.001, 0.10)
        self.pitch.setPoint(self.set_point_pitch)
        # Roll set point
        self.set_point_roll = 0
        self.roll = PID(0.02, 0.001, 0.10)
        self.roll.setPoint(self.set_point_roll)
        # Yaw set point
        self.lock_yaw = True
        self.set_point_yaw = 90
        self.yaw = PID(0.2, 0.001, 1)
        self.yaw.setPoint(self.set_point_yaw)

    def update_forces(self, forces):
        # Update forces in min and max values
        for i, force in enumerate(forces):
            force = max(self.min_force, force)
            force = min(self.max_force, force)
            forces[i] = force
        # set values to quad
        self.quad.motors_force = forces

    def stabilize(self):
        forces = [0, 0, 0, 0]
        # acceleration
        aforce = self.acceleration.update(self.quad.vel[1])
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
        if self.lock_yaw:
            yforce = self.yaw.update(self.quad.yaw)
            forces[0] += yforce
            forces[1] += yforce
            forces[2] -= yforce
            forces[3] -= yforce

        self.update_forces(forces)

    def apply_commands(self):
        if self.lock_yaw and self.command_yaw:
            self.lock_yaw = False
        if not self.command_yaw and not self.lock_yaw:
            self.set_point_yaw = self.quad.yaw
            self.yaw.setPoint(self.set_point_yaw)
            self.lock_yaw = True


        if self.set_point_acceleration != self.command_acceleration:
            self.set_point_acceleration = -self.command_acceleration
            self.acceleration.setPoint(self.set_point_acceleration)

        forces = [
            self.quad.motors_force[0] + self.command_pitch + self.command_yaw,
            self.quad.motors_force[1] + -self.command_pitch + self.command_yaw,
            self.quad.motors_force[2] + self.command_roll - self.command_yaw,
            self.quad.motors_force[3] + -self.command_roll - self.command_yaw,
        ]
        self.update_forces(forces)

    def step(self):
        self.stabilize()
        self.apply_commands()
        self.quad.step()
