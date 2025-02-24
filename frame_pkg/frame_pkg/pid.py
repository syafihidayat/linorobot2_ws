import math

class PID:
    class Params:
        def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0):
            self.kp = kp
            self.ki = ki
            self.kd = kd


    class Error:
        def __init__(self):
            self.proportional = 0.0
            self.integral = 0.0
            self.derivative = 0.0
            self.previous = 0.0

    class Velocity:
        def __init__(self):
            self.linear = 0.0
            self.angular = 0.0
            self.angular_filter = 0.0

    class Output:
        def __init__(self):
            self.heading = 0.0
            self.linear = 0.0

    def __init__(self):
        self.headingParams = self.Params()
        self.baseParams = self.Params()
        self.err = self.Error()
        self.vel = self.Velocity()
        self.output = self.Output()
        self.u = 0.0
        self.uT = 0.0

    def set_base_params(self,kp ,ki ,kd):
        self.baseParams.kp = kp
        self.baseParams.ki = ki
        self.baseParams.kd = kd

    def set_heading_params(self, kp,ki,kd):
        self.headingParams.kp = kp
        self.headingParams.ki = ki
        self.headingParams.kd = kd

    def control_base(self, error,speed):
        
        if error > 180:
            error -= 360
        elif error < -180:
            error +=360

        else:
            
            self.err.proportional = error
        
        self.err.integral += self.err.proportional
        self.derivetive = (self.err.proportional - self.err.previous)
        self.previous = self.err.proportional

        self.uT = self.headingParams.kp * self.err.proportional +  self.headingParams.ki * self.err.integral + self.headingParams.kd * self.err.derivative

        return max(-speed , min(self.uT, speed))

    def controlling(self,error,speed):
        self.err.proportional = error
        self.err.integral += self.err.proportional
        self.err_derivative = self.err.proportional - self.err.previous
        

        self.err.previous = self.err.proportional

        self.u = self.baseParams.kp * self.err.proportional + self.baseParams.ki * self.err.integral + self.baseParams.kd * self.err.derivative

        return max(-speed, min(self.u, speed))


    def get_error_p(self):
        return self.err.proportional

    def get_error_i(self):
        return self.err.integral

    def get_error_d(self):
        return self.err.derivative

    def get_heading_output(self):
        return self.output.heading

    def get_linear_output(self):
        return self.output.linear

    def get_u(self):
        return self.u
    
    def get_Control(self):
        return self.uT

    
