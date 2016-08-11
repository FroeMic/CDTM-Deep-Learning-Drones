class PID:
    ''' PID Controller '''
    def __init__(self, Kp=0.25, Kd=0.25, Ki=0.05):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.last_error = 0

    def step(self, error):
        nextStep = self.Kp*error + self.Ki*(error+self.last_error) + self.Kd*(error-self.last_error)
        self.last_error = error
        return nextStep