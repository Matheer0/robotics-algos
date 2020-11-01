# Alex Li
# 260678992

class PIDController:
    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 3900.0
        self.Ki = 1000.0
        self.Kd = 3500.0
        self.bias = 0.0
        # pygame settings
        self.target_fps = 60.0
        self.dt = 1.0 / self.target_fps
        # pid intermediate value settings
        self.cumulated_error = 0.0
        self.windup_limit = 20.0
        self.previous_error = 0.0
        return

    def reset(self):
        self.Kp = 3900.0
        self.Ki = 1000.0
        self.Kd = 3500.0
        # pid intermediate value settings
        self.cumulated_error = 0.0
        self.previous_error = 0.0
        return

#TODO: Complete your PID control within this function. At the moment, it holds
#      only the bias. Your final solution must use the error between the 
#      target_pos and the ball position, plus the PID gains. You cannot
#      use the bias in your final answer. 
    def get_fan_rpm(self, vertical_ball_position):

        # Part I. current cross-track error for P-Ctrl: e(t)
        cte = self.target_pos - vertical_ball_position

        # Part II. integration of error for I-Ctrl
        self.cumulated_error += cte * self.dt
        # for I-Ctrl, we also need to consider integrator windup
        if self.cumulated_error > self.windup_limit:
            self.cumulated_error = self.windup_limit
        elif -self.cumulated_error > self.windup_limit:
            self.cumulated_error = -self.windup_limit

        # Part III.rate of change in error for D-Ctrl: d[e(t)]/dt
        change_in_error = cte - self.previous_error
        rate_of_change = change_in_error / self.dt

        # calculate PID control
        output = (self.Kp * cte) + (self.Ki * self.cumulated_error) + (self.Kd * rate_of_change)
        # update error value for the next iteration
        self.previous_error = cte

        return output
