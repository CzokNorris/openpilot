from selfdrive.controls.lib.drive_helpers import rate_limit
import numpy as np
from common.numpy_fast import clip, interp

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class PIController():
  def __init__(self, k_p, k_i, k_d = ([0], [0]), k_f=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d  # derivative gain
    self.k_f = k_f  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.d_rate = rate
    self.sat_limit = sat_limit
    self.convert = convert

    self.smooth_error = 0
    self.angle_time = 10

    self.smooth_derivative = 0
    self.d_time = 10

    self.d_log = [0 for _ in range(self.d_time)]
    self.d_log_p = 0
    
    self.l_p, self.l_i, self.l_d, self.l_f = (0,0,0,0)
    self.summ = 0
    self.reset()

  def read_tune(self):
    with open("/data/openpilot/tune.txt","r") as f:
      ls = []
      try:
        for line in f:
          ls.append(float(line))
        self.l_p = ls[0]
        self.l_i = ls[1]
        self.l_d = ls[2]
        self.l_f = ls[3]
        if self.l_f+self.l_d+self.l_i+self.l_p != self.summ:
          print("new tune pidf:", self.l_p, self.l_i, self.l_d, self.l_f)
        self.summ = self.l_f+self.l_d+self.l_i+self.l_p
      except Exception:
        self.summ = 0
        print("_____________________________________________\n FILE READING ERROR\n'''''''''''''''''''''''''''''''''''''''''''''''''''")


  @property
  def k_p(self):
    if self.summ != 0:
      return self.l_p
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    if self.summ != 0:
      return self.l_i
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    if self.summ != 0:
      return self.l_d
    return interp(self.speed, self._k_d[0], self._k_d[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0

    self.smooth_angle= 0
    self.smooth_derivative = 0
    self.d_log = [0 for _ in range(self.d_time)]

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.read_tune()
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p

    if self.summ != 0:
      self.k_f = self.l_f
    self.f = feedforward * self.k_f

    new_smooth_error = (1 - 1/self.angle_time) * self.smooth_error + error / self.angle_time
    d_error = (new_smooth_error - self.smooth_angle) * self.d_rate
    self.smooth_error = new_smooth_error

    self.d_log[self.d_log_p] = d_error
    derivative = sum(self.d_log) / len(self.d_log)

    self.d = derivative * self.k_d

    self.d_log_p +=1
    if self.d_log_p>=self.d_time:
      self.d_log_p=0

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i + self.d

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i + self.d
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    self.saturated = self._check_saturation(control, check_saturation, error)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
