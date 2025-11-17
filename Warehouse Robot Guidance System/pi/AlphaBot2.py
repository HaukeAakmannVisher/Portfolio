import time
import RPi.GPIO as GPIO

class AlphaBot2(object):
    """
    AlphaBot2 motor driver with voltage-based compensation:
      - Left motor voltage model (linear): V_L(p)  = 0.078*p + 0.665
      - Right motor voltage model (4th):  V_R(p)  = -2.57e-7*p^4 + 7.23e-5*p^3
                                           -7.58e-3*p^2 + 0.37*p - 0.049
      - Compensation: find p_right so V_R(p_right) ~= V_L(p_left), then *0.97
      - Forward(): applies 0.5 s torque boost (~+50%) on right motor, then uses
                   compensated PWM.
    NOTE: In this file, channel A (AIN*, ENA/PWMA) is the RIGHT motor,
          channel B (BIN*, ENB/PWMB) is the LEFT motor — matching your code.
    """

    def __init__(self, ain1=13, ain2=12, ena=6, bin1=21, bin2=20, enb=26):
        # Pin map
        self.AIN1 = ain1   # right dir
        self.AIN2 = ain2   # right dir
        self.BIN1 = bin1   # left dir
        self.BIN2 = bin2   # left dir
        self.ENA  = ena    # right PWM (PWMA)
        self.ENB  = enb    # left  PWM (PWMB)

        # Baseline duties (0..100)
        self.PA  = 0       # right baseline
        self.PB  = 90      # left baseline (your original default)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.ENA,  GPIO.OUT)
        GPIO.setup(self.ENB,  GPIO.OUT)

        # 500 Hz like your original
        self.PWMA = GPIO.PWM(self.ENA, 500)   # right
        self.PWMB = GPIO.PWM(self.ENB, 500)   # left
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)
        self.stop()

    # -------------------- Public motions --------------------

    def forward(self):
        # Directions for forward
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

        # Left motor uses its baseline PB directly
        left_pwm = _clamp_pwm(self.PB)
        self.PWMB.ChangeDutyCycle(left_pwm)

        # Right motor: compute compensated PWM to match left voltage
        right_pwm_comp = self._compensate_right_pwm(left_pwm)

        # Torque boost on right for 0.5 s (~+50%, capped at 100)
        boost = min(100, int(1.5 * right_pwm_comp))
        self.PWMA.ChangeDutyCycle(boost)
        time.sleep(0.5)
        self.PWMA.ChangeDutyCycle(right_pwm_comp)

    def backward(self):
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)

        left_pwm = _clamp_pwm(self.PB)
        self.PWMB.ChangeDutyCycle(left_pwm)
        self.PWMA.ChangeDutyCycle(self._compensate_right_pwm(left_pwm))

    def left(self):
        # on-spot: left wheel backward, right forward (same duty)
        duty = 30
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)
        self.PWMA.ChangeDutyCycle(duty)
        self.PWMB.ChangeDutyCycle(duty)

    def right(self):
        # on-spot: left forward, right backward (same duty)
        duty = 30
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)
        self.PWMA.ChangeDutyCycle(duty)
        self.PWMB.ChangeDutyCycle(duty)

    def stop(self):
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.LOW)

    # -------------------- Setters / low-level --------------------

    def setPWMA(self, value):
        self.PA = _clamp_pwm(value)
        self.PWMA.ChangeDutyCycle(self.PA)

    def setPWMB(self, value):
        self.PB = _clamp_pwm(value)
        self.PWMB.ChangeDutyCycle(self.PB)

    def setMotor(self, left, right):
        """
        Differential command (left/right in -100..100).
        When both sides move the same direction, the RIGHT duty is compensated
        to match LEFT voltage.
        """
        # Right (A)
        if (right >= 0) and (right <= 100):
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            right_cmd = right
        elif (right < 0) and (right >= -100):
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            right_cmd = -right
        else:
            right_cmd = 0

        # Left (B)
        if (left >= 0) and (left <= 100):
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            left_cmd = left
        elif (left < 0) and (left >= -100):
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
            left_cmd = -left
        else:
            left_cmd = 0

        # Compensation if both intend same direction
        if (right >= 0 and left >= 0) or (right <= 0 and left <= 0):
            right_cmd = self._compensate_right_pwm(left_cmd)

        self.PWMA.ChangeDutyCycle(_clamp_pwm(right_cmd))
        self.PWMB.ChangeDutyCycle(_clamp_pwm(left_cmd))

    # -------------------- Compensation logic --------------------

    @staticmethod
    def _left_voltage(p):
        # V_L(p) = 0.078*p + 0.665
        return 0.078 * p + 0.665

    @staticmethod
    def _right_voltage(p):
        # V_R(p) = -2.57e-7*p^4 + 7.23e-5*p^3 - 7.58e-3*p^2 + 0.37*p - 0.049
        return (
            (-2.57e-7) * (p ** 4)
            + (7.23e-5) * (p ** 3)
            - (7.58e-3) * (p ** 2)
            + 0.37 * p
            - 0.049
        )

    @classmethod
    def _solve_right_pwm_for_voltage(cls, v_star):
        """
        Solve V_R(p) = v_star for p in [0,100] via binary search.
        Keeps it SciPy-free and fast on the Pi.
        """
        lo, hi = 0.0, 100.0
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            if cls._right_voltage(mid) < v_star:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def _compensate_right_pwm(self, p_left):
        """
        Given LEFT PWM, return RIGHT PWM so voltages match,
        then apply ~3% reduction (tuning).
        """
        p_left = _clamp_pwm(p_left)
        v_star = self._left_voltage(p_left)
        p_right = self._solve_right_pwm_for_voltage(v_star)
        p_right *= 0.97  # small reduction used in testing
        return _clamp_pwm(int(round(p_right)))


# -------------------- helpers --------------------
def _clamp_pwm(x):
    try:
        x = int(x)
    except Exception:
        x = 0
    if x < 0:
        return 0
    if x > 100:
        return 100
    return x


