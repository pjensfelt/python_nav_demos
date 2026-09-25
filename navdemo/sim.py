"""Robot simulation and the path-following controllers.

The robot is a unicycle, (x, y, a) with translation speed v and rotation
speed w. The controller only gets to choose the *reference* speeds vRef and
wRef; the actual speeds follow them under finite accelerations, which is
what makes a short lookahead or a slow control loop show up as overshoot
and oscillation instead of perfect tracking.
"""

import numpy as np

# Physics runs at this fixed step, independent of the control period, so
# that changing the control period changes *only* how often the controller
# gets to act -- the MATLAB demo ran Nsim = 10 physics steps per control
# step instead, which is the same thing at its fixed dt = 0.01 s.
PHYSICS_DT = 0.001


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Robot:
    def __init__(self, x=0.0, y=0.0, a=0.0):
        self.start_pose = (x, y, a)
        self.reset()

    def reset(self):
        self.x, self.y, self.a = self.start_pose
        self.v = 0.0
        self.w = 0.0

    @property
    def pose(self):
        return self.x, self.y, self.a

    def physics_step(self, vRef, wRef, accV, accW, dt=PHYSICS_DT):
        """One Euler step. The speeds move toward their references by at
        most acc*dt, then the pose integrates the new speeds.

        (pure_pursuit.m had `a = a + w*dt` here, with the control period
        dt instead of the physics step ddt, so the heading turned Nsim = 10
        times faster than w said it did -- in effect 10x the gain and 10x
        the rotational acceleration limit. That is fixed here, which is why
        the defaults are kP = 10 and acc_w = 3600 deg/s^2 instead of 1 and
        360: those are what the MATLAB demo was really running with. Put
        acc_w back to 360 with kP = 10 to see the limit cycle that a gain
        too high for the available acceleration produces.)
        """
        self.v += np.clip(vRef - self.v, -accV * dt, accV * dt)
        self.w += np.clip(wRef - self.w, -accW * dt, accW * dt)
        self.x += self.v * np.cos(self.a) * dt
        self.y += self.v * np.sin(self.a) * dt
        self.a = wrap_angle(self.a + self.w * dt)


def speed_reduction(aErr, sigma):
    """The further the target is from straight ahead, the slower we go:
    a Gaussian in the heading error, 1 when facing the target. sigma = inf
    turns it off (always full speed)."""
    if np.isinf(sigma):
        return 1.0
    return float(np.exp(-0.5 * (aErr / sigma) ** 2))


def control_heading(pose, target, vMax, kP, sigma):
    """The MATLAB demo's controller (calcCtrl in pure_pursuit.m): turn toward
    the lookahead point with a proportional gain on the heading error, and
    slow down while the heading error is large.

        aErr = atan2(yT - y, xT - x) - a
        wRef = kP * aErr
        vRef = vMax * exp(-aErr^2 / (2 sigma^2))
    """
    x, y, a = pose
    aErr = wrap_angle(np.arctan2(target[1] - y, target[0] - x) - a)
    vRef = vMax * speed_reduction(aErr, sigma)
    wRef = kP * aErr
    return vRef, wRef, aErr


# Turn in place (pure pursuit only): above this angle to the target the
# robot stops and turns on the spot, at TURN_GAIN * alpha capped at
# TURN_RATE_MAX, until the target is back in front.
TURN_IN_PLACE_ANGLE = np.deg2rad(90)
TURN_GAIN = 2.0                    # 1/s
TURN_RATE_MAX = np.deg2rad(180)    # rad/s


def control_pure_pursuit(pose, target, vMax, sigma, turn_in_place=False):
    """Classic (geometric) pure pursuit: drive the circular arc that leaves
    the robot tangent to its current heading and passes through the
    lookahead point.

    With alpha the angle to the target in the robot frame and Ld the
    distance to it, that arc has curvature

        kappa = 2 sin(alpha) / Ld

    and w = v * kappa follows it at whatever speed v we choose. There is no
    gain to tune here -- the lookahead distance *is* the gain.

    The catch: w is proportional to v. When the target is behind the robot,
    the slow-down makes v almost zero, so w is almost zero too, and the
    robot crawls along a huge arc instead of turning round. With
    `turn_in_place` it stops and turns on the spot instead whenever the
    target is more than TURN_IN_PLACE_ANGLE off -- the usual practical fix.
    (heading-P doesn't need this: its w = kP * aErr doesn't depend on v.)
    """
    x, y, a = pose
    dx, dy = target[0] - x, target[1] - y
    Ld = max(np.hypot(dx, dy), 1e-9)
    alpha = wrap_angle(np.arctan2(dy, dx) - a)
    if turn_in_place and abs(alpha) > TURN_IN_PLACE_ANGLE:
        return 0.0, float(np.clip(TURN_GAIN * alpha, -TURN_RATE_MAX, TURN_RATE_MAX)), alpha
    kappa = 2.0 * np.sin(alpha) / Ld
    vRef = vMax * speed_reduction(alpha, sigma)
    wRef = vRef * kappa
    return vRef, wRef, alpha


CONTROL_LAWS = ("heading-P", "pure pursuit")


class Follower:
    """Glues path, robot and controller together, and keeps the logs that
    the strip charts (and the MATLAB demo's figures 2 and 3) show."""

    # How far ahead of the previous closest point to look for the new one
    # (see Path.closest). Generous, so a disturbance doesn't lose the path,
    # but short enough not to jump to a later part of a self-crossing path.
    SEARCH_WINDOW = 1.0  # m, on top of the lookahead distance

    LOG_DT = 0.01  # s between logged samples

    def __init__(self, path, robot):
        self.path = path
        self.robot = robot
        self.reset()

    def reset(self):
        self.robot.reset()
        self.t = 0.0
        self.s = 0.0
        self.vRef = self.wRef = 0.0
        self.aErr = 0.0
        self.target = self.path.point_at(0.0)
        self.closest_pt = self.path.start.copy()
        self.e = 0.0
        self.done = False
        self.t_done = None
        self._next_ctrl = 0.0
        self._next_log = 0.0
        self.log_t, self.log_v, self.log_w, self.log_e = [], [], [], []
        self.trail = [(self.robot.x, self.robot.y)]

    def set_path(self, path):
        self.path = path
        self.reset()

    @property
    def max_abs_e(self):
        return max((abs(e) for e in self.log_e), default=0.0)

    def control(self, p):
        """One controller update: find where we are on the path, pick the
        lookahead point, compute vRef/wRef."""
        x, y, _ = self.robot.pose
        xc, yc, s, e = self.path.closest(x, y, s_min=self.s,
                                         window=p["lookahead"] + self.SEARCH_WINDOW)
        self.s, self.e = s, e
        self.closest_pt = np.array([xc, yc])

        # Stop once the closest point is the end of the path -- the MATLAB
        # demo's "index == length(Xp)" test.
        if not self.done and s >= self.path.length - 1e-6:
            self.done = True
            self.t_done = self.t
        if self.done:
            self.vRef = self.wRef = 0.0
            return

        self.target = self.path.point_at(s + p["lookahead"])
        if p["law"] == "pure pursuit":
            self.vRef, self.wRef, self.aErr = control_pure_pursuit(
                self.robot.pose, self.target, p["vmax"], p["sigma"],
                p.get("turn_in_place", False))
        else:
            self.vRef, self.wRef, self.aErr = control_heading(
                self.robot.pose, self.target, p["vmax"], p["kp"], p["sigma"])
        self.brake_for_goal(p)

    # Braking deceleration as a fraction of acc_v: below the real limit, so
    # the robot has margin to actually follow the profile.
    BRAKE_FRACTION = 0.5

    def brake_for_goal(self, p):
        """Brake *into* the goal instead of at it.

        Only asking for v = 0 once the goal is reached (as the MATLAB demo
        did) overshoots it by v^2 / (2 acc_v) -- 0.25 m at 1 m/s and
        2 m/s^2. Instead the speed is capped by a braking profile,

            v <= sqrt(2 * a_brake * d)      d = distance left along the path,

        the speed from which braking at a_brake stops exactly at the end.
        The robot decelerates uniformly over the last v^2 / (2 a_brake)
        metres and comes to rest on the goal.
        """
        if self.vRef <= 0 or np.isinf(p["accv"]):
            return   # with unlimited deceleration there is nothing to plan for
        d = max(self.path.length - self.s, 0.0)
        v_cap = np.sqrt(2 * self.BRAKE_FRACTION * p["accv"] * d)
        if v_cap < self.vRef:
            if p["law"] == "pure pursuit":
                self.wRef *= v_cap / self.vRef    # the same arc, just slower
            self.vRef = v_cap

    def advance(self, duration, p):
        """Simulate `duration` seconds: physics every PHYSICS_DT, the
        controller every p['ctrl_dt'], and a log sample every LOG_DT."""
        n = max(int(round(duration / PHYSICS_DT)), 1)
        for _ in range(n):
            if self.t >= self._next_ctrl - 1e-12:
                self.control(p)
                self._next_ctrl += p["ctrl_dt"]
            self.robot.physics_step(self.vRef, self.wRef, p["accv"], p["accw"])
            self.t += PHYSICS_DT
            if self.t >= self._next_log - 1e-12:
                self._next_log += self.LOG_DT
                self.log_t.append(self.t)
                self.log_v.append(self.robot.v)
                self.log_w.append(self.robot.w)
                self.log_e.append(self.e)
                self.trail.append((self.robot.x, self.robot.y))

    def disturb(self, rng, dist=0.3, angle=np.deg2rad(30)):
        """Kick the robot sideways and twist it -- like 'd' in the
        localization demos, but here it's the controller that has to cope."""
        r = self.robot
        side = rng.choice([-1.0, 1.0])
        r.x += side * dist * -np.sin(r.a)
        r.y += side * dist * np.cos(r.a)
        r.a = wrap_angle(r.a + rng.uniform(-angle, angle))
        self.trail.append((np.nan, np.nan))  # break the trail line at the jump
        self.trail.append((r.x, r.y))
