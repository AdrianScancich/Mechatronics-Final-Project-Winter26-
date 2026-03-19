from task_share import Share
from utime      import ticks_us, ticks_diff
import micropython
from math import sin, cos, atan2, sqrt, pi

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

L        = 150.0   # Lookahead distance (mm).
WB       = 141.0   # wheelbase mm
VMAX     = 190.0   # wheel speed cap mm/s
NEAR_TOL = 25.0    # distance to final waypoint that counts as arrival mm

PATH_1 = ( #200mm-radius right turn, then hard right and straight into garage
    (   0.0,    0.0),
    (  34.7,   -3.0),
    (  68.4,  -12.1),
    ( 100.0,  -26.8),
    ( 128.6,  -46.8),
    ( 153.2,  -71.4),
    ( 173.2, -100.0),
    ( 187.9, -131.6),
    ( 197.0, -165.3),
    ( 200.0, -200.0),
    ( 200.0, -260.0),
    ( 200.0, -280.0),
    ( 150.0, -340.0),
    ( 100.0, -340.0),
    (  50.0, -340.0),
    (   0.0, -340.0),
    ( -50.0, -340.0),
    (-100.0, -340.0),
    (-150.0, -340.0),
    (-200.0, -340.0),
    (-250.0, -340.0),
    (-300.0, -340.0),
)

PATH_2 = ( #200mm-radius right turn, then straight
    (   -0.0,    0.0),
    (  -52.1,    3.8),
    ( -102.6,   15.1),
    ( -150.0,   33.5),
    ( -192.9,   58.5),
    ( -229.8,   89.2),
    ( -259.8,  125.0),
    ( -281.9,  164.5),
    ( -295.5,  206.6),
    ( -300.0,  250.0),
    ( -300.0,  300.0),
    ( -300.0,  350.0),
    ( -300.0,  400.0),
    ( -300.0,  450.0),
)

PATHS = (None, PATH_1, PATH_2) 

def _clamp(x, lo, hi):
    if x < lo: return lo
    if x > hi: return hi
    return x

class task_trajectory_calculator:
    '''
    Pure-pursuit trajectory follower.
    '''

    def __init__(self, trajectoryGo: Share,
                 X1_psi: Share, X_global: Share, Y_global: Share,
                 trajDataset: Share, RomiVelocity: Share,
                 leftVelocity: Share, rightVelocity: Share):

        self._state         = S0_INIT
        self._goFlag        = trajectoryGo
        self._X1_psi        = X1_psi
        self._X_global      = X_global
        self._Y_global      = Y_global
        self._trajDataset   = trajDataset
        self._RomiVelocity  = RomiVelocity
        self._leftVelocity  = leftVelocity
        self._rightVelocity = rightVelocity

        self._path  = None
        self._seg   = 0
        self._xoff  = 0.0
        self._yoff  = 0.0
        self._n     = 0

        print("Trajectory Calculator Task object instantiated")

    def _lookahead(self, xm, ym):
        '''
        Find the lookahead point on the path.
        '''
        path = self._path
        n    = self._n

        for i in range(self._seg, n - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]

            dx = bx - ax
            dy = by - ay

            fx = ax - xm
            fy = ay - ym

            a = dx*dx + dy*dy
            b = 2.0*(fx*dx + fy*dy)
            c = fx*fx + fy*fy - L*L

            disc = b*b - 4.0*a*c
            if disc < 0.0:
                continue

            sq = sqrt(disc)
            for t in ((-b + sq)/(2.0*a), (-b - sq)/(2.0*a)):
                if 0.0 <= t <= 1.0:
                    self._seg = i
                    return ax + t*dx, ay + t*dy

        self._seg = n - 1
        fx, fy = path[n - 1]
        return fx, fy

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                if self._goFlag.get():
                    idx = int(self._trajDataset.get())
                    if 1 <= idx < len(PATHS) and PATHS[idx] is not None:
                        self._path = PATHS[idx]
                    else:
                        self._path = PATH_1

                    self._n    = len(self._path)
                    self._seg  = 0

                    self._xoff = self._X_global.get()
                    self._yoff = self._Y_global.get()

                    self._state = S2_RUN

            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._state = S1_WAIT
                else:
                    xg = self._X_global.get()
                    yg = self._Y_global.get()
                    pm = self._X1_psi.get()

                    xm = xg - self._xoff
                    ym = yg - self._yoff

                    fx, fy = self._path[self._n - 1]
                    dfx = fx - xm
                    dfy = fy - ym
                    if sqrt(dfx*dfx + dfy*dfy) < NEAR_TOL:
                        self._leftVelocity.put(0.0)
                        self._rightVelocity.put(0.0)
                        self._goFlag.put(False)
                        self._state = S1_WAIT
                        continue

                    lx, ly = self._lookahead(xm, ym)

                    dlx = lx - xm
                    dly = ly - ym

                    ey = -sin(pm)*dlx + cos(pm)*dly

                    kappa = 2.0 * ey / (L * L)

                    v = _clamp(self._RomiVelocity.get(), 0.0, VMAX)

                    vL = _clamp(v - kappa*(WB/2.0)*v, -VMAX, VMAX)
                    vR = _clamp(v + kappa*(WB/2.0)*v, -VMAX, VMAX)

                    self._leftVelocity.put(vL)
                    self._rightVelocity.put(vR)

            yield self._state
