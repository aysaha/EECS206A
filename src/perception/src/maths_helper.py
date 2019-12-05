from math import atan2, sin, cos, sqrt, pi
# Note for myself: switch to complex class in maths package next time after the second custom definition of a complex operation

def barycenter(points, weights=None):
    if weights is not None:
        assert len(points) == len(weights), "Weights have wrong size"

        W = sum(weights)
        return [sum([weight*point[0] for point, weight in zip(points, weights)])/W, sum([weight*point[1] for point, weight in zip(points, weights)])/W]

    W = len(points)
    return (sum([point[0] for point in points])/W, sum([point[1] for point in points])/W)


def point2tuple(point):
    return (point.x, point.y)


def diff(p1, p2):
    return (p1[0] - p2[0], p1[1] - p2[1])


def add(p1, p2):
    return (p1[0] + p2[0], p1[1] + p2[1])


def norm(point):
    return sqrt(point[0]**2 + point[1]**2)


def phase(point):
    return atan2(point[1], point[0])


def div(p1, p2): # Return polar coordinates
    new_phase = phase(p1) - phase(p2)
    if new_phase <= -pi:
        new_phase += 2*pi
    if new_phase > pi:
        new_phase -= 2*pi
    return (norm(p1)/norm(p2), new_phase)


def mul(p1, p2_mod, p2_phase):
    x, y = (p1[0]*p2_mod, p1[1]*p2_mod)

    return (x * cos(p2_phase) - y * sin(p2_phase), x * sin(p2_phase) + y * cos(p2_phase))
