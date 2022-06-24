import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull, convex_hull_plot_2d


class TwoD_Obs:

    def __init__(self, points):
        self.points = points

    def convex_AB_hull(self, A, B, plot=False):
        min_x = min(A[0], B[0])
        max_x = max(A[0], B[0])
        p = [tuple(i) for i in self.points if min_x <= i[0] <= max_x] + [tuple(A), tuple(B)]
        hull = ConvexHull(p)
        if plot:
            convex_hull_plot_2d(hull)
            plt.show()
            plt.close()
        # there is room for improvements if we take only the "lower" part of the hull connecting A and B
        v = [p[i] for i in hull.vertices if p[i] != A and p[i] != B]
        if len(v) != len(hull.vertices) - 2:
            print("warning: strange behaviour on hull computation")
        return v

    def intersects(self, func):
        exist_lower, exist_up = False, False
        for x, y in self.points:
            fy = func(x)
            if fy < y:
                exist_up = True
            if fy > y:
                exist_lower = True
            if exist_lower and exist_up:
                return True
        return False

    def plot(self, ax=None, close=False, ret_scatter=False, **scatter_kwargs):
        if ax is None:
            f, ax = plt.subplots()
        x, y = zip(*self.points)
        scat = ax.scatter(x, y, **scatter_kwargs)

        if close:
            plt.close()
        if ret_scatter:
            return scat
        return ax


def random_2d_obstacle(start_point, n_points, spacing=0.1):
    area = [start_point]

    for n in range(n_points):
        anchor_point = area[np.random.randint(0, len(area))]
        new_p = []
        for v in anchor_point:
            n_value = (v + spacing) if np.random.rand() < 0.5 else v
            new_p.append(n_value)
        area.append(new_p)
    return np.unique(area, axis=0)


def square_block(start_point, width, spacing=0.1):
    n = int(width/spacing)
    x = np.linspace(start_point[0], start_point[0] + width, num=n)
    y = np.linspace(start_point[1], start_point[1] + width, num=n)

    grid = np.meshgrid(x, y)

    # stack values to have them like two D coords. Then flat the first dimensions
    return np.stack(grid, axis=-1).reshape(-1, 2)


if __name__ == '__main__':
    point_A_ = 1, 1
    point_B_ = 5, 8

    o1_ = TwoD_Obs(square_block((3, 3.5), 1.2))
    o2_ = TwoD_Obs(random_2d_obstacle((1.8, 2.8), 100))
    o3_ = TwoD_Obs(random_2d_obstacle((0.8, 3), 100))

    new_artificial_obs = TwoD_Obs(list(o1_.points) + list(o2_.points) + list(o3_.points))

    hv_ = new_artificial_obs.convex_AB_hull(point_A_, point_B_, plot=True)

    m_ = (point_A_[1] - point_B_[1])/(point_A_[0] - point_B_[0])
    n_ = point_A_[1] - m_*point_A_[0]
    ab_equation = lambda x: m_*x + n_

    print("o1: ", o1_.intersects(ab_equation))
    print("o2: ", o2_.intersects(ab_equation))
    print("o3: ", o3_.intersects(ab_equation))
    print("artif: ", new_artificial_obs.intersects(ab_equation))
