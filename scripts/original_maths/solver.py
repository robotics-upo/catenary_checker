import numpy as np
import matplotlib.animation as animation

from twoD_obstacles import TwoD_Obs


def get_parable_abc(p1, p2, p3):
    x1, y1 = p1[0], p1[-1]
    x2, y2 = p2[0], p2[-1]
    x3, y3 = p3[0], p3[-1]
    if x1 == x2 or x2 == x3 or x1 == x3:
        return None, None, None
    a = ((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/((x1-x3)*(x1-x2)*(x3-x2))
    b = (y1-y2-a*(x1**2 - x2**2))/(x1 - x2)
    c = y3 - a*(x3**2) - b*x3
    return a, b, c


def parable_approx(func, objects, A, B):
    """
    A method for solving the existence of a parable connecting A and B without intersecting the obstacles.
    :param func:
    :param objects: must be a list of two-dimensional obstacles
    :param A:
    :param B:
    :return: a tuple: a parable connecting A and B without intersecting the obstacles (if there is one),
        and the history of the processing
    """
    inter_o = [o for o in objects if o.intersects(func)]
    hist = [(func, objects)]

    if len(inter_o) == 0:
        return func, hist

    # create an artificial obstacle containing the points of the intersecting obstacles
    artificial_obs = TwoD_Obs(np.vstack([o.points for o in inter_o]))

    hull_vertex = artificial_obs.convex_AB_hull(A, B, plot=False)

    min_x, max_x = min(A[0], B[0]), max(A[0], B[0])
    # check stop conditions:
    #   some point of the convex hull has lower y than A
    #   some point has x value lower or equal than the minimum x and max-equal than the maximum x
    # A or B have already been removed from the hull vertices
    for h in hull_vertex:
        if h[1] < A[1] or h[0] <= min_x or h[0] >= max_x:
            return None, hist

    # compute the longest parable that pass through A, B and some point in the convex hull
    max_a, b, c = -np.inf, None, None
    for v in hull_vertex:
        na, nb, nc = get_parable_abc(A, v, B)
        if na is not None and na >= max_a:
            max_a, b, c = na, nb, nc
    # check existence
    if b is None:
        return None, hist
    # get the lower y for the parable
    lx = -b/(2*max_a)
    ly = max_a*(lx**2) + b*lx + c
    if ly < 0:
        print("info: Parable goes below zero")
        return None, hist
    # do a recursive call
    s, h = parable_approx(lambda x: max_a*(x**2) + b*x + c, objects, A, B)
    # extend the current history
    hist.extend(h)
    return s, hist


def plot_solver_history(hist, A, B, obstacles, x_eps=1):
    fig = plt.figure(figsize=(10, 7))
    min_x, max_x = min(A[0], B[0]), max(A[0], B[0])
    min_y, max_y = min(A[1], B[1]), max(A[1], B[1])
    ax = fig.add_subplot(autoscale_on=False, xlim=(min_x - x_eps, max_x + x_eps), ylim=(min_y - x_eps, max_y + x_eps))

    axes = [o.plot(ax=ax, ret_scatter=True, **{'c': 'blue'}) for o in obstacles]
    func_plot, = ax.plot([], [], '-', lw=1)
    ax.scatter([A[0], B[0]], [A[1], B[1]])

    def animate(hist_step):
        f, objects = hist[hist_step]
        for i, o in enumerate(objects):
            if o.intersects(f):
                axes[i].set_color('red')
            else:
                axes[i].set_color('blue')
        x = np.linspace(A[0], B[0])
        y = list(map(f, x))
        func_plot.set_data(x, y)

        artists = list(axes) + [func_plot]
        return artists

    dt = 0.4
    ani = animation.FuncAnimation(
        fig, animate, len(hist), interval=dt*1000, blit=True, repeat=False)

    # saving to mp4 using ffmpeg writer
    writer_video = animation.FFMpegWriter(fps=2)
    ani.save('decision_process.mp4', writer=writer_video)

    plt.show()

    plt.close()


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from twoD_obstacles import square_block, random_2d_obstacle
    from aprox_catenary import iterative_min_max_catenary

    point_A_ = 1.5, 2
    point_B_ = 8, 8

    m_ = (point_A_[1] - point_B_[1])/(point_A_[0] - point_B_[0])
    n_ = point_A_[1] - m_*point_A_[0]
    ab_equation = lambda x: m_*x + n_

    o1_ = TwoD_Obs(square_block((3, 3.5), 1.2))
    o2_ = TwoD_Obs(random_2d_obstacle((1.8, 2.8), 100))
    o3_ = TwoD_Obs(random_2d_obstacle((0.8, 3), 100))
    o4_ = TwoD_Obs(square_block((5, 4), 1))

    obs_ = [o1_, o2_, o3_, o4_]

    solution, history = parable_approx(ab_equation, obs_, point_A_, point_B_)

    plot_solver_history(history, point_A_, point_B_, obs_)

    if solution is not None:
        cat_, d_ = iterative_min_max_catenary(solution, point_A_, point_B_)

        art_obs_ = TwoD_Obs(list(o1_.points) + list(o2_.points) + list(o3_.points) + list(o4_.points))
        ax_ = art_obs_.plot()

        cat_eval_on = np.linspace(0, cat_.L.sum(), 100)
        x, y = zip(*[cat_.s2xyz(c)[:2] for c in cat_eval_on])
        ax_.plot(x, y, label='catenary')

        eval_on_ = np.linspace(point_A_[0], point_B_[0], num=100)
        ax_.plot(eval_on_, solution(eval_on_), label="parable")
        ax_.scatter([point_A_[0], point_B_[0]], [point_A_[1], point_B_[1]])
        plt.title("Distancia: {0}".format(d_))
        plt.legend()
        plt.show()
