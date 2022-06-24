import matplotlib.pyplot as plt
import numpy as np

from pycatenary import cable


def iterative_min_max_catenary(f, A, B, L_max=50, eval_points=100):
    # the minimum distance is the line between the points
    L_min = np.linalg.norm(np.array(A) - np.array(B))

    # define properties of cable
    w = 1.036  # submerged weight
    EA = 560e3  # axial stiffness
    floor = False  # if True, contact is possible at the level of the anchor
    anchor = list(A)
    fairlead = list(B)
    if len(A) == 2:
        # adding a simulated z value
        anchor = [anchor[0], anchor[1], 0]
        fairlead = [fairlead[0], fairlead[1], 0]

    # add an epsilon to the l_min corresponding to the line, as this is not a catenary
    L_min = int(L_min) + 1
    if L_min > L_max:
        print("Warning! L_min {0} is bigger than L_max".format(L_min))
        return None, None

    min_distance, best_cat = np.inf, None
    for i in range(L_min, L_max):
        # print("\n Iter {0}".format(i))
        cat, cat_points = define_and_eval(i, w, EA, anchor, fairlead, floor, A, eval_points=eval_points)
        # evaluate on the same points of the catenary
        f_points = f(cat_points[:, 0])
        distance = np.abs(cat_points[:, -1] - f_points).max()

        if distance < min_distance:
            min_distance = distance
            best_cat = cat
    return best_cat, min_distance


def eval_cat(cat, L, eval_points=100, to_size_two=False):
    index = 2 if to_size_two else 3
    cat_eval_on = np.linspace(0, L, eval_points)
    return np.array([cat.s2xyz(c)[:index] for c in cat_eval_on])


def define_and_eval(L, w, EA, anchor, fairlead, floor, A, eval_points=100):
    # create cable instance
    cat = cable.MooringLine(L=L, w=w, EA=EA, anchor=anchor, fairlead=fairlead, floor=floor, nd=len(A))
    # compute calculations
    cat.computeSolution()
    return cat, eval_cat(cat, L, eval_points=eval_points, to_size_two=len(A) == 2)


def get_max_distance(f1, f2, A, B, eval_points=100):
    eval_on = np.linspace(A, B, eval_points)

    # f1_eval = np.array(map(f1, eval_on))
    f1_eval = f1(eval_on)
    f2_eval = f2(eval_on)

    return max(np.abs(f1_eval - f2_eval))


if __name__ == '__main__':

    import os
    import pandas as pd

    A_ = (0, 0)  # this should not be change. The rest of the parameters are set thinking on the origin of coords
    bx_min, bx_max = 5, 50
    dict_info = {"B": [], "distance": [], "parable_a": [], "parable_b": [], "cat_L": []}
    save_dir = "results"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    for i_ in range(300):
        print("Case: {0}".format(i_))
        bx = np.random.randint(bx_min, bx_max)
        max_by = int(np.sqrt(50**2 - bx**2))
        by = np.random.randint(0, max_by)

        par_a = np.random.rand()/bx
        b = (by - par_a*(bx**2))/bx

        p_ = lambda x: par_a*x**2 + b*x
        try:
            c_, d_ = min_max_catenary(p_, A_, (bx, by), max_iter=100)

            dict_info["B"].append((bx, by))
            dict_info["distance"].append(d_)
            dict_info["parable_a"].append(par_a)
            dict_info["parable_b"].append(b)
            dict_info["cat_L"].append(c_.L.sum())

            cat_eval_on = np.linspace(0, c_.L.sum(), 100)
            x, y = zip(*[c_.s2xyz(c)[:2] for c in cat_eval_on])
            plt.plot(x, y, label='catenary')

            eval_on_ = np.linspace(A_[0], bx, num=100)
            plt.plot(eval_on_, p_(eval_on_), label="parable")
            plt.scatter([A_[0], bx], [A_[1], by])
            plt.legend()
            f_name = os.path.join(save_dir, "imgs/{0}.png".format(i_))
            plt.savefig(f_name)
            plt.close()
        except Exception as e:
            print(e)

    df_ = pd.DataFrame.from_dict(dict_info)
    df_.to_csv(os.path.join(save_dir, "cat_search_data.csv"), index=False)

    # # # computations assume a maximum length of 50. Therefore, distance between A and B must comply with this.
    # p_ = lambda x: 0.9*x**2 + x
    # p_ = lambda x: 0.025861157939626265*x**2 + 0.2500264197508384*x
    # # B_ = (5, p_(5))
    # B_ = (29, 29)
    # A_ = (0, 0)
    #
    # c_, d_ = min_max_catenary(p_, A_, B_, max_iter=2)
    # print("\n === Distance: {0}, Length: {1}".format(d_, c_.L.sum()))
    #
    # print(c_.anchor, c_.fairlead)
    # cat_eval_on = np.linspace(0, c_.L.sum(), 100)
    # x, y = zip(*[c_.s2xyz(c)[:2] for c in cat_eval_on])
    # plt.plot(x, y, label='catenary')
    #
    # eval_on_ = np.linspace(A_[0], B_[0], num=100)
    # plt.plot(eval_on_, p_(eval_on_), label="parable")
    # plt.scatter([A_[0], B_[0]], [A_[1], B_[1]])
    # plt.legend()
    # plt.show()
    # plt.close()
