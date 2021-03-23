from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from FK import *
from skeleton_model import *

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def drawVector(fig, result, **kwargs):
    ms = kwargs.get('mutation_scale', 20)
    ars = kwargs.get('arrowstyle', '-')
    lc = kwargs.get('lineColor', 'k')
    pc = kwargs.get('projColor', 'k')
    pointEnable = kwargs.get('pointEnable', True)
    projOn = kwargs.get('proj', True)
    lineStyle = kwargs.get('lineStyle', '-')
    annotationString = kwargs.get('annotationString', '')
    lineWidth = kwargs.get('lineWidth', 1)

    num = 0
    joint_num = 13
    RLtoLL = 8
    for i in result:
        if num == joint_num:
            break
        if num == RLtoLL - 1:
            xs = [result[1][0, 3], result[RLtoLL][0, 3]]
            ys = [result[1][1, 3], result[RLtoLL][1, 3]]
            zs = [result[1][2, 3], result[RLtoLL][2, 3]]
            num = num + 1

            out = Arrow3D(xs, ys, zs, mutation_scale=ms, arrowstyle=ars, color=lc,
                          linestyle=lineStyle, linewidth=lineWidth)
            fig.add_artist(out)

            if pointEnable: fig.scatter(xs[1], ys[1], zs[1], color='k', s=50)

            if annotationString != '':
                fig.text(xs[1], ys[1], zs[1], annotationString, size=15, zorder=1, color='k')

        else:
            # [parent joint_xyz, child joint point_xyz]
            xs = [result[num][0, 3], result[num + 1][0, 3]]
            ys = [result[num][1, 3], result[num + 1][1, 3]]
            zs = [result[num][2, 3], result[num + 1][2, 3]]
            num = num + 1

            out = Arrow3D(xs, ys, zs, mutation_scale=ms, arrowstyle=ars, color=lc, linestyle=lineStyle,
                          linewidth=lineWidth)
            fig.add_artist(out)

            if pointEnable: fig.scatter(xs[1], ys[1], zs[1], color='k', s=50)

            if annotationString != '':
                fig.text(xs[1], ys[1], zs[1], annotationString, size=15, zorder=1, color='k')
