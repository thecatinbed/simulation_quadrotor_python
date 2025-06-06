import matplotlib.pyplot as plt

def draw3d(ctrl_buffer, measurement, coordinate, fig_name):
    """
    绘制三维轨迹图。
    图像将保存在名为images的文件夹中。
    :param ctrl_buffer: 包括期望位置的数据。
    :param measurement: 包括位置测量数据。
    :param coordinate: 坐标系选择；0: Z轴向下，1: Z轴向上。
    :param fig_name: 图片名称。
    """
    position = measurement[:, 0:3]
    position_des = ctrl_buffer[:, 0:3]
    fig = plt.figure()
    #ax = fig.gca(projection='3d')
    ax = fig.add_subplot(111, projection='3d')
    # ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    # ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    # ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    if coordinate == 0:
        x = position[:, 0]
        y = position[:, 1]
        z = position[:, 2]
    elif coordinate == 1:
        x = -position[:, 0]
        y = -position[:, 1]
        z = -position[:, 2]
    ax.set_xlabel('x(m)', fontsize=10, labelpad=5)
    ax.set_ylabel('y(m)', fontsize=10, labelpad=5)
    ax.set_zlabel('z(m)', fontsize=10, labelpad=5)
    ax.plot3D(x, y, z, 'black')
    z_des = position_des[:, 2]
    x_des = position_des[:, 0]
    y_des = position_des[:, 1]
    ax.tick_params(labelsize=8)
    ax.plot3D(x_des, y_des, z_des, 'r:')
    ax.legend(['actual', 'desire'], fontsize=8, loc='upper left', bbox_to_anchor=(0, 0.8))
    ax.view_init(20, -30)
    plt.savefig('./images/{}.png'.format(fig_name), format='png', dpi=900, bbox_inches='tight')
    plt.show()
