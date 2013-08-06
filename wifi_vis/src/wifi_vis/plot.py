def plot_robot_path_xytheta(ax, xytheta_path):
    a = ax.plot(xytheta_path[:,0], xytheta_path[:,1])
    ax.set_aspect('equal')
    return a

def plot_signal_strength_xytheta(ax, xytheta, signal, smin, smax, label):
    return ax.scatter(xytheta[:,0], xytheta[:,1], s=200, marker='*', cmap=plt.cm.jet,
                      vmin=smin, vmax=smax, c=signal, label=label)
