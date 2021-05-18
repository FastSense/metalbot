import matplotlib.pyplot as plt
import numpy as np

# TODO
# https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html

def build_XY_graph(ax, robot_state_df, parameters={}):
    """
    """
    ax.plot(
        np.array(robot_state_df['x']),
        np.array(robot_state_df['y']),
        **parameters
    )

def build_XT_graph(ax, robot_state_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['x']),
        **parameters
    )


def build_YT_graph(ax, robot_state_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['y']),
        **parameters
    )

def build_RollT_graph(ax, robot_state_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['roll']),
        **parameters
    )

def build_PitchT_graph(ax, robot_state_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['pitch']),
        **parameters
    )

def build_YawT_graph(ax, robot_state_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['yaw']),
        **parameters
    )

def build_VT_graph(ax, robot_state_df, control_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['v']),
        **parameters
    )

    ax.plot(
        np.array(time_seq),
        np.array(control_df['x']),
        **parameters
    )

def build_WT_graph(ax, robot_state_df, control_df, time_seq, parameters={}):
    """
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['w']),
        **parameters
    )

    ax.plot(
        np.array(time_seq),
        np.array(control_df['yaw']),
        **parameters
    )

def build_general_graph_for_rosbot(robot_state_df, control_df=None, time_list=None):
    """
    """
    fig, axs = plt.subplots(8)
    build_XY_graph(axs[0], robot_state_df, parameters={'label':'Y(X)'})
    build_XT_graph(axs[1], robot_state_df, time_list)
    build_YT_graph(axs[2], robot_state_df, time_list)
    build_RollT_graph(axs[3], robot_state_df, time_list)
    build_PitchT_graph(axs[4], robot_state_df, time_list)
    build_YawT_graph(axs[5], robot_state_df, time_list)
    build_VT_graph(axs[6], robot_state_df, control_df, time_list)
    build_WT_graph(axs[7], robot_state_df, control_df, time_list)

    plt.show()
    

