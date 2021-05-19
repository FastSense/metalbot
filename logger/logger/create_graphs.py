import matplotlib.pyplot as plt
import numpy as np

# TODO
# https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html

def build_XY_graph(
    ax,
    robot_state_df,
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :parameters: (dict)
    """
    ax.plot(
        np.array(robot_state_df['x']),
        np.array(robot_state_df['y']),
        **parameters
    )
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('y(x)')
    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)

def build_XT_graph(
    ax,
    robot_state_df,
    time_seq,
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['x']),
        **parameters
    )
    ax.set_xlabel('t')
    ax.set_ylabel('x')
    ax.set_title('x(t)')
    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)

def build_YT_graph(
    ax,
    robot_state_df, 
    time_seq, 
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['y']),
        **parameters
    )
    ax.set_xlabel('t')
    ax.set_ylabel('y')
    ax.set_title('y(t)')

    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)

def build_RollT_graph(
    ax,
    robot_state_df, 
    time_seq,
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['roll']),
        **parameters
    )
    ax.set_xlabel('t')
    ax.set_ylabel('roll')
    ax.set_title('roll(t)')

    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)

def build_PitchT_graph(
    ax,
    robot_state_df,
    time_seq,
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['pitch']),
        **parameters
    )
    ax.set_xlabel('t')
    ax.set_ylabel('pitch')
    ax.set_title('pitch(t)')

def build_YawT_graph(
    ax, 
    robot_state_df, 
    time_seq, 
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['yaw']),
        **parameters
    )
    ax.set_xlabel('t')
    ax.set_ylabel('yaw')
    ax.set_title('yaw(t)')

    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)
    

def build_VT_graph(
    ax,
    robot_state_df,
    control_df,
    time_seq,
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['v_x']),
        **parameters,
        label='V_x(t)'

    )

    ax.plot(
        np.array(time_seq),
        np.array(control_df['x']),
        label='UV_x(t)'
    )

    ax.set_xlabel('t')
    ax.set_ylabel('V_x')
    ax.set_title('V_x(t) and UV_x(t)')
    
    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)

def build_WT_graph(
    ax, 
    robot_state_df, 
    control_df, 
    time_seq, 
    k_model_state_df=None,
    nn_model_state_df=None,
    parameters={}
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :control_df: (pandas.DataFrame or dict)
        :time_seq: (list)
        :parameters: (dict)
    """
    ax.plot(
        np.array(time_seq),
        np.array(robot_state_df['w_z']),
        **parameters
    )

    ax.plot(
        np.array(time_seq),
        np.array(control_df['yaw']),
    )

    ax.set_xlabel('t')
    ax.set_ylabel('W_z')
    ax.set_title('W_z(t) and UW_z(t)')

    legend = ['Rosbot state']
    legend = legend + ['Kinematic model state'] if k_model_state_df is not None else legend
    legend = legend + ['NN model state'] if nn_model_state_df is not None else legend
    ax.legend(legend)
    

def build_general_graph_for_rosbot(
    robot_state_df,
    control_df=None,
    time_list=None,     
    k_model_state_df=None,
    nn_model_state_df=None
):
    """
    Args:
        :robot_state_df: (pandas.DataFrame or dict)
        :control_df: (pandas.DataFrame or dict)
        :time_seq: (list)
    """
    fig, axs = plt.subplots(8)
    build_XY_graph(axs[0], robot_state_df)
    build_XT_graph(axs[1], robot_state_df, time_list)
    build_YT_graph(axs[2], robot_state_df, time_list)
    build_RollT_graph(axs[3], robot_state_df, time_list)
    build_PitchT_graph(axs[4], robot_state_df, time_list)
    build_YawT_graph(axs[5], robot_state_df, time_list)
    build_VT_graph(axs[6], robot_state_df, control_df, time_list)
    build_WT_graph(axs[7], robot_state_df, control_df, time_list)
    plt.grid(True)
    plt.legend(loc='best') 
    plt.show()
    

