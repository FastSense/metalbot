import matplotlib.pyplot as plt
import numpy as np

# TODO
# https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html
"""
Plot tools for rosbot
"""

def build_data_from_T_graph(
    ax,
    robot_state_seq,
    time_seq,
    k_model_state_seq=None,
    nn_model_state_seq=None,
    control_seq=None,
    legend = ['Rosbot state']
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_seq: (pandas.Series) Seq
        :k_model_state_seq: (pandas.Series)
        :nn_model_state_seq: (pandas.Series)
        :parameters: (dict)
    """
    
    ax.set_xlabel('t')
    ax.set_ylabel(robot_state_seq.name)
    ax.set_title('{}(t)'.format(robot_state_seq.name))

    ax.plot(
        np.array(time_seq),
        np.array(robot_state_seq),
    )

    kinetic_model_exist = k_model_state_seq is not None 
    nn_model_exist = nn_model_state_seq is not None
    control_seq_exist = control_seq is not None
    
    if kinetic_model_exist:
        legend = legend + ['Kinematic model state']
        ax.plot(
            np.array(time_seq),
            np.array(k_model_state_seq),
        )

    if nn_model_exist:
        legend = legend + ['NN model state']
        ax.plot(
            np.array(time_seq),
            np.array(nn_model_state_seq),
        )

    if control_seq_exist:
        legend = legend + ['Control']
        ax.plot(
            np.array(time_seq),
            np.array(control_seq),
        )

    ax.grid()
    ax.legend(legend)

def build_XY_graph(
    ax,
    robot_state_df,
    k_model_state_df=None,
    nn_model_state_df=None,
):
    """
    Args:
        :ax: (matplotlib.axes.Axes)
        :robot_state_df: (pandas.DataFrame or dict)
        :parameters: (dict)
    """

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('y(x)')

    legend = ['Rosbot state']
    ax.plot(
        np.array(robot_state_df['x']),
        np.array(robot_state_df['y']),
        color='black',
        linestyle='--'
    )

    kinetic_model_exist = k_model_state_df is not None  
    nn_model_exist = nn_model_state_df is not None 

    if kinetic_model_exist:
        legend = legend + ['Kinematic model state']
        ax.plot(
            np.array(k_model_state_df['x']),
            np.array(k_model_state_df['y']),
        )

    if nn_model_exist:
        legend = legend + ['NN model state']
        ax.plot(
            np.array(nn_model_state_df['x']),
            np.array(nn_model_state_df['y']),
        )

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
        :k_model_state_df: (pandas.DataFrame or dict)
        :nn_model_state_df: (pandas.DataFrame or dict)
        :control_df: (pandas.DataFrame or dict)
        :time_seq: (list)
    """
    columns = robot_state_df.columns
    number_of_keys = len(columns)
    fig, axs = plt.subplots(number_of_keys+1, figsize=(9,30))
    for i in range(number_of_keys):
        key = columns[i]
        k_model_state_seq_= k_model_state_df[key] if k_model_state_df is not None else None
        nn_model_state_seq_= nn_model_state_df[key] if nn_model_state_df is not None else None
        control_seq_ = control_df[key] if key in ['v_x', 'w_z'] else None
        
        build_data_from_T_graph(
            axs[i],
            robot_state_df[key],
            time_list,
            k_model_state_seq_,
            nn_model_state_seq_,
            control_seq_
        )

    
    build_XY_graph(
        axs[-1],
        robot_state_df,
        k_model_state_df,
        nn_model_state_df,
    )

    plt.subplots_adjust(
        left=0.1,
        bottom=0.1, 
        right=0.9, 
        top=0.9, 
        wspace=0.4, 
        hspace=0.6
    )

    plt.grid(True)
    plt.legend(loc='best') 
    # plt.show()
    return fig
    

