import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import argparse
from logger.utils import parse_logger_output_data

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
  

def build_group_of_graphs(    
    robot_state_df,
    control_df=None,
    time_list=None,     
    k_model_state_df=None,
    nn_model_state_df=None,
    keys=list()
):
    """
    """
    fig, axs = plt.subplots(len(keys))
    
    for i in range(len(keys)):
        key = keys[i]
        k_model_state_seq_= k_model_state_df[key] if k_model_state_df is not None else None
        nn_model_state_seq_= nn_model_state_df[key] if nn_model_state_df is not None else None
        control_seq_ = control_df[key] if control_df is not None and key in control_df.columns else None
        
        build_data_from_T_graph(
            axs[i],
            robot_state_df[key],
            time_list,
            k_model_state_seq_,
            nn_model_state_seq_,
            control_seq_
        )
    return fig

def build_general_graph_for_rosbot(
    robot_state_df,
    control_df=None,
    time_list=None,     
    k_model_state_df=None,
    nn_model_state_df=None,
    save_to_png=True,
    path=None
):
    """
    Args:
        :robot_state_df: (pandas.DataFrame or dict)
        :k_model_state_df: (pandas.DataFrame or dict)
        :nn_model_state_df: (pandas.DataFrame or dict)
        :control_df: (pandas.DataFrame or dict)
        :time_list: (list)
    """

    plt.grid(True)
    plt.legend(loc='best') 

    plt.subplots_adjust(
        left=0.15,
        bottom=0.15, 
        right=0.8, 
        top=0.8, 
        wspace=0.5, 
        hspace=0.5
    )

    fig_x_y_z = build_group_of_graphs(
        robot_state_df,
        control_df=control_df,
        time_list=time_list,     
        k_model_state_df=k_model_state_df,
        nn_model_state_df=nn_model_state_df,
        keys=['x', 'y', 'z']
    )
    
    if save_to_png and path is not None:
        plt.savefig('{}.{}'.format(path + 'X_Y_Z_graph', 'png'), fmt='png')

    fig_angs = build_group_of_graphs(
        robot_state_df,
        control_df=control_df,
        time_list=time_list,     
        k_model_state_df=k_model_state_df,
        nn_model_state_df=nn_model_state_df,
        keys=['roll', 'pitch', 'yaw']
    )
    
    if save_to_png and path is not None:
        plt.savefig('{}.{}'.format(path + 'Angles_graph', 'png'), fmt='png')

    fig_lin_vels = build_group_of_graphs(
        robot_state_df,
        control_df=control_df,
        time_list=time_list,     
        k_model_state_df=k_model_state_df,
        nn_model_state_df=nn_model_state_df,
        keys=['v_x', 'v_y', 'v_z']
    )
    if save_to_png and path is not None:
        plt.savefig('{}.{}'.format(path + 'Linear_velocities_graph', 'png'), fmt='png')

    fig_ang_vels = build_group_of_graphs(
        robot_state_df,
        control_df=control_df,
        time_list=time_list,     
        k_model_state_df=k_model_state_df,
        nn_model_state_df=nn_model_state_df,
        keys=['w_x', 'w_y', 'w_z']
    )
    if save_to_png and path is not None:
        plt.savefig('{}.{}'.format(path + 'Angular_velocities_graph', 'png'), fmt='png')


    fig_xy, ax = plt.subplots(1)
    build_XY_graph(
        ax,
        robot_state_df,
        k_model_state_df,
        nn_model_state_df,
    )

    if save_to_png and path is not None:
        plt.savefig('{}.{}'.format(path + 'XY_graph', 'png'), fmt='png')

    
def main():
    """

    """
    parser = argparse.ArgumentParser()
    # first arg
    parser.add_argument(
        '-folder_path',
        action='store',
        dest='folder_path',
        required=True,
        help='absolute path to the folder with data.csv'
    )
    
    # second arg
    parser.add_argument(
        '-output_folder',
        action='store', 
        dest='output_folder', 
        required=False,
        default=None, 
        help="absolute path to the output folder"
    )       

    args = parser.parse_args()

    data = parse_logger_output_data(args.folder_path)
    rosbot_state_df, k_model_state_df, nn_model_state_df = data[:3]
    control_df, time = data[3:]

    k_model_state_df = None if len(k_model_state_df) < 2 else k_model_state_df
    nn_model_state_df = None if len(nn_model_state_df) < 2 else nn_model_state_df

    fig = build_general_graph_for_rosbot(
        rosbot_state_df,
        control_df=control_df,
        time_list=list(time['t']),     
        k_model_state_df=k_model_state_df,
        nn_model_state_df=nn_model_state_df,
        save_to_png=True,
        path=args.output_folder
    )

    # if args.output_folder is not None:
    #     plt.savefig(
    #         '{}'.format(args.output_folder + '/create_graphs.png'),
    #         fmt='png'
    #     )

    plt.show()

if __name__ == '__main__':
    main()

