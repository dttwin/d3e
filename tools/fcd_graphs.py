import numpy as np
import pandas as pd
# import matplotlib
# matplotlib.use('agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from collections import namedtuple
import sys

plt.rcParams['animation.ffmpeg_path'] = r'C:\Devel\FFmpeg\bin\ffmpeg.exe'
# plt.rcParams['animation.ffmpeg_path'] = r'D:\Video\ffmpeg-4.1.1-win64-static\bin\ffmpeg.exe'
DataXY = namedtuple('DataXY', ['x_data', 'y_data', 'x_min', 'x_max', 'y_min', 'y_max', 'ylabel', 'color'])


class AnimationParameters(object):

    def __init__(self):
        self.color = None
        self.x_data = None
        self.y_data = []
        self.x_label = ''
        self.y_label = ''
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None
        self.width = 600
        self.height = 600
        self.legend = []
        self.legend_loc = None


class Animation(object):

    def __init__(self, file_name, data_frames, params):
        """

        :type data_frames: list
        :type file_name: str
        :type params: AnimationParameters
        :param file_name:
        :param data_frames:
        :param params:
        """
        self.file_name = file_name
        self.data_frames = data_frames
        self.parameters = params

    def get_parameters(self):
        return self.parameters

    def get_file_name(self):
        return self.file_name

    def get_dataframes(self):
        return self.data_frames


def update_figure(plot_lines, frame_n, my_data, my_params):
    """

    :type frame_n: int
    :type my_params: AnimationParameters
    :param frame_n:
    :param my_data:
    :param my_params:
    :return:
    """
    x = my_data[0][my_params.x_data]
    i = 0
    for y_name in my_params.y_data:
        for df in my_data:
            y = df[y_name]
            plot_lines[i].set_data(x.iloc[:frame_n], y.iloc[:frame_n])
            i += 1


def generate_animation(animation, fps=16, dpi=96):
    """

    :param animation:
    :param fps:
    :param dpi:
    """
    params = animation.get_parameters()
    movie_writer = FFMpegWriter(fps=fps)
    print(animation)
    fig = plt.figure(figsize=(params.width / dpi, params.height / dpi), dpi=dpi)
    axes = [plt.gca()]
    num_y_axes = 1
    if len(params.y_data) > 1:
        axes.append(axes[0].twinx())
        num_y_axes = 2
    lines = []
    dfs = animation.get_dataframes()
    num_frames = sys.maxsize
    idx = 0
    for y_name in params.y_data:
        didx = 0
        for df in dfs:
            colr = params.color[idx][didx]
            l, = axes[idx].plot([], [], label=y_name, color=colr)
            lines.append(l)
            didx += 1
            num_df_rows = df.shape[0]
            if num_df_rows < num_frames:
                num_frames = num_df_rows
        if num_y_axes == 1:
            colr = 'black'
        axes[idx].set_ylim(params.y_min[idx], params.y_max[idx])
        axes[idx].set_ylabel(params.y_label[idx], color=colr)
        axes[idx].tick_params('y', colors=colr)
        if idx == 0:
            idx = 1
    if params.legend:
        if params.legend_loc is None:
            plt.legend(params.legend)
        else:
            plt.legend(params.legend, loc=params.legend_loc)
    plt.xlim(params.x_min, params.x_max)
    plt.xlabel(params.x_label)
    fig.tight_layout()

    # Get the number of frames

    with movie_writer.saving(fig, animation.get_file_name(), dpi=dpi):
        # for j in range(timestamps.size):
        j = 0
        t_max = 1.0
        for frame_id in range(num_frames):
            print(f'Frame {frame_id+1}/{num_frames}, {100*(frame_id+1)/num_frames:.2f}% finished')
            # timestamp = 10.0 * frame_id / fps
            # if t_max - timestamp < 1e-6:
            #     j += 1
            #     t_max += 1.0
            #    print('Timestamp', timestamp, '- new frame', j)
            update_figure(lines, frame_id, dfs, params)
            movie_writer.grab_frame()

    del movie_writer


def partial_cumsum(df, columns):
    """

    :param df: 
    :param columns: 
    :return: 
    """
    cdf = df.copy()
    df_nans = df.loc[df['waiting'].isnull(), 'timestamp']
    if df_nans.shape[0] > 0:
        idx0 = 0
        idx1 = idx0 + 1
        idx_max = df_nans.index[-1]
        for idx in df_nans.index:
            if idx > idx1 or idx == idx_max:
                if idx == idx_max:
                    idx0 = idx1
                    idx = df.index[-1]
                cdf.loc[idx0:idx, columns] = df.loc[idx0:idx, columns].cumsum()
            idx0 = idx
            idx1 = idx0 + 1
    else:
        cdf.loc[:, columns] = df.loc[:, columns].cumsum()
    return cdf


DO_VEHICLES = True
DO_TOTALS = False
DO_SPEED = False
DO_WAITING = True

if DO_TOTALS:
    channels = ('CO2', 'waiting', 'speed')
    ref_df = pd.read_csv('fcd_notts_total.csv')
    cref_df = partial_cumsum(ref_df, channels)
    llr_df = pd.read_csv('fcd_llr_total.csv')
    cllr_df = partial_cumsum(llr_df, channels)

    dllr_df = cllr_df[cllr_df['timestamp'] <= 50698]
    dllr_df.loc[:, 'waiting'] = cref_df.loc[:, 'waiting'] - llr_df.loc[:, 'waiting']
    dllr_df.loc[:, 'speed'] = cref_df.loc[:, 'speed'] - llr_df.loc[:, 'speed']

    plt.plot(dllr_df.timestamp, dllr_df.waiting)
    plt.title('Total waiting time gain')
    plt.show()

    plt.plot(dllr_df.timestamp, dllr_df.speed)
    plt.title('Total speed gain')
    plt.show()

    ap = AnimationParameters()
    ap.x_min = dllr_df['timestamp'].min()
    ap.x_max = dllr_df['timestamp'].max()
    ap.y_min = [0]
    ap.y_max = [1.05*dllr_df['waiting'].max()]
    ap.y_label = ['Cumulative waiting time savings [s]']
    ap.x_label = 'Simulated time [s]'
    ap.x_data = 'timestamp'
    ap.y_data = ['waiting']
    ap.color = [('tab:blue',)]
    ap.legend = None
    ap.legend_loc = 'upper left'
    ap.width = 580
    ap.height = 380

    anim = Animation('zizkov_llr_wait_total.mp4', [dllr_df], ap)
    generate_animation(anim)

    ap = AnimationParameters()
    ap.x_min = ref_df['timestamp'].min()
    ap.x_max = dllr_df['timestamp'].max()
    ap.y_min = [0]
    ap.y_max = [1.05*dllr_df['speed'].max()]
    ap.y_label = ['Cumulative absolute speed gain [m/s]']
    ap.x_label = 'Simulated time [s]'
    ap.x_data = 'timestamp'
    ap.y_data = ['speed']
    ap.color = [('tab:blue',)]
    ap.legend = None
    ap.legend_loc = 'upper left'
    ap.width = 580
    ap.height = 380

    anim = Animation('zizkov_llr_speed_total.mp4', [dllr_df], ap)
    generate_animation(anim)

if DO_VEHICLES:
    ref_df = pd.read_csv('fcd_notts_samples.csv')
    llr_df = pd.read_csv('fcd_llr_samples.csv')

    # Fill missing timestamp values with NaNs
    channels = ('CO2', 'waiting', 'speed')
    max_timestamp = max(ref_df['timestamp'].max(), llr_df['timestamp'].max())
    ref_range = np.arange(43200.0, max_timestamp+1)
    ref_index = pd.Index(ref_range, name="timestamp")
    ref_df = ref_df.set_index('timestamp').reindex(ref_index).reset_index()
    cref_df = partial_cumsum(ref_df, channels)

    llr_index = pd.Index(ref_range, name="timestamp")
    llr_df = llr_df.set_index('timestamp').reindex(llr_index).reset_index()
    cllr_df = partial_cumsum(llr_df, channels)

    plt.plot(ref_df.timestamp, cref_df.waiting)
    plt.plot(llr_df.timestamp, cllr_df.waiting)
    plt.title('Waiting time')
    plt.xlabel('Simulation time [s]')
    plt.ylabel('Cumulative waiting time [s]')
    plt.show()

    plt.plot(ref_df.timestamp, cref_df.speed)
    plt.plot(llr_df.timestamp, cllr_df.speed)
    plt.xlabel('Simulation time [s]')
    plt.title('Cumulative speed')
    plt.show()

    if DO_WAITING:
        ap = AnimationParameters()
        ap.x_min = ref_range[0]
        ap.x_max = ref_range[-1]
        ap.y_min = [0]
        ap.y_max = [1.05*max(cref_df['waiting'].max(), cllr_df['waiting'].max())]
        ap.y_label = ['Cumulative waiting time [s]']
        ap.x_label = 'Simulated time [s]'
        ap.x_data = 'timestamp'
        ap.y_data = ['waiting']
        ap.color = [('tab:orange', 'tab:red')]
        ap.legend = ['Reference', 'LLR']
        ap.legend_loc = 'upper left'
        ap.width = 580
        ap.height = 380

        anim = Animation('zizkov_llr_wait.mp4', [cref_df, cllr_df], ap)
        generate_animation(anim)

    if DO_SPEED:
        ap = AnimationParameters()
        ap.x_min = ref_range[0]
        ap.x_max = ref_range[-1]
        ap.y_min = [0]
        ap.y_max = [1.05*max(cref_df['speed'].max(), cllr_df['speed'].max())]
        ap.y_label = ['Cumulative speed (travelled distance) [m]']
        ap.x_label = 'Simulated time [s]'
        ap.x_data = 'timestamp'
        ap.y_data = ['speed']
        ap.color = [('tab:orange', 'tab:red')]
        ap.legend = ['Reference', 'LLR']
        ap.legend_loc = 'upper left'
        ap.width = 580
        ap.height = 380

        anim = Animation('zizkov_llr_speed.mp4', [cref_df, cllr_df], ap)
        generate_animation(anim)

    plt.plot(ref_df.timestamp, ref_df.fuel.cumsum())
    plt.plot(llr_df.timestamp, llr_df.fuel.cumsum())
    plt.title('Fuel')
    plt.xlabel('Simulation time [s]')
    plt.show()

    # plt.plot(ref_pd.timestamp, ref_pd.fuel.cumsum() - llr_pd.fuel.cumsum())
    # plt.show()

    plt.plot(ref_df.timestamp, cref_df.CO2/1000)
    plt.plot(llr_df.timestamp, cllr_df.CO2/1000)
    plt.title('CO2')
    plt.xlabel('Simulation time [s]')
    plt.legend(['reference', 'llr'])
    plt.show()

    plt.plot(ref_df.timestamp, (cref_df.CO2-cllr_df.CO2)/1000)
    plt.title('CO_2 savings')
    plt.xlabel('Simulation time [s]')
    plt.show()

    plt.plot(ref_df.timestamp, ref_df.HC.cumsum())
    plt.plot(llr_df.timestamp, llr_df.HC.cumsum())
    plt.title('HC')
    plt.xlabel('Simulation time [s]')
    plt.show()

    plt.plot(ref_df.timestamp, ref_df.PMx.cumsum())
    plt.plot(llr_df.timestamp, llr_df.PMx.cumsum())
    plt.title('PMx')
    plt.xlabel('Simulation time [s]')
    plt.show()
