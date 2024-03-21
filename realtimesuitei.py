#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time
import pandas as pd
import numpy as np
from scipy import fftpack

# Define the wait times here
WAIT_TIME_AFTER_KEYPRESS = 1
SUBSCRIBE_DURATION = 5

def getKey(key_timeout):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# フーリエ変換をする関数
def calc_fft(data, samplerate):
    spectrum = fftpack.fft(data)                                     # 信号のフーリエ変換
    amp = np.sqrt((spectrum.real ** 2) + (spectrum.imag ** 2))       # 振幅成分
    amp = amp / (len(data) / 2)                                      # 振幅成分の正規化（辻褄合わせ）
    phase = np.arctan2(spectrum.imag, spectrum.real)                 # 位相を計算
    phase = np.degrees(phase)                                        # 位相をラジアンから度に変換
    freq = np.linspace(0, samplerate, len(data))                     # 周波数軸を作成
    return spectrum, amp, phase, freq

# DataFrameから列方向に順次フーリエ変換を行う関数
def df_fft(df, dt):
    # データフレームを初期化
    df_amp = pd.DataFrame()
    df_phase = pd.DataFrame()
    df_fft = pd.DataFrame()

    # 列方向に順次フーリエ変換（DFT）をするコード
    for i in range(len(df.columns)):
        data = df.iloc[:, i]                                         # フーリエ変換するデータ列を抽出
        spectrum, amp, phase, freq = calc_fft(data.values, 1/dt)     # フーリエ変換をする関数を実行
        df_amp[df.columns[i]] = pd.Series(amp)                       # 列名と共にデータフレームに振幅計算結果を追加
        df_phase[df.columns[i] + '_phase[deg]'] = pd.Series(phase)   # 列名と共にデータフレームに位相計算結果を追加

    df_fft['freq[Hz]'] = pd.Series(freq)                             # 周波数軸を作成
    df_fft = df_fft.join(df_amp).join(df_phase)                      # 周波数・振幅・位相のデータフレームを結合
    df_fft = df_fft.iloc[range(int(len(df)/2) + 1),:]                # ナイキスト周波数でデータを切り捨て

    return df_fft

class ForceSensorSubscriber(object):
    def __init__(self):
        self.data = []
        self.subscriber = rospy.Subscriber("/force_sensor_vel", Twist, self.callback)

    def callback(self, msg):
        self.data.append({"time": rospy.get_time(), "linear_z": msg.linear.z})

    def to_df(self):
        df = pd.DataFrame(self.data)
        if len(df) > 0:
            df = df.iloc[:-1]  # 最後の行を削除
        return df

def main():
    rospy.init_node('force_sensor_subscriber', anonymous=True)
    force_sensor_subscriber = ForceSensorSubscriber()

    rate = rospy.Rate(10)  # 10Hz
    start_time = None
    while not rospy.is_shutdown():
        key = getKey(0.1)
        if key == 'w' and start_time is None:
            time.sleep(WAIT_TIME_AFTER_KEYPRESS)
            start_time = rospy.get_time()
        elif start_time is not None and rospy.get_time() - start_time > SUBSCRIBE_DURATION:
            break
        rate.sleep()

    df = force_sensor_subscriber.to_df()
    print("-------------------")
    print(df)
    print("-------------------")
    df["time"] = (df["time"].values - df["time"].values[0] )
    print(df)
    print("-------------------")
    time_threshold = df.iloc[-1]['time'] - SUBSCRIBE_DURATION
    df = df[df['time'] > time_threshold]
    print(df)
    print("-------------------")
    df["time"] = (df["time"].values - df["time"].values[0] )
    df = df.reset_index(drop=True)
    print(df)
    print("-------------------")
    print(len(df))
    print("-------------------")
    dt = SUBSCRIBE_DURATION / (len(df) - 1 )
    print(dt)
    print("-------------------")
    df_fft_result = df_fft(df, dt)
    print(df_fft_result)
    print("-------------------")
    df_fft_result["linear_z"] = (df_fft_result["linear_z"].values / df_fft_result["linear_z"].max(axis=0))
    print(df_fft_result)
    print("-------------------")
    freq_band = df_fft_result[(df_fft_result["freq[Hz]"] >= 0.75) & (df_fft_result["freq[Hz]"] <= 1.0)]
    print(freq_band)
    print("-------------------")
    
    max_z = freq_band['linear_z'].idxmax()
    if max_z == 0:
        freq_band = freq_band.loc[max_z : max_z+1]
    elif max_z == len(freq_band) - 1:
        freq_band = freq_band.loc[max_z-1 : max_z]
    else:
        freq_band= freq_band.loc[max_z-1 : max_z+1]
    freq_band = freq_band.copy()
    freq_band.loc[:, "freq[Hz]"] = freq_band["freq[Hz]"] * 2
    freq_band.loc[:, 'product'] = freq_band['freq[Hz]'] * freq_band['linear_z']
    sum_pro = freq_band['product'].sum()
    sum_lin = freq_band['linear_z'].sum()
    freq_est = sum_pro / sum_lin
    # print(freq_est)
    # print("-------------------")
    # print(2 * freq_est)
    print("\033[93m" + str(freq_est) + "\033[0m")
    sys.exit()

if __name__=="__main__":
    main()
