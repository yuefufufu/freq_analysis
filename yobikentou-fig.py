#!/usr/bin/env python3
import rospy
import numpy as np
from scipy import fftpack
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# 各線の太さを設定
linewidths = [0.5, 0, 0.5, 0, 0]#x,y,z,z,hoge
linewidths_fft = [0.5, 0.5, 0.5, 0, 0]#x,y,z,z,hoge

# alphabet_list = list('abcdefghijklmnopqrst')
alphabet_list = list('abcdefg')

#7,12,16

num1 = 160
num2 = 2


for alphabet in alphabet_list:
    csv_file = os.path.expanduser('~/catkin_ws/rosbag/honjikken/') + alphabet + '_force_' + str(num1) + '-' + str(num2) + '-.csv'
    png_file = os.path.expanduser('~/catkin_ws/rosbag/honjikken/fig_force/') + alphabet + '_forcexy_' + str(num1) + '-' + str(num2) + '.png'
    # csv_file = os.path.expanduser('~/catkin_ws/rosbag/next/ken/u_220f.csv')
    # png_file = os.path.expanduser('~/catkin_ws/rosbag/next/ken/u_220f.png')

    df=pd.read_csv(csv_file)

    df=df.interpolate()
    df["%time"]=(df["%time"].values-df["%time"].values[0])/1000000000
    df=df[(df["%time"]>=20)&(df["%time"]<=40)]
    

    # df_original = df.copy()
    

    df["%time"]=(df["%time"].values-df["%time"].values[0])


    df_original = df
    df_original = df_original.reindex(columns=["%time","field.linear.z","field.linear.y","field.linear.x","field.angular.x","field.angular.y","field.angular.z"])
    print(df_original)
    df_original["field.linear.z"] = df_original["field.linear.z"] + 4

    print(df_original)



    df=df.rename(columns={"%time":"time","field.linear.x":"lin.x","field.linear.y":"lin.y","field.linear.z":"lin.z","field.angular.x":"ang.x","field.angular.y":"ang.y","field.angular.z":"ang.z"})

    df = df[["time","lin.z"]]
    # df = df[["time","lin.x","lin.y","lin.z","ang.z"]]

    df.to_csv('~/catkin_ws/rosbag/yobikentou_1/data_change.csv',index=False)

    def calc_fft(data,samplerate):
        spectrum=fftpack.fft(data)
        amp=np.sqrt((spectrum.real**2)+(spectrum.imag**2))
        amp=amp/(len(data)/2)
        phase=np.arctan2(spectrum.imag,spectrum.real)
        phase=np.degrees(phase)
        freq=np.linspace(0,samplerate,len(data))
        return spectrum,amp,phase,freq
    def csv_fft(in_file,out_file):
        df=pd.read_csv(in_file,encoding='SHIFT-JIS')
        dt=df.T.iloc[0,1]
        df_amp=pd.DataFrame()
        df_phase=pd.DataFrame()
        df_fft=pd.DataFrame()
        for i in range(len(df.T)-1):
            data=df.T.iloc[i+1]
            spectrum,amp,phase,freq=calc_fft(data.values,1/dt)
            df_amp[df.columns[i+1]]=pd.Series(amp)
            df_phase[df.columns[i+1]+'_phase[deg]']=pd.Series(phase)
        df_fft['freq[Hz]']=pd.Series(freq)
        df_fft=df_fft.join(df_amp).join(df_phase)
        df_fft=df_fft.iloc[range(int(len(df)/2)+1),:]
        df_fft.to_csv(out_file)
        return df,df_fft

    df,df_fft=csv_fft(in_file='~/catkin_ws/rosbag/yobikentou_1/data_change.csv',out_file='~/catkin_ws/rosbag/yobikentou_1/data_fft.csv')


    # df_fft["lin.x"]=(df_fft["lin.x"].values/df_fft["lin.x"].max(axis=0))
    # df_fft["lin.y"]=(df_fft["lin.y"].values/df_fft["lin.y"].max(axis=0))
    df_fft["lin.z"]=(df_fft["lin.z"].values/df_fft["lin.z"].max(axis=0))
    # df_fft["ang.x"]=(df_fft["ang.x"].values/df_fft["ang.x"].max(axis=0))
    # df_fft["ang.y"]=(df_fft["ang.y"].values/df_fft["ang.y"].max(axis=0))
    # df_fft["ang.z"]=(df_fft["ang.z"].values/df_fft["ang.z"].max(axis=0))

    plt.rcParams['font.size']=10
    plt.rcParams['xtick.direction']='in'
    plt.rcParams['ytick.direction']='in'
    fig=plt.figure(figsize=(10,5))
    ax1=fig.add_subplot(121)
    ax1.yaxis.set_ticks_position('both')
    ax1.xaxis.set_ticks_position('both')
    ax2=fig.add_subplot(122)
    ax2.yaxis.set_ticks_position('both')
    ax2.xaxis.set_ticks_position('both')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Force (N)')

    ax1.set_ylim(-40, 40)
    ax2.set_xlabel('Frequency [Hz]')
    ax2.set_ylabel('Signal value')
    ax2.set_xlim(0,5)
    size=len(df.T)-1

    for i in range(size):
        ax1.plot(np.array(df_original.T.iloc[0]),np.array(df_original.T.iloc[i+1]),label=df.columns[i+1],lw=linewidths[i])
        ax2.plot(np.array(df_fft.T.iloc[0]),np.array(df_fft.T.iloc[i+1]),label=df_fft.columns[i+1],lw=linewidths_fft[i])

    # ax1.legend(loc="upper left")
    # ax2.legend(loc="upper right")
    fig.tight_layout()

    plt.pause(0.5)
    plt.close()

    directory = os.path.dirname(png_file)
    if not os.path.exists(directory):
        os.makedirs(directory)

    fig.savefig(png_file)
