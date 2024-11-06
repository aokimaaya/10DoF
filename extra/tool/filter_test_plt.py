import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy
import datetime
data_= pd.read_csv("./tool/data.csv")
data=list(data_["Present Position"][15:50])
data= [data_["Present Position"][15]]*20 + data
y_live_sosfilt=[]
name =str(datetime.datetime.now()).replace(" ", "_").replace(":", "-")
# data[:171]
d=3
fs = 30
Wn=2
from utils.digitalfilter import LiveSosFilter
sos1 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                    ftype="butter", output="sos")

live_sosfilter1 = LiveSosFilter(sos1,[data_["Present Position"][15]]*4)#list(range(data[0]/2,data[0])))#[name]*4)#
y_live_sosfilt= [live_sosfilter1(y) for y in data]
plt.figure(figsize=[16, 10])
plt.plot(range(0,len(data)), data,label="Noisy signal - Follower")
# plt.plot(ts, y_scipy_sosfilt, lw=2, label="SciPy lfilter")
plt.plot(range(0,len(y_live_sosfilt)), y_live_sosfilt, lw=5, label="LiveLFilter Output")

plt.legend(loc="lower center", bbox_to_anchor=[0.5, 1], ncol=2,
            fontsize="smaller")
plt.xlabel("Time / s")
plt.ylabel("Amplitude")
plt.tight_layout()
# plt.pause(0.25)
plt.savefig("./tool/fig/"+name+".png")
plt.show()
plt.legend(loc="lower center", bbox_to_anchor=[0.5, 1], ncol=2,
            fontsize="smaller")
plt.xlabel("Time / s")
plt.ylabel("Amplitude")
plt.tight_layout()
# plt.pause(0.25)
plt.savefig("./tool/fig/"+name+".png")
plt.show()