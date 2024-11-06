import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from matplotlib import cm

angles_list=[50,45,40,35,30,25,20]
diff = len(angles_list)
q=256#PCn.shape[0]/diff
trails = int(1280*diff/(q*diff))

seq = 1280*diff

print(angles_list,diff,q,trails,seq)

data_des = {}
keyz = ['20', '35', '50', 'AC']
for k in keyz:
    data_des[k] = pd.read_csv(r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping\Analysis\Data_Processed\Data_Fixed_Motion_{}.csv".format(k))

data_mg = pd.read_csv(r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping\Analysis\Data_Processed\Data_Motion_Generation.csv")

units  ={"angle":"Angle in degree","current":"Current in milli-ampere"} #["Angle in degree"]*3+["Current in milli-ampere"]*3
a_i = {"angle":["A1","A2","A3"],"current":["C1","C2","C3"]}

keyz = list(data_des.keys())
fontsize = 70
font = {'family': 'normal', 'weight': 'bold', 'size': fontsize}
plt.rc('font', **font)
col = cm.tab20(range(14))
q = 1280
an = 3

for k in keyz[:3]:
    for i in a_i.keys():
        for j in range(3):
            kk = a_i[i][j]
            print(kk, k, data_des[k].columns, data_des[k][kk].shape)
            print(data_mg.columns)
            ta = data_des["AC"][a_i[i][j] + "_"][:int(q * an)]
            x1 = data_des[k][a_i[i][j] + "_"][:int(q * an)]
            y1 = data_mg[a_i[i][j] + "_"][:int(q * an)]
            y1_ = data_mg[a_i[i][j] + "p" + "_"][:int(q * an)]
            print("q", q, x1.shape, y1.shape)
            time_x = list(range(ta.shape[0]))
            fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(45, 25), sharey=True)

            for d in range(an):
                plt.subplot(1, 3, d + 1)
                cy = 0
                qs = 0
                sq = 1
                [i.set_linewidth(4.5) for i in ax[d].spines.values()]
                print(d)

                time_xx = time_x[0:q - 1]
                print(time_xx)
                ta1 = ta[d * q + qs:-1 + (1 + d) * q - cy] ** sq
                yy1 = y1[d * q + qs:-1 + (1 + d) * q - cy] ** sq
                yy1_ = y1_[d * q + qs:-1 + (1 + d) * q - cy] ** sq
                plt.plot(time_xx, yy1, lw=8, c=col[d * 2], label=str(angles_list[d]) + "_Realized_Motion")
                plt.plot(time_xx, ta1, lw=7, linestyle="-.", c=col[d * 2 + 1], label=str(angles_list[d]) + "_Train_values")
                labels = np.array(time_xx[::320])*0.0625

                plt.xticks(time_xx[::320], labels.astype(int),)

                plt.xlabel("Time in seconds")
                plt.legend(loc="lower center", bbox_to_anchor=[0.5, 1], ncol=1, fontsize=fontsize - 10)
                if d == 0:
                    plt.ylabel(units[i])
            plt.savefig(r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\extra\tool\{}_{}_.pdf".format(k,a_i[i][j]))


            # plt.show()

    #         break

    #     break

    # break
