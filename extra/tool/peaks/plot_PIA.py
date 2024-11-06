# df_3d_sum_all= df_d_dict_agg[scaling][funcvar][func][k]
scaling="no-scale"
funcvar="Mean"
func="Mean"
k="angle"
import numpy as np
import matplotlib.pyplot as plt
# from scipy import electrocardiogram
import scipy
# import scipy.datasets as electrocardiogram
from scipy.signal import find_peaks
#################
font = {'family' : 'normal',
            'weight' : 'bold',
            'size'   : 120}
plt.rc('font', **font)
#################
# print(scaling,funcvar,func,k)
# display(df_3d_sum_all)
import pandas as pd
df_3d_sum_all= pd.read_csv("C:/Users/81809/Desktop/Three_DoF_Robotarm-main/Three_DoF_Robotarm/extra/tool/peaks/Performance Index for Fixed motion vs Learning motion - "+scaling+"-"+ k+"_difference-"+funcvar+"-"+func+".csv")#r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping\Analysis\503520_MG_PRED_T_{}-{}-{}_.csv".format(scaling,func,k),index=True)
print(df_3d_sum_all)
# plt.subplot(2,1,kk_+1)
# df_3a.reset_index().plot(x="test_angle",y=["50","35","20"],subplots=True,layout=(1,3),kind="bar",figsize=(90,20),rot=0,xlabel="test angles in degree",ylabel="Adaptability Index")#.legend(title = "Adaptability Index for designated motion",ncol = 3)legend =True
# df_3d_sum_all.columns=["test_angle","50","35","20","Realized motion (output of servomotor)","Generated motion (input of servomotor)","Train data"]#,"Motion Generation","Predicted"
df_3d_sum_all.plot(x="test_angle",y=["50","35","20","Realized motion (output of servomotor)"] ,kind="bar",legend =True,figsize=(60,35),rot=0,xlabel="test angles in degree",ylabel="Performance Index - "+k,fontsize=100)#funcvar+"("+k+"_difference)^2")
# if k == "current":ax=ax[kk_],
plt.legend(title = "Performance Index for Fixed motion vs Learning motion ",ncol = 2,fontsize = 100, fancybox=True, framealpha=0.1,bbox_to_anchor=[0.5, 0.97],loc="lower center")
plt.savefig("C:/Users/81809/Desktop/Three_DoF_Robotarm-main/Three_DoF_Robotarm/extra/tool/peaks/Performance Index for Fixed motion vs Learning motion - "+scaling+"-"+ k+"_difference-"+funcvar+"-"+func+"_.pdf")#+"-"+func
# plt.show()            