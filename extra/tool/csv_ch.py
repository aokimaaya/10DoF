from tool.dym_setup import Control
import pandas as pd
DymControl = Control()
def _csv_singed_angle(csv_df,ang_list):
    

    # csv_d=csv_df[["a1",	"a2",	"a3"]]
    for a in ang_list:
        csv_df[a]=csv_df[a].apply(lambda row: DymControl.toSigned32(row))#DymControl.toSigned32(row)
        # print(csv_df["a"+a]-csv_df[a])
    # return csv_df
csv_df = pd.read_csv(
                r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping\_small_board\test\csv\10.csv",skiprows=3, index_col=None,dtype='float64'
            )
colums_=["step","time"	,"A1",	"A2",	"A3","C1","C2","C3"]

csv_df.columns=colums_#[2:csv_df.shape[1]]#["step","time"	,"a1",	"a2",	"a3","C1","C2","C3"]
print(csv_df,csv_df.shape)
# csv_df =
_csv_singed_angle(csv_df,colums_[2:5])
print(csv_df[csv_df["A3"]<0])