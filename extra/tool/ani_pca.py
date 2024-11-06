##PCA data
import time

import cv2
import matplotlib.animation
import matplotlib.pylab as plt
import numpy as np
import pandas as pd
import torchvision.datasets as datasets
from matplotlib import cm
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA
from sklearn.manifold import TSNE

# logs_path = r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\logs\prediction_logs\wiping\2022-12-08_21-16-19.096681-0to605x5x7"
# log_dir = r"\predict_20221207_202813-inc5x5x7.csv"
# limit_seq = -1
# # load dataset
# data_= pd.read_csv(logs_path+log_dir,skiprows=2, index_col=None, dtype="float64")

logs_path = r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\logs\prediction_logs\wiping\2022-12-12_21-50-45.382105-withHid"
log_dir = r"\hid.csv"
limit_seq = -1
# load dataset
data_ = pd.read_csv(
    logs_path + log_dir, skiprows=2, index_col=None, dtype="float64"
).dropna()

print(data_.head(5))
print(data_.shape, data_[:limit_seq].shape)
data_rows = data_[:limit_seq]  # .to_numpy()
colm = data_rows.columns


def update_graph(num):
    index = num  # np.where(target==num)
    # print("i,index",i,index)
    #     ax.scatter( PCn[index,0], PCn[index,1], PCn[index,2])

    print("index", index)
    # Data for three-dimensional scattered points
    zdata = PCn[index : index + q, 2]  # 15 * np.random.random(100)
    xdata = PCn[
        index : index + q, 0
    ]  # xline#np.sin(zdata) + 0.1 * np.random.randn(100)
    ydata = PCn[
        index : index + q, 1
    ]  # yline# np.cos(zdata) + 0.1 * np.random.randn(100)
    # zdata = PCn[index:index+1,2]#15 * np.random.random(100)
    # xdata =PCn[index:index+1,0]# xline#np.sin(zdata) + 0.1 * np.random.randn(100)
    # ydata =PCn[index:index+1,1]#yline# np.cos(zdata) + 0.1 * np.random.randn(100)
    # ax.scatter3D(xdata, ydata, zdata, c=zdata)
    # data=df[df['time']==num]
    # print("zdata",zdata)
    # graph.axes.set_xticks(np.arange(min(list(PCn[:, 0])),max(list(PCn[:, 0])),50),rotation = 60,size=12)
    # graph.axes.set_yticks(np.arange(min(list(PCn[:, 1])),max(list(PCn[:, 1])),500),rotation = 60,size=12)
    # ax.axes.set_zticks(np.arange(min(list(PCn[:, 2])),max(list(PCn[:, 2])),50),rotation = 60,size=12)
    # ax.axes.set_zticks(len(zdata))

    cc = int((index + 1) / (q * 5)) * 2
    mm = int((index + 1) / (q))

    print(colr[cc])
    c = colr[cc]
    m = marker[mm + 1]

    print("cc", cc, "m", m)

    print(list(L.get_texts()))
    # graph.set_data (xdata, ydata)
    # graph.set_marker(m)
    # graph.set_color(c)
    # graph.set_3d_properties(zdata)
    L.get_texts().append(marker[mm + 1] + i + " " + str(cc))
    print(list(L.get_texts()))

    (graph,) = ax.plot(xdata, ydata, zdata, linestyle="-", marker=m, c=c)
    title.set_text(i + " Step ={}".format(num))
    # L.get_texts()[0]t(marker[cc+1] +  i +" "+str(cc))

    # L.get_texts()[0].set_text([marker[:cc+1] +  i +" "+str(cc)])

    return (
        title,
        graph,
    )


for i in ["Hid"]:  # ["angle", "current"]:
    # data_.plot(x="step",y=["train_"+i+"_loss","test_"+i+"_loss"])
    q = 256
    colr = cm.tab20(range(3))

    marker = [".", "^", "o", "*", "+", ",", "1", "2", "3", "s", "x", "h"]
    steps = list(range(len(data_rows)))
    if i == "angle":

        x_positions = np.array(data_rows[colm[2:5]]).astype(int)
        y_positions = np.array(data_rows[colm[5:8]]).astype(int)

    elif i == "current":
        x_positions = np.array(data_rows[colm[8:11]]).astype(int)
        y_positions = np.array(data_rows[colm[11:]]).astype(int)
    else:

        y_positions = np.array(data_rows[colm[14:26]])

    # if i == "angle":

    # idx = np.random.permutation(60000)[:5000] # Randomly extract 5,000 data from 60,000 data.
    # mnist_data = datasets.MNIST('~/tmp/mnist', train=True, download=True)
    # target = x_positions#mnist_data.targets.numpy()[idx]
    # target_name = x_positions#[0,1,2,3,4,5,6,7,8,9]
    # x_train = x_positions#mnist_data.data.numpy()[idx]

    # load image feature extracted by CNNAE
    im_feat = y_positions  # np.load('./output/2_im_feat_10000.npy')
    print(im_feat.shape)
    # im_feat = im_feat[idx]
    # PCA
    pca = PCA(n_components=3, random_state=41)
    PCn = pca.fit_transform(
        im_feat
    )  # .singular_values_                                       # PCA process
    print("PCA data size: ", PCn.shape, PCn)

    fig = plt.figure(figsize=(18, 16))
    ax = plt.axes(projection="3d")
    q = 256
    marker = [".", "^", "o", "*", "+", ".", "^", "o", "*", "+"]
    colr = cm.tab20(range(10))
    print("PCn.ndim", PCn.shape, PCn)
    s = 0
    # L=ax.legend(marker[1] +str(0))
    for i in range(10):  # target_name:

        index = i * q  # np.where(target==i)
        # cc=round(( index)/q)-1

        cc = int((index + 1) / (q * 5))
        mm = int((index + 1) / (q))
        #     print(colr[int(cc)])
        c = colr[int(cc) * 2]
        m = marker[mm]
        if (s + 1) % q == 0:
            print("cc", cc, "m", m, "s", s)
        # print("i,index",i,index)
        #     ax.scatter( PCn[index,0], PCn[index,1], PCn[index,2])

        # Data for three-dimensional scattered points
        zdata = PCn[index : index + q, 2]  # 15 * np.random.random(100)
        xdata = PCn[
            index : index + q, 0
        ]  # xline#np.sin(zdata) + 0.1 * np.random.randn(100)
        ydata = PCn[
            index : index + q, 1
        ]  # yline# np.cos(zdata) + 0.1 * np.random.randn(100)
        ax.scatter3D(
            xdata, ydata, zdata, linestyle="-", color=c, label=str(cc) + "-" + m
        )

        # L.get_texts().append(m + str( i) +" "+str(cc))
        s += 1
    # Data for a three-dimensional line
    zline = PCn[:, 2]  # np.linspace(0, 15, 1000)
    xline = PCn[:, 0]  # np.sin(zline)
    yline = PCn[:, 1]  # np.cos(zline)
    ax.plot3D(xline, yline, zline, "gray")
    ax.set_xlabel("First Component")
    ax.set_ylabel("Second Component")
    ax.set_zlabel("Third Component")
    ax.legend()
    plt.show()

    img_array = []
    ele = -90
    for n in range(-120, 120):
        ax.view_init(n, ele)
        ele += 0.75

        # redraw the canvas
        fig.canvas.draw()

        # convert canvas to image
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        img_array.append(img)

    for ele in range(90, 120):
        ax.view_init(n, ele)
        n -= 0.75

        # redraw the canvas
        fig.canvas.draw()

        # convert canvas to image
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        height, width, layers = img.shape
        size = (width, height)

        img_array.append(img)

    tag = ""
    out = cv2.VideoWriter(
        logs_path + "vid_3d_" + ".avi", cv2.VideoWriter_fourcc(*"DIVX"), 15, size
    )

    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

    plt.clf()


# %matplotlib inline
# fig = plt.figure(figsize=(18, 16))
# ax = plt.axes(projection='3d')


# for i in target_name:

#     index = np.where(target==i)
#     # print("i,index",i,index)
# #     ax.scatter( PCn[index,0], PCn[index,1], PCn[index,2])


#     # Data for three-dimensional scattered points
#     zdata = PCn[index,0]#15 * np.random.random(100)
#     xdata =PCn[index,1]# xline#np.sin(zdata) + 0.1 * np.random.randn(100)
#     ydata =PCn[index,2]#yline# np.cos(zdata) + 0.1 * np.random.randn(100)
# ax.scatter3D(xdata, ydata, zdata, c=zdata)
# Data for a three-dimensional line
#     # Data for three-dimensional scattered points
# for index in range(10):
#     zdata = PCn[index:index+q,2]#15 * np.random.random(100)
#     xdata =PCn[index:index+q,0]# xline#np.sin(zdata) + 0.1 * np.random.randn(100)
#     ydata =PCn[index:index+q,1]#yline# np.cos(zdata) + 0.1 * np.random.randn(100)
#     # ax.scatter3D(xdata, ydata, zdata, c=zdata)
#     # data=df[df['time']==num]
#     # print("zdata",zdata)
#     # graph.axes.set_xticks(np.arange(min(list(PCn[:, 0])),max(list(PCn[:, 0])),50),rotation = 60,size=12)
#     # graph.axes.set_yticks(np.arange(min(list(PCn[:, 1])),max(list(PCn[:, 1])),500),rotation = 60,size=12)
#     # ax.axes.set_zticks(np.arange(min(list(PCn[:, 2])),max(list(PCn[:, 2])),50),rotation = 60,size=12)
#     # ax.axes.set_zticks(len(zdata))
#     # graph, = ax.plot(xdata, ydata, zdata, linestyle="-")


#     cc=round(( index+1)/q)


#     print(colr[int(cc)])
#     c= colr[int(cc)]
#     m=marker[int(cc)]
#     print("cc",cc,"m",m)
#     # graph.set_data (xdata, ydata)
#     # graph.set_marker(m)
#     # graph.set_color(c)
#     # graph.set_3d_properties(zdata)
#     title.set_text(i +' 3D Step ={}'.format(index))
#     zline =PCn[:,0] #np.linspace(0, 15, 1000)
#     xline =PCn[:,1] #np.sin(zline)
#     yline =PCn[:,2]#np.cos(zline)
#     ax.plot(xline, yline, zline,marker=m, c=c)
#     plt.xlabel("First Component")
#     plt.ylabel("Second Component")
#     plt.ylabel("Third Component")
#     plt.show()
