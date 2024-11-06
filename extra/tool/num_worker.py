from time import time
import torch
import multiprocessing as mp
for num_workers in range(2, mp.cpu_count(), 2):  
    train_loader = torch.utils.data.DataLoader(train_reader,shuffle=True,num_workers=num_workers,batch_size=64,pin_memory=True)
start = time()
for epoch in range(1, 3):
    for i, data in enumerate(train_loader, 0):
        pass
end = time()
print("Finish with:{} second, num_workers={}".format(end - start, num_workers))