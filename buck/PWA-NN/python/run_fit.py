import os
import sys
import csv
print(sys.version)

import numpy as np
import timeit
import torch
import scipy.io as sio
import pandas as pd

from qpfit import QPNet
from qpfit import QPDataset
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn

torch.set_num_threads(os.cpu_count())
print("Number of threads = " + str(torch.get_num_threads()) + "\n")

param = dict()
param['nVar'] = int(sys.argv[1]) # num of parameters to be used n_z
param['maxIter'] = 10
param['batch_size'] = 50
param['numEpoch'] = 2#150
param['log_interval'] = 10000
param['learning_rate'] = 1e-3

print("==================================\n")
print("Running with:") 
print("number of variables (n_z) = %i" % param['nVar'])
print("number of epochs = %i" % param['numEpoch'])
print("batch size = %i" % param['batch_size'])
print("learning rate = %f \n" % param['learning_rate'])
print("==================================\n")

datafile = 'MATLABsamples'
savefile = 'NNparams' 

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# device = 'cuda:0'
# device = 'cpu'

qp_loader = QPDataset(datafile)
dataloader = DataLoader(qp_loader, batch_size=param['batch_size'],
                        shuffle=True, num_workers=torch.get_num_threads(), pin_memory=True)

# Initialize the model
nParam = qp_loader.x.shape[1]
nInputs = qp_loader.y.shape[1]

model = QPNet(param['nVar'], in_features=nParam, out_features=nInputs, maxIter=param['maxIter'])

model = model.to(device)
loss_fn = torch.nn.MSELoss()

# Train
numEpoch = param['numEpoch']
log_interval = param['log_interval']

# Initialize the optimizer
learning_rate = param['learning_rate']
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

print('Training initialized')
model.train()

loss_vector = []
for epoch in range(numEpoch):
    for i_batch, (x, y) in enumerate(dataloader):
        x, y = x.to(device), y.to(device)

        def closure():
            if torch.is_grad_enabled():
                optimizer.zero_grad()
            y_pred = model(x)
            loss = loss_fn(y_pred, y)
            if loss.requires_grad:
                loss.backward()
            return loss

        optimizer.step(closure)

        if i_batch % log_interval == 0:
            with torch.no_grad():
                loss = closure()
            print('Train Epoch: {}  Loss: {:.6f}'.format(
                 epoch, loss.item()))

            loss_vector.append(loss.item())
            
    filename = savefile + '_var_' + str(param['nVar']) + '_epoch_' + str(epoch)
    model.save(filename)

print("\n###########################################\n")
print("Training complete!")
print("best epoch: " + str(np.argmin(loss_vector)))
print("minimal loss = " + str(min(loss_vector)) + "\n")
print("###########################################\n")

results = {'nVar': [param['nVar']],
           'numEpochs': [param['numEpoch']],
           'bestEpoch': [np.argmin(loss_vector)],
           'loss': [min(loss_vector)]}
struct = pd.DataFrame(results)
logfile = '../data/info.csv'

try:
    pd.read_csv(logfile)
    struct.to_csv(logfile, mode='a', header=False, index=False)

except:
    struct.to_csv(logfile, mode='a', index=False)