from qpth.qp import QPFunction

import torch
import torch.nn as nn
from torch.autograd import Function, Variable
from torch.nn.parameter import Parameter
import torch.nn.functional as F

from torch.utils.data import Dataset

import scipy.io as sio

# Class to learn a QP from data
#
# in_features - dimension of the parameter (state)
# out_features - dimension of the output (control input)
# nVar - dimension of the QP to learn
#
# Given a parameter x in R^{nBatch x in_features}, the output of the QP is given by
#
#  f = linmap_f(x)
#  z = min 0.5 z'*(L'*L + eps*I)*z + f'*z s.t. z >= 0
#  out = linmap_out(z)
#
# Parameters learned: L, linmap_f, linmap_out
# Note that linmap is affine

class QPNet(nn.Module):
    def __init__(self, nVar, in_features, out_features, eps=1e-4, maxIter=5):
        super().__init__()
        self.eps = eps
        self.nVar = nVar
        self.in_features = in_features
        self.out_features = out_features
        self.maxIter = maxIter

        # Gz <= h => z >= 0
        self.H = Parameter(torch.randn(nVar,nVar))
        self.meye = Variable(-torch.eye(nVar))
        self.h = Variable(torch.zeros(nVar))
        self.I = Variable(torch.eye(self.nVar))
        self.e = Variable(torch.Tensor())

        # Input linear layer 
        self.InMap = nn.Linear(in_features=in_features, out_features=nVar) 

        # Output linear layer
        self.OutMap = nn.Linear(in_features=nVar, out_features=out_features) 

        # Simplified projection layer (see paper Section 3.2)
        self.umin = 0
        self.umax = 1

    def to(self, device):
        new_self = super(QPNet, self).to(device)
        new_self.H = new_self.H.to(device)
        new_self.meye = new_self.meye.to(device)
        new_self.h = new_self.h.to(device)
        new_self.I = new_self.I.to(device)
        new_self.e = new_self.e.to(device)
        return new_self

    def forward(self, x):
        y1 = self.InMap(x)      
        y2 = QPFunction(verbose=-1, eps=self.eps, maxIter=self.maxIter)(1e-3 * self.I + self.H.t().mm(self.H), \
            self.InMap(x), self.meye, self.h, self.e, self.e)
        y3 = self.OutMap(y2)

        u = torch.clamp(y3, self.umin, self.umax)
        return u 

    def prepQP(self):
        self.Q = self.H.t().mm(self.H) + 1e-3 * torch.eye(self.nVar)
        self.e = Variable(torch.Tensor())
    
    def evalQP(self, x):
        return QPFunction(verbose=-1, eps=self.eps, maxIter=self.maxIter)(self.Q, self.f(x), self.meye, self.h, self.e, self.e)

    def save(self, datafile):
        def get_numpy(param):
            return param.detach().cpu().numpy()

        dat = {'F':get_numpy(self.InMap.weight), 'f':get_numpy(self.InMap.bias),
            'H':get_numpy(self.H),
            'G':get_numpy(self.OutMap.weight), 'g':get_numpy(self.OutMap.bias),
            'maxIter':self.maxIter,
            'type':'QPNet'}
        sio.savemat('../data/' + datafile + '.mat', dat)


class QPDataset(Dataset):
    def __init__(self, mat_file):
        mat = sio.loadmat('../data/' + mat_file + '.mat')
        self.x = torch.tensor(mat['X'].T, requires_grad=False).type(torch.float32)
        self.y = torch.tensor(mat['U'].T, requires_grad=False).type(torch.float32)

        q = mat['U'][0]
        cnt = sum(1 for p in q if (p < 1.0 and p > -1.0))

    def __len__(self):
        return self.x.shape[0]

    def __getitem__(self, idx):
        return self.x[idx,:], self.y[idx,:]
