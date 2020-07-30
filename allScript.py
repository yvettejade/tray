import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as Data
from torch.autograd import Variable 
import numpy as np
from torch.utils.data import TensorDataset, DataLoader
import sklearn.preprocessing
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_absolute_error
from datasets import SyntheticSplit
from datasets import SyntheticDataset


class NN(nn.Module):
    def __init__(self,hS1,hS2):
            super(NN, self).__init__()
            # params
            self.inputSize = 2005
            self.hiddenSize1 = hS1
            self.hiddenSize2 = hS2
            self.outputSize =  3
            
            self.linear1 = nn.Linear(self.inputSize, self.hiddenSize1)
            self.linear2 = nn.Linear(self.hiddenSize1, self.hiddenSize2)
            self.linear3 = nn.Linear(self.hiddenSize2, self.outputSize)

    def forward(self, x):
        x = self.linear1(x)
        x = F.relu(x)
        x = self.linear2(x)
        x = F.relu(x)
        x = self.linear3(x)
        return x

def run(hS1, hS2,tInputs,tTargets,pInputs,pTargets):
    model = NN(hS1, hS2)

    opt = optim.SGD(model.parameters(), lr=1e-8)
    loss_fn = F.mse_loss

    loss = loss_fn(model(tInputs), tTargets)


    for epoch in range(500):
        # Generate predictions
        pred = model(tInputs)
        loss = loss_fn(pred, tTargets)
        # Perform gradient descent
        loss.backward()
        opt.step()
        opt.zero_grad()

    x=loss_fn(model(tInputs), tTargets)

    preds = model(pInputs)
    y=mean_absolute_error(pTargets.detach().numpy(), preds.detach().numpy())
    return x,y

def main():
    gen = SyntheticDataset()
    data = gen.split()
    trainPercent = 0.7
    predPercent = 0.2
    validPercent = 0.1

    # train
    trainInputs, b = data[0].tensors()
    tInputs = torch.FloatTensor(trainInputs)
    tInputs = tInputs.view(int(gen.__len__() * trainPercent), -1)
    
    trainTargets = []
    for a in range(int(gen.__len__() * trainPercent)):
        resultTuple = (data[0].quantitative_tensor('infectiousness')[a], data[0].quantitative_tensor('i_out')[a], data[0].quantitative_tensor('i_rec_prop')[a])
        trainTargets.append(resultTuple)
    tTargets = torch.FloatTensor(trainTargets)

    # test
    predInputs, c = data[1].tensors()
    pInputs = torch.FloatTensor(predInputs)
    pInputs = pInputs.view(int(gen.__len__() * predPercent), -1)
    
    predTargets = []
    for a in range(int(gen.__len__() * predPercent)):
        resultTuple = (data[1].quantitative_tensor('infectiousness')[a], data[1].quantitative_tensor('i_out')[a], data[1].quantitative_tensor('i_rec_prop')[a])
        predTargets.append(resultTuple)
    pTargets = torch.FloatTensor(predTargets)
    
    # validation
    '''validInputs, c = data[2].tensors()
    vInputs = torch.FloatTensor(validInputs)
    vInputs = vInputs.view(int(gen.__len__() * validPercent), -1)
    
    validTargets = []
    for a in range(int(gen.__len__() * validPercent)):
        resultTuple = (data[2].quantitative_tensor('infectiousness')[a], data[2].quantitative_tensor('i_out')[a], data[2].quantitative_tensor('i_rec_prop')[a])
        validTargets.append(resultTuple)
    vTargets = torch.FloatTensor(validTargets)'''

    f = open("results.txt","w+")
    for a in range(5000):
        for b in range(5000):
            avgX=0
            avgY=0
            if (a != 0 and b != 0) and (a % 100 == 0 and b % 100 == 0):
                x,y=run(a,b,tInputs,tTargets,pInputs,pTargets)
                x1,y1=run(a,b,tInputs,tTargets,pInputs,pTargets)
                x2,y2=run(a,b,tInputs,tTargets,pInputs,pTargets)
                x3,y3=run(a,b,tInputs,tTargets,pInputs,pTargets)
                x4,y4=run(a,b,tInputs,tTargets,pInputs,pTargets)
                avgX=x+x1+x2+x3+x4
                avgY=y+y1+y2+y3+y4
                avgX/=5
                avgY/=5
                f.write(str(a) + " " + str(b) + ": "+str(avgX) + " " + str(avgY) + "\n")



if __name__ == "__main__":
    main()