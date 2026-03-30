#-*- coding: utf-8 -*-
import torch
import torchvision.models as models

model = models.resnet34().cuda()

x = torch.randn(size=(1, 3, 144, 144)).cuda()

while True:
    result = model(x)
    print("运行成功")
