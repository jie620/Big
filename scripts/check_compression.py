#!/usr/bin/env python3
import torch
from compress_detector import distill
student=[torch.ones(1,8,4,4,requires_grad=True)]
teacher=[torch.zeros(1,8,4,4)]
loss=distill(student,teacher,0.1)
assert abs(loss.item()-0.1)<1e-6
loss.backward()
assert student[0].grad is not None and torch.isfinite(student[0].grad).all()
print("distillation gradient check passed")
