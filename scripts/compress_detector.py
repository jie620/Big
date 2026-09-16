#!/usr/bin/env python3
"""Structured channel pruning plus detector-head distillation for YOLO11/YOLOv8.

Requires torch-pruning. The detection head is preserved so teacher and student
raw outputs have identical semantics. Only external Conv blocks are pruning
roots; internal split/attention blocks retain their channel invariants.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import torch
import torch.nn as nn
import torch.nn.functional as F
from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionTrainer
from ultralytics.nn.modules import Conv, Detect


def raw_maps(output):
    return output[1] if isinstance(output,tuple) else output


def distill(student,teacher,weight):
    outputs=raw_maps(student);targets=raw_maps(teacher)
    if len(outputs)!=len(targets):raise ValueError("teacher/student detection scales differ")
    losses=[]
    for output,target in zip(outputs,targets):
        if output.shape!=target.shape:raise ValueError("teacher/student head shape differs")
        losses.append(F.mse_loss(output.float(),target.detach().float()))
    return weight*sum(losses)/len(losses)


def prune(model,ratio,imgsz):
    import torch_pruning as tp
    model.eval()
    for p in model.parameters():p.requires_grad_(True)
    sample=torch.zeros(1,3,imgsz,imgsz,device=next(model.parameters()).device)
    before=sum(p.numel() for p in model.parameters())
    # Roots are standalone stage downsampling convolutions. Dependency graph
    # propagates their removals to consumers; no split/attention internals pruned.
    roots=[block.conv for block in model.model if type(block) is Conv]
    graph=tp.DependencyGraph().build_dependency(model,example_inputs=sample)
    for layer in roots:
        count=int(layer.out_channels*ratio)//8*8
        if count<8 or layer.out_channels-count<8:continue
        importance=layer.weight.detach().abs().mean(dim=(1,2,3))
        indices=torch.argsort(importance)[:count].tolist()
        group=graph.get_pruning_group(layer,tp.prune_conv_out_channels,idxs=indices)
        if not graph.check_pruning_group(group):raise RuntimeError("Invalid pruning dependency group")
        group.prune()
        # Rebuild after structural mutation; do not reuse stale dependency indices.
        graph=tp.DependencyGraph().build_dependency(model,example_inputs=sample)
    with torch.no_grad():model(sample)
    after=sum(p.numel() for p in model.parameters())
    if after>=before:raise RuntimeError("No channels removed")
    return {"parameters_before":before,"parameters_after":after,"parameter_reduction":1-after/before}


class DistillationTrainer(DetectionTrainer):
    def __init__(self,student,teacher,kd_weight,**kwargs):
        self.student=student
        self.teacher=teacher.eval().requires_grad_(False)
        self.kd_weight=kd_weight
        self.handles=[]
        super().__init__(**kwargs)
    def get_model(self,cfg=None,weights=None,verbose=True):
        # Keep the physically pruned modules; rebuilding from YAML loses pruning.
        return self.student
    def _setup_train(self,world_size):
        if world_size>1:raise ValueError("Compression trainer currently supports one GPU")
        super()._setup_train(world_size)
        self.teacher.to(self.device)
        self.student_raw=None
        self.install_hooks()
    def save_model(self):
        # Hooks close over teacher/trainer and must never be pickled into weights.
        handles=self.handles
        for handle in handles:handle.remove()
        try:super().save_model()
        finally:self.handles=[]
        # Resume hooks through a small dedicated installer.
        self.install_hooks()
    def install_hooks(self):
        def capture(module,inputs,output):self.student_raw=output
        def augment(module,inputs,output):
            if not module.training or not isinstance(inputs[0],dict):return output
            with torch.no_grad():teacher=self.teacher(inputs[0]["img"])
            return output[0]+distill(self.student_raw,teacher,self.kd_weight),output[1]
        self.handles=[self.model.model[-1].register_forward_hook(capture),self.model.register_forward_hook(augment)]


def main():
    p=argparse.ArgumentParser();p.add_argument("teacher");p.add_argument("--data",required=True);p.add_argument("--output",type=Path,required=True)
    p.add_argument("--ratio",type=float,default=0.25);p.add_argument("--epochs",type=int,default=50);p.add_argument("--batch",type=int,default=4)
    p.add_argument("--imgsz",type=int,default=640);p.add_argument("--device",default="0");p.add_argument("--kd-weight",type=float,default=0.1)
    p.add_argument("--prune-only",action="store_true")
    a=p.parse_args()
    if not 0<a.ratio<0.8 or a.kd_weight<0 or a.epochs<1:p.error("invalid compression parameters")
    if a.output.exists():p.error("output exists; choose a new run directory")
    teacher=YOLO(a.teacher).model.float();student=deepcopy(teacher)
    report=prune(student,a.ratio,a.imgsz)
    a.output.mkdir(parents=True)
    (a.output/"pruning.json").write_text(json.dumps(report,indent=2)+"\n")
    # Ultralytics can reload a serialized pruned nn.Module without YAML rebuild.
    artifact=YOLO(a.teacher);artifact.model=student;artifact.save(str(a.output/"pruned.pt"))
    if not a.prune_only:
        trainer=DistillationTrainer(student,teacher,a.kd_weight,overrides={"model":a.teacher,"data":a.data,"epochs":a.epochs,"batch":a.batch,"imgsz":a.imgsz,"device":a.device,"project":str(a.output),"name":"distilled","amp":False,"workers":0,"exist_ok":False})
        trainer.train()
    print(json.dumps(report,indent=2))

if __name__=="__main__":main()
