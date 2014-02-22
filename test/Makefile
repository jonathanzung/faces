#!/bin/bash

export PATH := ../bin:$(PATH)
n=50

raw: capture.config
	rm -rf raw/
	mkdir -p raw
	capture -o raw/ -n $(n)
seg: raw raw/0.pcd behead.config
	rm -rf seg/
	mkdir -p seg
	behead -i raw/ -o seg/ -n $(n)
stitch: seg seg/0.pcd stitch.config
	rm -rf stitch/
	mkdir -p stitch
	stitch -i seg/ -o stitch/ -n $(n)
mesh.stl: stitch stitch/0.pcd graft.config
	graft -i stitch/merged.pcd -o ./
	view.py mesh.stl
super: stitch/0.pcd mesh.stl super.config
	mkdir -p super
	super -i stitch/ -o super/ -s mesh.stl -n $(n)
superiter: super.config
	super -i super/ -o super/ -s super/mesh.stl -n $(n)

clean:
	rm -r raw
	rm -r seg
	rm -r stitch
	rm mesh.stl
