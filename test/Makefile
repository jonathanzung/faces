#!/bin/bash

export PATH := ../:../super/:$(PATH)
all: mesh.stl

raw: capture.config
	rm -rf raw/
	mkdir -p raw
	capture -o raw/
rowm: raw raw/set.meta torowm.config
	rm -rf rowm/
	mkdir -p rowm/
	torowm -i raw/ -o rowm/
	rm -rf pcd/
	mkdir -p pcd
	topcd -i rowm/ -o pcd/ -n 0

pcdstraight: rowm rowm/set.meta straighten.config
	rm -rf straight/
	mkdir -p straight/
	straighten -i rowm/ -o straight/
	rm -rf pcdstraight/
	mkdir -p pcdstraight
	topcd -i straight/ -o pcdstraight/ -n 0

pcdstitch: pcdstraight pcdstraight/set.meta stitch.config
	rm -rf pcdstitch/
	mkdir -p pcdstitch
	mkdir pcdstitch/orientations
	stitch -i pcdstraight/ -o pcdstitch/

mesh.stl: pcdstitch pcdstitch/merged.pcd graft.config
	graft -i pcdstitch/merged.pcd -o ./
	view.py mesh.stl

pcdsuper: mesh.stl super.config
	rm -rf pcdsuper
	mkdir -p pcdsuper
	super -i pcdstraight/ -o pcdsuper/ -r pcdstitch/orientations/ -s mesh.stl

norm: rowm rowm/set.meta normal.config
	rm -rf norm/
	mkdir -p norm
	mkdir -p norm/sample
	mkdir -p norm/samplenormal
	normal -i rowm/ -o norm/
	rm -rf pcdnorm/
	mkdir -p pcdnorm
	topcd -i norm/ -o pcdnorm/ -n 1

calibrate: norm norm/sample/set.meta trace.config
	rm -rf reg/
	rm -rf pcdreg/
	mkdir -p reg/
	mkdir -p reg/sample
	mkdir -p reg/samplenormal
	mkdir -p pcdreg/
	trace -i norm/ -o reg/
	topcd -i reg/ -o pcdreg/ -n 1

clean:
	touch torowm.config
	touch normal.config
	touch trace.config
	touch topcd.config
	touch stitch.config
	touch trace.config
	touch graft.config
	touch super.config

cleanall:
	touch capture.config
	touch torowm.config
	touch normal.config
	touch trace.config
	touch topcd.config
	touch stitch.config
	touch trace.config
	touch graft.config
	touch super.config
