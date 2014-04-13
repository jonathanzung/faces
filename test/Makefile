#!/bin/bash

export PATH := ../:$(PATH)
all: norm

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

straighten: rowm rowm/set.meta straighten.config
	rm -rf straighten/
	mkdir -p straighten/
	straighten -i rowm/ -o straighten/
	rm -rf spcd/
	mkdir -p spcd
	topcd -i straighten/ -o spcd/ -n 0

stitch: rowm rowm/set.meta stitch.config
	rm -rf stitch/
	mkdir -p stitch
	stitch -i spcd/ -o stitch/ -n 20

norm: rowm rowm/set.meta normal.config
	rm -rf norm/
	mkdir -p norm
	mkdir -p norm/sample
	mkdir -p norm/samplenormal
	normal -i rowm/ -o norm/
	rm -rf pcdnorm/
	mkdir -p pcdnorm
	topcd -i norm/ -o pcdnorm/ -n 1
straightenconfig: norm norm/sample/set.meta trace.config
	rm -rf reg/
	trace -i norm/ -o reg/

clean:
	touch torowm.config
	touch normal.config
	touch trace.config
	touch topcd.config
	touch stitch.config
	touch trace.config
cleanall:
	touch capture.config
	touch torowm.config
	touch normal.config
	touch trace.config
	touch topcd.config
	touch stitch.config
