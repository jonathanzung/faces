#!/bin/bash
export PATH := ../:../super/:$(PATH)
all: pcdsuper

#raw: capture.config
#	rm -rf raw/
#	mkdir -p raw
#	capture -o raw/

rowm: raw raw/set.meta torowm.config
	rm -rf rowm/
	mkdir -p rowm/
	torowm -i raw/ -o rowm/
	rm -rf pcd/
	mkdir -p pcd
	topcd -i rowm/ -o pcd/ -n 0

pcdseg: rowm rowm/set.meta pcd pcd/set.meta
	rm -rf pcdseg
	mkdir -p pcdseg
	./segmentscript
	cp pcd/set.meta pcdseg/set.meta

pcdstitch: pcdseg pcdseg/set.meta stitch.config
	rm -rf pcdstitch/
	mkdir -p pcdstitch
	mkdir pcdstitch/orientations
	stitch -i pcdseg/ -o pcdstitch/

mesh.stl: pcdstitch pcdstitch/merged.pcd graft.config
	graft -i pcdstitch/merged.pcd -o ./
	#view.py mesh.stl

pcdsuper: mesh.stl super.config
	rm -rf pcdsuper
	mkdir -p pcdsuper
	mkdir -p pcdsuper/orientations
	super -i pcdstitch/ -o pcdsuper/ -r pcdstitch/orientations/ -s mesh.stl
	#view.py pcdsuper/mesh.stl

pcdgen: gen.config graft.config
	rm -rf pcdgen/
	mkdir -p pcdgen
	mkdir pcdgen/orientations
	mkdir pcdgen/scans
	gen -o pcdgen/
	graft -i pcdgen/merged.pcd -o pcdgen/

pcdsupergen: pcdgen super.config
	rm -rf pcdsupergen
	mkdir -p pcdsupergen
	mkdir -p pcdsupergen/orientations
	super -i pcdgen/scans/ -o pcdsupergen/ -r pcdgen/orientations/ -s pcdgen/mesh.stl
	compare pcdsupergen/spine_initial.pcd pcdgen/spine.pcd
	compare pcdsupergen/spine.pcd pcdgen/spine.pcd
	pcl_viewer -ps 3 pcdsupergen/spine.pcd pcdgen/spine.pcd pcdgen/merged.pcd
	#openf pcdsupergen/mesh.stl

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
