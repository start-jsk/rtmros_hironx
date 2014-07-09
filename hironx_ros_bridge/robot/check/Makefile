FREEZE=/home/sam/hiro-nxo_sys-check/freeze_bin/usr/share/doc/python2.7/examples/Tools/freeze/freeze.py
CURDIR:=$(shell pwd)
TMPDIR:=$(shell mktemp -d)
DATE:=$(shell date +'%y-%m-%d')
PYFILES:=$(wildcard *.py)

all: robot-system-check-base-${DATE}

robot-system-check-base-${DATE}: $(PYFILES)
	cd ${TMPDIR} && python ${FREEZE} ${CURDIR}/robot-system-check-base.py && make
	cp ${TMPDIR}/robot-system-check-base ${CURDIR}/robot-system-check-base-${DATE}
	rm -fr ${TMPDIR}
