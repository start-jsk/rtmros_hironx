#! /bin/sh

rm -rf _build
mkdir _build
(cd _build ; cmake -DCMAKE_INSTALL_PREFIX=/opt/nextage-open ..; make VERBOSE=1; make install)
#(cd _build ; cmake -DCMAKE_INSTALL_PREFIX=/tmp/hrpsys-base ..; make VERBOSE=1; make install)
