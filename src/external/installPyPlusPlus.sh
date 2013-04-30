#!/bin/sh

set -e

# create a location to store downloaded data
mkdir -p /wg/stor5/mpomarlan/ompl/build/pyplusplus
cd /wg/stor5/mpomarlan/ompl/build/pyplusplus

# get sources
# gccxml snapshot of 11/6/2012
curl --location-trusted https://github.com/gccxml/gccxml/archive/2cbeb9d631e0198fcbeca3d230ef49fe07e87dd8.tar.gz | tar xzf -
curl --location-trusted https://bitbucket.org/ompl/pygccxml/downloads/pygccxml-r575.tgz | tar xzf -
curl --location-trusted https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1238.tgz | tar xzf -

# build & install gccxml
cd gccxml-2cbeb9d631e0198fcbeca3d230ef49fe07e87dd8
/usr/bin/cmake  .
/usr/bin/cmake --build .
sudo /usr/bin/cmake --build . --target install

# build & install pygccxml and Py++
cd ../pygccxml
/usr/bin/python setup.py build
sudo /usr/bin/python setup.py install
cd ../pyplusplus
/usr/bin/python setup.py build
sudo /usr/bin/python setup.py install
