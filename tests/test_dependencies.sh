#!/bin/sh
set -eux
# test=/tmp/madcad-test
test=$(mktemp -d)
repo=$(realpath -m $0/../..)

# (re)create env
if [ -d $test ]; then rm -r $test; fi
python -m venv $test
cd $test

# setup env
. $test/bin/activate
# python -m pip install wheel
python -m pip install $repo pyside6

# madcad package
python -c "import madcad"
python -c "import madcad.core"
python -c "import madcad.rendering.d3"
python -c "import madcad.text"

# madcad docs
python -m pip install $(cat $repo/docs/requirements.txt)
python -m sphinx $repo/docs $test/docs

# clean
deactivate
rm -r $test
