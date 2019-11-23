#/bin/bash
echo "petkit_make.sh version 20191109"
echo ""


pk_base_path=$(cd `dirname $0`; pwd)
echo ${pk_base_path}
sdk_path="$(realpath ../SDK/esp-idf)"
echo ${sdk_path}

cd ${sdk_path}
./install.sh

basedir="$(dirname $0)"
echo "All done! You can now run:"
echo ""
echo "  . ${basedir}/export.sh"
echo ""
. ./export.sh
echo "finish export.sh"


cd ${pk_base_path}
if [ "$1" == "clean" ]; then
    make clean
elif [ "$1" == "menuconfig" ]; then
    make menuconfig
elif [ "$1" == "flash" ]; then
    make flash
else
    make
fi

