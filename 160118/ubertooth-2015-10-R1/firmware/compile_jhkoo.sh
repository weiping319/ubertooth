make clean 
make bluetooth_rxtx
cp bluetooth_rxtx/bluetooth_rxtx.hex ../ubertooth-one-firmware-bin/
cp bluetooth_rxtx/bluetooth_rxtx.bin ../ubertooth-one-firmware-bin/
cp bluetooth_rxtx/bluetooth_rxtx.dfu ../ubertooth-one-firmware-bin/
cd ..
#cd ../ubertooth-one-firmware-bin
#sudo ubertooth-dfu -d bluetooth_rxtx.dfu -r
#sudo ubertooth-dfu --write bluetooth_rxtx.dfu
#sudo ubertooth-dfu --detach
