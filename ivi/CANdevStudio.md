ref: https://github.com/GENIVI/CANdevStudio

### pre install
```sh
sudo apt install libqt5serialbus5-dev
sudo apt install libqt5svg5-dev
```

### install
```sh
git clone https://github.com/GENIVI/CANdevStudio.git
cd CANdevStudio
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

### Run
```sh
CANdevStudio
```
