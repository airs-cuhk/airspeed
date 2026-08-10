# ROH Serial API Example

Example for ROH simple control.

## 1. Install PCAN Driver

Download from [PCAN Drivers](https://peak-system.com.cn/driver/) according to your system.

## 2. Preparation

### 2.1 Install PCAN-Basic lib

Windows:

Download from https://www.peak-system.com/PCAN-Basic.126.0.html?&L=1 

Put files to the same partition as your ohand_serial_sdk, e.g., d:\

Make dirs/files look as d:\PCAN-Basic

---

Linux:

Download from https://www.peak-system.com/PCAN-Basic-Linux.433.0.html?&L=1

```BASH
tar -xzf PCAN-Basic_Linux-4.10.0.4.tar.gz
cd PCAN-Basic_Linux-4.10.0.4/libpcanbasic/pcanbasic
make clean
sudo make && sudo make install
```

## 3. Run

### 3.1 Run simple_control

* Open the `simple_control.py` file and modify the communication interface and device address as needed, for example:

```python
PORT_TYPE = PORT_UART
ADDRESS_HAND = 2
```

* Run the program:

Windows:

```BASH
python simple_control.py
```

Linux:

``` BASH
sudo chmod o+rw /dev/ttyUSB0
python3 simple_control.py
```

* Press 'ctrl-c' to exit the program.

### 3.2 Run dual_rohand_control

* Open the `dual_rohand_control.py` file and modify the  device address as needed, for example:

```python
HAND_ID = [0x02, 0x03]
```

* Run the program:

Windows:

```BASH
python dual_rohand_control.py
```

Linux:

``` BASH
sudo chmod o+rw /dev/ttyUSB0
python3 dual_rohand_control.py
```

* Press 'ctrl-c' to exit the program.
