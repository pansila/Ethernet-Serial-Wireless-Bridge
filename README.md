# Ethernet-Serial-Wireless Bridge

## Topology

![](https://i.loli.net/2018/03/28/5abb1fe7b7812.png)

## Usage

1. Compile the project

   ```bash
   # make
   # make server
   ```

2. Apply the execution permission to vd_load and vd_unload:

   ```bash
   # chmod +x vd_unload
   # chmod +x vd_load
   # chmod +x server
   ```

3. Load the driver:

   ```bash
   # ./vd_load
   ```

   You can change the IP address in vd_load script file

3. run server

   ```bash
   # ./server
   ```

   now you can use the veth0 device and send and receive data through it.:)

---
Reference:
[Linux serial-to-ethernet bridge](https://www.ibm.com/developerworks/cn/linux/l-serialnet/index.html)
