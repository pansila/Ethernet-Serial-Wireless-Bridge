# Ethernet-Serial-Wireless Bridge

## Topology

![](https://i.loli.net/2018/03/28/5abb1fe7b7812.png)

## Usage

1. Compile the project

  ```bash
   [root@localhost]#make
  ```

2. load the module

  First, creat the character devices:

  ```bash
   [root@localhost]#mknod /dev/vd_rx c 200 0
   [root@localhost]#mknod /dev/vd_tx c 201 0
  ```

  then, let the vd_load and vd_unload can execute:

  ```bash
   [root@localhost]#chmod +x vd_unload
   [root@localhost]#chmod +x vd_load
  ```

  last, load the driver:

  ```bash
   [root@localhost]#./vd_load
  ```

  You can change the IP address in vd_load script file

3. run server
  ```bash
   [root@localhost]#./server
  ```

  now you can use the veth0 device and send and receive data through it.:)

---
Reference:
[Linux serial-to-ethernet bridge](https://www.ibm.com/developerworks/cn/linux/l-serialnet/index.html)
