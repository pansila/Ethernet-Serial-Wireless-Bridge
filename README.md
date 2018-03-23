# Ethernet-Serial-Wireless Bridge

1. Compile the project

   [root@localhost]#make

2. load the module

First, creat the character devices:
   [root@localhost]#mknod /dev/vd_rx c 200 0
   [root@localhost]#mknod /dev/vd_tx c 201 0

then, let the vd_load and vd_unload can execute:
   [root@localhost]#chmod +x vd_unload
   [root@localhost]#chmod +x vd_load

last, load the driver:
   [root@localhost]#./vd_load
You can change the IP address in vd_load script file

3. run server
   [root@localhost]#./server

now you can use the veth0 device and send and receive data 
through it.:)

