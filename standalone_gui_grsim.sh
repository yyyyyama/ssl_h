# grSim 用

# Vision と Refbox を受信するインターフェースのアドレス
interface_receive=192.168.10.4

./build/standalone-gui \
--vision-address 224.5.23.2 \
--vision-port 10020 \
--vision-if-address ${interface_receive} \
--refbox-address 224.5.23.1 \
--refbox-port 10003 \
--refbox-if-address ${interface_receive} \
--simproto-address 127.0.0.1 \
--simproto-sim-port 10300 \
--simproto-blue-port 10301 \
--simproto-yellow-port 10302 \
--radio simproto-all