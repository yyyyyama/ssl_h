
# 実機用

# Vision と Refbox を受信するインターフェースのアドレス
interface_receive=192.168.10.4
# ロボットにコマンドを送信する用のインターフェイスのアドレス
interface_robot=192.168.10.4

./build/standalone-gui \
--vision-address 224.5.23.2 \
--vision-port 10006 \
--vision-if-address ${interface_receive} \
--refbox-address 224.5.23.1 \
--refbox-port 10003 \
--refbox-if-address ${interface_receive} \
--kiks-address 224.5.23.2 \
--kiks-port 10004 \
--kiks-if-address ${interface_robot} \
--radio humanoid
