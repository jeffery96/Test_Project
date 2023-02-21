import cantools
import can
from zcan.zcan import ZCanBus
from can import Message

test_msg = Message(arbitration_id=0x18FFE6A5, data=[1, 2, 3, 4, 5], channel=0)
# print(test_msg)

# with ZCanBus(bitrate=250) as bus:
#     bus.send(test_msg)
#     for msg in bus:
#         print('收到报文：')
#         print(msg)

bus = ZCanBus(bitrate=250)
bus.send(test_msg)
for msg in bus:
    print('收到报文：')
    print(msg)

# dbc = cantools.database.load_file(r'C:\Users\lin_xiaobin\Desktop\澳洲物流车WSD5050HR1EV\CANoe工程\WSD5050HR1EV_v0.5.dbc', encoding='gb2312')
# bus = can.interface.Bus(bustype='canalystii', channel=0, bitrate=250000)
pass
