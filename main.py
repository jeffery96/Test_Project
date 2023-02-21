from collections.abc import Iterator

import cantools
import can
from zlgcan.zlgcan_interface import ZlgCanBus
from can import Message

test_msg = Message(arbitration_id=0x18FFE6A5, data=[1, 2, 3, 4, 5], channel=0)
# print(test_msg)

# with ZCanBus(bitrate=250) as bus:
#     bus.send(test_msg)
#     for msg in bus:
#         print('收到报文：')
#         print(msg)

bus = ZlgCanBus(bitrate='250K')
# bus.send(test_msg)
# bus.send_periodic(test_msg, 0.1, 1)
# for msg in bus:
#
#     print(msg)
# rcv_msg = bus.recv()
# print(rcv_msg)

# bus.shutdown()

# dbc = cantools.database.load_file(r'C:\Users\lin_xiaobin\Desktop\澳洲物流车WSD5050HR1EV\CANoe工程\WSD5050HR1EV_v0.5.dbc', encoding='gb2312')
# bus1 = can.interface.Bus('test', bustype='virtual')
# bus2 = can.interface.Bus('test', bustype='virtual')
# msg1 = can.Message(arbitration_id=0xabcde, data=[1,2,3])
# bus1.send(msg1)
# # for msg in bus1:
# #     print(msg)
# msg2 = bus2.recv()
# print(msg2)

pass
