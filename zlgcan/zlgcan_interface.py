import collections
import json
import time
from ctypes import c_ubyte
from typing import List, Optional, Tuple, Any, Union, Sequence, Dict

import can
from can import Message, BitTiming, CanTimeoutError
from can.bus import BusABC
from can.typechecking import CanFilters

import zlgcan.zlgcan as driver


class ZlgCanBus(BusABC):

    def __init__(
            self,
            interface: str = 'USBCAN-II',
            channel: Union[int, Sequence[int], str] = (0, 1),
            dev_index: int = 0,
            bitrate: str = '250K',
            # bitrate: Optional[int] = None,
            bit_timing: Optional[BitTiming] = None,
            can_filters: Optional[CanFilters] = None,
            rx_queue_size: Optional[int] = None,
            **kwargs: Dict[str, Any],
    ):
        """
        :param interface:
            The ZLGCAN device type.
        :param channel:
            Optional channel number, list/tuple of multiple channels, or comma
            separated string of channels. Default is to configure both
            channels.
        :param dev_index:
            Optional USB device number. Default is 0 (first device found).
        :param bitrate:
            CAN bitrate in bits/second. Required unless the bit_timing argument is set.
        :param bit_timing:
            Optional BitTiming instance to use for custom bit timing setting.
            If this argument is set then it overrides the bitrate argument.
        :param can_filters:
            Optional filters for received CAN messages.
        :param rx_queue_size:
            If set, software received message queue can only grow to this many
            messages (for all channels) before older messages are dropped
        """
        super().__init__(channel=channel, can_filters=can_filters, **kwargs)

        if not (bitrate or bit_timing):
            raise ValueError("Either bitrate or bit_timing argument is required")

        if isinstance(channel, str):
            # Assume comma separated string of channels
            self.channels = [int(ch.strip()) for ch in channel.split(",")]
        elif isinstance(channel, int):
            self.channels = [channel]
        else:  # Sequence[int]
            self.channels = list(channel)

        self.rx_queue = collections.deque(
            maxlen=rx_queue_size
        )  # type: Deque[Tuple[int, driver.Message]]

        self.channel_info = f"U{interface}: device {dev_index}, channels {self.channels}"

        self.device = driver.ZCAN()
        self._dev_info = None  # 设备配置信息文件
        self._dev_handle = None  # CAN设备句柄
        self._chn_handle = {}  # CAN通道句柄

        with open("./zlgcan/dev_info.json", "r") as fd:
            self._dev_info = json.load(fd)
        if self._dev_info is None:
            raise ValueError("dev_info.json no exist!")

        # 打开设备
        self._dev_handle = self.device.OpenDevice(
            self._dev_info[interface]["dev_type"], dev_index, 0)

        chn_cfg = driver.ZCAN_CHANNEL_INIT_CONFIG()
        # 暂不支持CANFD设备
        chn_cfg.can_type = driver.ZCAN_TYPE_CAN
        # 0表示正常模式，能收能发
        chn_cfg.config.can.mode = 0
        bitrate = bitrate.upper()
        brt = self._dev_info[interface]["chn_info"]["baudrate"][bitrate]
        chn_cfg.config.can.timing0 = brt["timing0"]
        chn_cfg.config.can.timing1 = brt["timing1"]
        chn_cfg.config.can.acc_code = 0
        chn_cfg.config.can.acc_mask = 0xFFFFFFFF

        for channel in self.channels:
            # 初始化每个CAN通道，并储存CAN通道句柄
            self._chn_handle[channel] = self.device.InitCAN(
                self._dev_handle, dev_index, chn_cfg)
            # 初始通道结束后，启动CAN通道
            res = self.device.StartCAN(self._chn_handle[channel])
            if res != driver.ZCAN_STATUS_OK:
                raise ValueError('Start CAN failure! Please check whether device is occupied.')

    # Delay to use between each poll for new messages
    #
    # The timeout is deliberately kept low to avoid the possibility of
    # a hardware buffer overflow. This value was determined
    # experimentally, but the ideal value will depend on the specific
    # system.
    RX_POLL_DELAY = 0.020

    def _recv_from_queue(self) -> Tuple[Message, bool]:
        """Return a message from the internal receive queue"""
        channel, raw_msg = self.rx_queue.popleft()  # type: can.Message

        # 将时间戳转为秒为单位
        timestamp = raw_msg.timestamp * 1e-6

        return (
            Message(
                channel=channel,
                timestamp=timestamp,
                arbitration_id=raw_msg.arbitration_id,
                is_extended_id=raw_msg.is_extended_id,
                is_remote_frame=raw_msg.is_remote_frame,
                dlc=raw_msg.dlc,
                data=bytes(raw_msg.data),
            ),
            False,
        )

    def poll_received_messages(self) -> None:
        """Poll new messages from the device into the rx queue but don't
        return any message to the caller

        Calling this function isn't necessary as polling the device is done
        automatically when calling recv(). This function is for the situation
        where an application needs to empty the hardware receive buffer without
        consuming any message.
        """
        for channel in self.channels:
            rcv_num = self.device.GetReceiveNum(self._chn_handle[channel], driver.ZCAN_TYPE_CAN)

            zcan_raw_msgs, act_rcv_num = self.device.Receive(
                self._chn_handle[channel], rcv_num)  # type: list

            # 将周立功ZCAN_Receive_Data转换为can.message格式
            raw_msgs = [Message(
                timestamp=zcan_raw_msg.timestamp,
                arbitration_id=zcan_raw_msg.frame.can_id,
                is_extended_id=zcan_raw_msg.frame.eff,
                is_remote_frame=zcan_raw_msg.frame.rtr,
                is_error_frame=zcan_raw_msg.frame.err,
                dlc=zcan_raw_msg.frame.can_dlc,
                data=bytes(zcan_raw_msg.frame.data),
            ) for zcan_raw_msg in list(zcan_raw_msgs)]

            self.rx_queue.extend(
                (channel, raw_msg) for raw_msg in raw_msgs
            )
        pass

    def _recv_internal(self, timeout: Optional[float]) -> Tuple[Optional[Message], bool]:
        """

        :param timeout: float in seconds
        :return:
        """

        if self.rx_queue:
            return self._recv_from_queue()

        deadline = None
        while deadline is None or time.time() < deadline:
            if deadline is None and timeout is not None:
                deadline = time.time() + timeout

            self.poll_received_messages()

            if self.rx_queue:
                return self._recv_from_queue()

            # If blocking on a timeout, add a sleep before we loop again
            # to reduce CPU usage.
            if deadline is None or deadline - time.time() > 0.050:
                time.sleep(self.RX_POLL_DELAY)

        return (None, False)

    def send(self, msg: Message, transmit_type: int = 2, timeout: Optional[float] = None) -> None:
        """Send a CAN message to the bus

        :param transmit_type: 发送类型，默认为2自发自收。
        :param msg: message to send
        :param timeout: timeout (in seconds) to wait for the TX queue to clear.
        If set to ``None`` (default) the function returns immediately.

        Note: Due to limitations in the device firmware and protocol, the
        timeout will not trigger if there are problems with CAN arbitration,
        but only if the device is overloaded with a backlog of too many
        messages to send.
        """
        # 将原can.Message打包为ZCAN_Transmit_Data类型
        raw_message = driver.ZCAN_Transmit_Data(
            driver.ZCAN_CAN_FRAME(
                msg.arbitration_id,
                msg.is_error_frame,  # err
                msg.is_remote_frame,  # rtr
                msg.is_extended_id,  # eff
                msg.dlc,  # can_dlc
                0,  # __pad
                0,  # __res0
                0,  # __res1
                (c_ubyte * 8)(*msg.data),
            ),

            transmit_type,  # transmit_type，2为自发自收
        )

        if msg.channel is not None:
            channel = msg.channel
        elif len(self.channels) == 1:
            channel = self.channels[0]
        else:
            raise ValueError(
                "Message channel must be set when using multiple channels."
            )

        send_result = self.device.Transmit(self._chn_handle[channel], raw_message, len=1)
        if timeout is not None and not send_result:
            raise CanTimeoutError(f"Send timed out after {timeout} seconds")

    @staticmethod
    def _detect_available_configs() -> List[can.typechecking.AutoDetectedConfig]:
        pass

    def shutdown(self) -> None:
        super().shutdown()
        res = self.device.CloseDevice(self._dev_handle)
        if res != driver.ZCAN_STATUS_OK:
            raise ValueError('Could not shut down bus!')

    def fileno(self) -> int:
        pass
