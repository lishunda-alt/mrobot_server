import json
import struct

# 0x5A + Version + serierNum + jsonLen + reqNum + rsv
PACK_HEAD_FMT_STR = '!BBHLH6s'
PACK_RSV_DATA = b'\x00\x00\x00\x00\x00\x00'


class MsgProcess(object):
    """
    packMsg(reqId, msgTyp, msg: dict) ->bytes:将信息打包为机器人接收的数据形式
    unpackHead(data) -> tuple:将机器人发的数据进行拆包
    """
    # staticmethod用于修饰类中的方法,使其可以在不创建类实例的情况下调用方法
    @staticmethod
    def packMsg(req_id, msg_type, msg=None) -> bytes:
        """
        返回一个 bytes 对象，其中包含根据格式字符串 "!BBHLH6s" 打包的各项的值
        参数个数必须与格式字符串所要求的值完全匹配。
        req_id: 请求以及相应的序号
        msg_type: API编号 
                启动站点任务3051 |
                取消站点任务3003 |
                暂停站点任务3001 |
                恢复站点任务3002 |
                启动任务组3100 |
                取消任务组3140 |
                暂停任务组3102 |
                恢复任务组3103 |
                查询AGV位置1004 |
                查询AGV任务状态:1020 |
        """
        msgLen = 0
        if msg:
            jsonStr = json.dumps(msg)
            msgLen = len(jsonStr)
            rawMsg = struct.pack(PACK_HEAD_FMT_STR, 0x5A, 1, req_id,
                                 msgLen, msg_type, PACK_RSV_DATA)
            rawMsg += bytearray(json.dumps(msg), 'ascii')
        else:
            msgLen = 0
            rawMsg = struct.pack(PACK_HEAD_FMT_STR, 0x5A, 1, req_id,
                                 msgLen, msg_type, PACK_RSV_DATA)
        return rawMsg

    @staticmethod
    def unpackHead(data) -> tuple:
        """
        根据格式字符串 format 从缓冲区 buffer 解包, 结果为一个元组
        返回一个元组(jsonLen, reqNum)
        """
        result = struct.unpack(PACK_HEAD_FMT_STR, data)
        jsonLen, reqNum = result[3], result[4]
        # reqNum = result[4]

        return jsonLen, reqNum
