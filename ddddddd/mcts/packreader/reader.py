import logging
import struct
import os, sys
sys.path.append(os.path.dirname(__file__))

# from frame_pb2 import Frame as Frame_v1, FrameShuffle
from frame.v2_pb2 import Frame as Frame_v2


END = -1
CONTINUE = 1
SUCCESS = 2


class Chunk(object):
    def __init__(self, reader, offset, length, lazy):
        self._reader = reader
        self.offset = offset
        self.length = length
        self.content = None
        if not lazy:
            self.read()

    def read(self):
        if self.content is None:
            self.content = self._reader._read(self.offset, self.length)
        return self.content


class Data(Chunk):
    def __init__(self, reader, offset, length, data_descriptor, topic, lazy):
        super(Data, self).__init__(reader, offset, length, lazy)
        self.data_descriptor = data_descriptor
        self.topic = topic


class Frame(Chunk):
    def __init__(self, reader, meta, offset, length, data_list, topic_meta):
        super(Frame, self).__init__(reader, offset, length, lazy=True)
        self.meta = meta
        self.data_list = data_list
        self.topic_meta = topic_meta


class DataDescriptor(object):
    def __init__(self, type):
        self.type = type


class Reader(object):
    def __init__(self, fid, topics=[], lazy=True):
        self._fid = fid
        self.version = struct.unpack('i', self._fid.read(4))[0]
        self._lazy = lazy
        self.topics = topics
        self.topics_obj = {}

    def __del__(self):
        # self.close()
        pass

    def close(self):
        # self._fid.close()
        pass

    def _read(self, offset, length):
        old_pos = self._fid.tell()
        self._fid.seek(offset)
        buf = self._fid.read(length)
        self._fid.seek(old_pos)
        return buf

    def _parse_frame_v0(self, frame_meta):
        data_list = []
        ss = self._fid.read(4)
        if ss == '':
            status = END
        else:
            length = struct.unpack('i', ss)[0]
            offset = self._fid.tell()
            if length == 0:
                status = CONTINUE
            else:
                if frame_meta.frame_v0.img_frame.count:
                    image_count = frame_meta.frame_v0.img_frame.count
                else:
                    image_count = 1
                left_length = length
                for i in range(image_count):
                    ss = self._fid.read(4)
                    if ss == '':
                        status = END
                        break
                    d_offset = self._fid.tell()
                    d_length = struct.unpack('i', ss)[0]
                    data = Data(self, d_offset, d_length, DataDescriptor("image"), "", self._lazy)
                    data_list.append(data)
                    self._fid.seek(d_offset + d_length)
                    left_length -= (d_length + 4)
                if left_length > 0:
                    for i in range(image_count):
                        ss = self._fid.read(4)
                        if ss == '':
                            status = END
                            break
                        d_offset = self._fid.tell()
                        d_length = struct.unpack('i', ss)[0]
                        data = Data(self, d_offset, d_length, DataDescriptor("parsing"), "", self._lazy)
                        data_list.append(data)
                        self._fid.seek(d_offset + d_length)
                self._fid.seek(offset + length)
            status = SUCCESS
        return status, data_list

    def _parse_frame_v1(self, frame_meta):
        data_list = []
        ss = self._fid.read(4)
        if ss == '':
            status = END
        else:
            length = struct.unpack('i', ss)[0]
            offset = self._fid.tell()
            if length == 0:
                status = SUCCESS
            else:
                for data_descriptor in frame_meta.frame_v1.data_descriptor:
                    if hasattr(data_descriptor, "data") and data_descriptor.data.with_data_field == False:
                        continue
                    ss = self._fid.read(4)
                    if ss == '':
                        status = END
                        break
                    d_offset = self._fid.tell()
                    d_length = struct.unpack('i', ss)[0]
                    data = Data(self, d_offset, d_length, data_descriptor, "", self._lazy)
                    data_list.append(data)
                    self._fid.seek(d_offset + d_length)
                self._fid.seek(offset + length)
                status = SUCCESS
        return status, data_list

    def _parse_frame_v2(self, frame_meta):
        data_list = []
        topic_meta = {}
        ss = self._fid.read(4)
        if ss == '':
            status = END
        else:
            length = struct.unpack('i', ss)[0]
            offset = self._fid.tell()
            if length == 0:
                status = SUCCESS
                for data_descriptor in frame_meta.data_list.data_descriptor:
                    topic_name = data_descriptor.message_name
                    if not hasattr(data_descriptor, "proto") or len(data_descriptor.proto) == 0:
                        continue
                    if topic_name in self.topics:
                        topic_meta[topic_name] = []
                    for serialized_proto in data_descriptor.proto:
                        if hasattr(serialized_proto, "meta") and topic_name in topic_meta and topic_name in self.topics_obj:
                            cur_obj = self.topics_obj[topic_name]()
                            cur_obj.ParseFromString(serialized_proto.meta)
                            topic_meta[topic_name].append(cur_obj)
            else:
                for data_descriptor in frame_meta.data_list.data_descriptor:
                    topic_name = data_descriptor.message_name
                    if not hasattr(data_descriptor, "proto") or len(data_descriptor.proto) == 0:
                        continue
                    if topic_name in self.topics:
                        topic_meta[topic_name] = []
                    for serialized_proto in data_descriptor.proto:
                        if hasattr(serialized_proto, "meta") and topic_name in topic_meta and topic_name in self.topics_obj:
                            cur_obj = self.topics_obj[topic_name]()
                            cur_obj.ParseFromString(serialized_proto.meta)
                            topic_meta[topic_name].append(cur_obj)
                        if hasattr(serialized_proto, "data_index") and serialized_proto.data_index > -1:
                            ss = self._fid.read(4)
                            if ss == '':
                                status = END
                                break
                            d_offset = self._fid.tell()
                            d_length = struct.unpack('i', ss)[0]
                            data = Data(self, d_offset, d_length, serialized_proto, data_descriptor.message_name, self._lazy)
                            data_list.append(data)
                            self._fid.seek(d_offset + d_length)
                self._fid.seek(offset + length)
                status = SUCCESS
        return status, data_list, topic_meta

    def __next__(self):
        while True:
            # get_frame
            try:
                ss = self._fid.read(4)
                if ss == b'':
                    raise StopIteration
                offset = self._fid.tell()
                length = struct.unpack('i', ss)[0]
                if length == 0:
                    raise StopIteration

                meta = self._fid.read(length)
            except Exception as e:
                raise StopIteration

            frame_meta = Frame_v2()
            try:
                frame_meta.ParseFromString(meta)
            except Exception as e:
                logging.warning('Parse frame failed')
                raise StopIteration
            version = frame_meta.version
            proto_version = frame_meta.proto_version
            topic_meta = {}
            if proto_version >= 2:
                try:
                    status, data_list, topic_meta = self._parse_frame_v2(frame_meta)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    logging.warning('Parse frame v2 failed')
                    raise StopIteration
            else:
                pass
                # if version < 0:
                #     frame_meta = FrameShuffle()
                # else:
                #     frame_meta = Frame_v1()
                # try:
                #     frame_meta.ParseFromString(meta)
                # except Exception as e:
                #     logging.warning('Parse frame failed')
                #     raise StopIteration
                # if proto_version == 0:
                #     try:
                #         status, data_list = self._parse_frame_v0(frame_meta)
                #     except Exception as e:
                #         logging.warning('Parse frame v0 failed')
                #         raise StopIteration
                # if proto_version == 1:
                #     try:
                #         status, data_list = self._parse_frame_v1(frame_meta)
                #     except Exception as e:
                #         logging.warning('Parse frame v1 failed')
                #         raise StopIteration

            if status == END:
                raise StopIteration
            elif status == CONTINUE:
                continue
            elif status == SUCCESS:
                frame = Frame(self, frame_meta, offset, length, data_list, topic_meta)
                return frame
            else:
                logging.exception('Not implemented')
                raise NotImplementedError

    def reset(self):
        self._fid.seek(0)
        _ = struct.unpack('i', self._fid.read(4))[0]

    def __iter__(self):
        self.reset()
        return self
