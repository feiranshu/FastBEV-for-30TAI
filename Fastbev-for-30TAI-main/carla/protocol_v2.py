#!/usr/bin/env python3
from __future__ import annotations
import json, socket, struct, zlib
from typing import Any

MAGIC=b"ADV2"; VERSION=2
MSG_PERCEPTION_BATCH=10; MSG_PERCEPTION_RESULT=11
MSG_CONTROL_INPUT=20; MSG_CONTROL_RESULT=21
MAX_PACKET_BYTES=64*1024*1024
HEADER=struct.Struct("!4sHHQQII")
ENTRY=struct.Struct("!16sII")
META_LEN=struct.Struct("!I")
LENGTH=struct.Struct("!I")

class ProtocolError(RuntimeError): pass

def _json_bytes(doc:dict[str,Any])->bytes:
    return json.dumps(doc,ensure_ascii=False,separators=(",",":"),allow_nan=False).encode("utf-8")

def pack_message(message_type,frame_id,timestamp_ns,item_count,payload):
    if len(payload)>MAX_PACKET_BYTES-HEADER.size: raise ProtocolError("payload too large")
    return HEADER.pack(MAGIC,VERSION,int(message_type),int(frame_id),int(timestamp_ns),int(item_count),len(payload))+payload

def unpack_header(packet):
    if len(packet)<HEADER.size: raise ProtocolError("packet shorter than ADV2 header")
    magic,version,msg_type,frame_id,timestamp_ns,item_count,payload_size=HEADER.unpack_from(packet)
    if magic!=MAGIC: raise ProtocolError(f"bad ADV2 magic: {magic!r}")
    if version!=VERSION: raise ProtocolError(f"unsupported ADV2 version: {version}")
    if payload_size!=len(packet)-HEADER.size: raise ProtocolError("ADV2 payload size mismatch")
    return {"message_type":msg_type,"frame_id":frame_id,"timestamp_ns":timestamp_ns,"item_count":item_count,"payload_size":payload_size}

def pack_json_message(message_type,frame_id,timestamp_ns,document,item_count=0):
    return pack_message(message_type,frame_id,timestamp_ns,item_count,_json_bytes(document))

def unpack_json_message(packet,expected_type=None):
    h=unpack_header(packet)
    if expected_type is not None and h["message_type"]!=expected_type:
        raise ProtocolError(f"expected message type {expected_type}, got {h['message_type']}")
    doc=json.loads(packet[HEADER.size:].decode("utf-8"))
    if not isinstance(doc,dict): raise ProtocolError("JSON payload must be an object")
    if "frame_id" in doc and int(doc["frame_id"])!=int(h["frame_id"]): raise ProtocolError("frame_id mismatch")
    doc["_wire"]=h
    return doc

def pack_perception_batch(frame_id,capture_ts_ns,camera_order,jpeg_by_camera,metadata):
    meta=dict(metadata); meta.setdefault("schema","perception_input_v2"); meta.setdefault("frame_id",int(frame_id)); meta.setdefault("capture_ts_ns",int(capture_ts_ns))
    mb=_json_bytes(meta); parts=[META_LEN.pack(len(mb)),mb]
    for name in camera_order:
        jpeg=jpeg_by_camera[name]; enc=name.encode("ascii")
        if len(enc)>16: raise ProtocolError("camera name too long")
        if len(jpeg)<4 or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"): raise ProtocolError(f"bad JPEG: {name}")
        parts += [ENTRY.pack(enc.ljust(16,b"\0"),len(jpeg),zlib.crc32(jpeg)&0xffffffff),jpeg]
    return pack_message(MSG_PERCEPTION_BATCH,frame_id,capture_ts_ns,len(camera_order),b"".join(parts))

def unpack_perception_batch(packet,verify_crc=True):
    h=unpack_header(packet)
    if h["message_type"]!=MSG_PERCEPTION_BATCH: raise ProtocolError("expected PERCEPTION_BATCH")
    payload=memoryview(packet)[HEADER.size:]
    if len(payload)<4: raise ProtocolError("missing metadata length")
    meta_size=META_LEN.unpack_from(payload,0)[0]; off=4
    if off+meta_size>len(payload): raise ProtocolError("truncated metadata")
    metadata=json.loads(bytes(payload[off:off+meta_size]).decode("utf-8")); off+=meta_size
    cameras=[]
    for _ in range(h["item_count"]):
        if off+ENTRY.size>len(payload): raise ProtocolError("truncated camera entry")
        nr,size,crc=ENTRY.unpack_from(payload,off); off+=ENTRY.size
        if off+size>len(payload): raise ProtocolError("truncated JPEG")
        jpeg=bytes(payload[off:off+size]); off+=size
        if verify_crc and (zlib.crc32(jpeg)&0xffffffff)!=crc: raise ProtocolError("JPEG CRC mismatch")
        cameras.append({"name":nr.rstrip(b"\0").decode("ascii"),"jpeg":jpeg,"size":size,"crc32":crc})
    if off!=len(payload): raise ProtocolError("trailing bytes")
    return {**h,"metadata":metadata,"cameras":cameras}

def pack_perception_result(document):
    return pack_json_message(MSG_PERCEPTION_RESULT,int(document["frame_id"]),int(document.get("finished_ts_ns",0)),document,len(document.get("objects",[])))
def unpack_perception_result(packet):
    doc=unpack_json_message(packet,MSG_PERCEPTION_RESULT)
    if len(doc.get("objects",[]))!=doc["_wire"]["item_count"]: raise ProtocolError("object count mismatch")
    return doc
def pack_control_input(frame_id,sample_ts_ns,metadata):
    doc=dict(metadata); doc.setdefault("schema","control_input_v2"); doc.setdefault("frame_id",int(frame_id)); doc.setdefault("sample_ts_ns",int(sample_ts_ns))
    return pack_json_message(MSG_CONTROL_INPUT,frame_id,sample_ts_ns,doc,0)
def unpack_control_input(packet): return unpack_json_message(packet,MSG_CONTROL_INPUT)
def pack_control_result(document):
    return pack_json_message(MSG_CONTROL_RESULT,int(document["frame_id"]),int(document.get("finished_ts_ns",0)),document,0)
def unpack_control_result(packet): return unpack_json_message(packet,MSG_CONTROL_RESULT)

def recv_exact(sock,size):
    chunks=[]; left=size
    while left:
        chunk=sock.recv(left)
        if not chunk: raise ConnectionError("socket closed")
        chunks.append(chunk); left-=len(chunk)
    return b"".join(chunks)
def recv_packet(sock):
    size=LENGTH.unpack(recv_exact(sock,4))[0]
    if size<HEADER.size or size>MAX_PACKET_BYTES: raise ProtocolError(f"invalid packet size {size}")
    return recv_exact(sock,size)
def send_packet(sock,packet):
    if len(packet)>MAX_PACKET_BYTES: raise ProtocolError("packet too large")
    sock.sendall(LENGTH.pack(len(packet))+packet)
