#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' fms '
__author__ = '747'

from enum import Enum
import argparse
import time
import ntpath


def crc32(data):
  checksum = 0
  l = len(data)

  for j in range(0, (int)(l / 2) * 2, 2):
    checksum += ((data[j]<<8) | data[j+1])

  if (l % 2):
    checksum += data[l - 1]

  checksum = checksum & 0xFFFFFFFF
  checksum = ~checksum
  checksum = checksum & 0xFFFFFFFF
  return checksum

def crc16(data):
  checksum = 0
  l = len(data)

  for j in range(0, (int)(l / 2) * 2, 2):
    checksum += ((data[j]<<8) | data[j+1])

  if (l % 2):
    checksum += data[l - 1]

  while checksum > 0xFFFF:
    checksum = ((checksum >> 16) & 0xFFFF) + (checksum & 0xFFFF)

  checksum = checksum & 0xFFFF;
  checksum = ~checksum
  checksum = checksum & 0xFFFF;

  return checksum

class packet_type(Enum):
  SM2_CTRL_FW = 0x0001
  A400_CTRL_FW = 0x0002
  J1_CTRL_FW = 0x0003
  SM2_MODULE_FW = 0x0004
  ESP32_MODULE_FW = 0x0005

class ugr_ctrl_flag(Enum):
  UGR_NORMAL = 0x00
  UGR_FORCE = 0x01

class ugr_status(Enum):
  UGR_STATUS_FIRST_BURN = 0xAA00
  UGR_STATUS_FIRST_WAIT = 0xAA01
  UGR_STATUS_FIRST_START = 0xAA02
  UGR_STATUS_FIRST_TRANS = 0xAA03
  UGR_STATUS_FIRST_END = 0xAA04
  UGR_STATUS_FIRST_JUMP_APP = 0xAA05

class packet:
  def __init__(self, bin, p_type, ctrl_flag, ver, radr, s_id, e_id):
    self.magic_string = "snapmaker update.bin"
    self.protocol_ver = 0x01
    self.pack_type = p_type
    self.ugr_ctrl_flag = ctrl_flag
    self.start_index = s_id
    self.end_index = e_id
    self.fw_version = ver
    self.timestamp = "2022.04.28:18:03:01"
    self.ugr_status = 0xAA00
    self.fw_lenght = len(bin)
    self.fw_checksum = crc32(bin)
    print("%x" % self.fw_checksum)
    self.fw_runaddr = radr
    self.channel = 0
    self.peer = 0
    self.packet_checksum = 0
    self.bin = bin

  def gen(self):
    payload = bytearray(0)
    payload.extend(bytes(self.magic_string, encoding="utf-8"))
    payload.append(0x00)
    payload.append(self.protocol_ver & 0xFF)
    payload.extend(self.pack_type.to_bytes(2, 'little'))
    payload.append(self.ugr_ctrl_flag & 0xFF)
    payload.extend(self.start_index.to_bytes(2, 'little'))
    payload.extend(self.end_index.to_bytes(2, 'little'))

    b = bytearray(self.fw_version, encoding="utf-8")
    print(b)
    l = len(b)
    for i in range(l, 32):
      b.append(0)
    payload.extend(b)
    print(b)

    b = bytearray(self.timestamp, encoding="utf-8")
    print(b)
    l = len(b)
    for i in range(l, 20):
      b.append(0)
    payload.extend(b)
    print(b)

    payload.extend(self.ugr_status.to_bytes(2, 'little'))
    payload.extend(self.fw_lenght.to_bytes(4, 'little'))
    if self.fw_checksum < 0:
      payload.extend(self.fw_checksum.to_bytes(4, 'little', signed=True))
    else:
      payload.extend(self.fw_checksum.to_bytes(4, 'little'))
    payload.extend(self.fw_runaddr.to_bytes(4, 'little'))
    payload.append(self.channel & 0xFF)
    payload.append(self.peer & 0xFF)
    self.packet_checksum = crc32(payload)
    payload.extend(self.packet_checksum.to_bytes(4, 'little'))

    l = len(payload)
    b = bytearray(0)
    for i in range(l, 256):
      b.append(0)
    payload.extend(b)

    return payload


VER = "V1.1.0"
FLAG = 1
# default entry addr for J1
RUNADDR = 0x08002800
# packet type, 1: SM2 CONTROLER, 2: A400 CONTROLER, 3: J1 CONTROLER, 4: SM2 MODUEL, 5: ESP32 MODUEL
TYPE = 3
START_ID = 0
END_ID = 23
O_NAME=None
parser = argparse.ArgumentParser(description="gen_header")
parser.add_argument('--file', '-f', help='bin file name')
parser.add_argument('--type', '-t', help='packet type, 1: SM2 CONTROLER, 2: A400 CONTROLER, 3: J1 CONTROLER, 4: SM2 MODUEL, 5: ESP32 MODUEL')
parser.add_argument('--ver',  '-v', help='version')
parser.add_argument('--flag', '-c', help='upgrade control flag, 0 for normal, 1 for force')
parser.add_argument('--radr', '-a', help='firmware run address')
parser.add_argument('--output', '-o', help='output file name')
args = parser.parse_args()

try:
  FILE = args.file
  TYPE = int(args.type)
  O_NAME = args.output
except:
  print("unsupported param")
  while True:
    time.sleep(1)

try:
  VER = args.ver
  FLAG = int(args.flag)
  RUNADDR = int(args.radr, 16)
except:
  pass

print("======== INFORMATION ======")
print("flie: " + FILE)
print("type: %d" % TYPE)
print("ver: " + VER)
print("flag %d: " % FLAG)
print("entry addr 0x%x: " % RUNADDR)
print("output file: {}".format(O_NAME))


# _(self, bin, p_type, ctrl_flag, ver, radr):
f = open(FILE, 'rb')
bin = f.read()
print("file lenght %d" % len(bin))

pt = packet(bin, TYPE, FLAG, VER, RUNADDR, START_ID, END_ID)
head = pt.gen()

hex_str = " ".join(["{:02x}".format(x) for x in head])
print(hex_str)

if isinstance(O_NAME, str):
  of = O_NAME
else:
  of = ntpath.basename(FILE) + ".pack"

f = open(of, 'wb')
f.write(head)
f.write(bin)

if __name__=='__main__':
    pass