import socket
import sys
import threading
import time
import cv2
import io
import struct
import numpy as np
import logging
from datetime import datetime
from collections import deque
import statistics
import os

# Set up logging with UTF-8 encoding to handle any special characters
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('rtsp_client.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# JPEG Huffman and quantization tables (from original rtsp_client.py)
dc_luminance_table = bytearray([
    0x00,
    0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
])

ac_luminance_table = bytearray([
    0x00,
    0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
    0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0,
    0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34,
    0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
    0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
    0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
    0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
    0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9,
    0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA
])

dc_chrominance_table = bytearray([
    0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
])

ac_chrominance_table = bytearray([
    0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0,
    0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
    0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
    0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA
])

huffman_table = [
    # Huffman table DC (luminance)
    0xff, 0xc4,
    0x00, 0x1f, 0x00,
    0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
    # Huffman table AC (luminance)
    0xff, 0xc4,
    0x00, 0xb5, 0x10,
    0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
    # Huffman table DC (chrominance)
    0xff, 0xc4,
    0x00, 0x1f, 0x01,
    0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
    # Huffman table AC (chrominance)
    0xff, 0xc4,
    0x00, 0xb5, 0x11,
    0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
]

# JPEG standard base quantization tables (from Annex K)
BASE_QT_LUMA = [
    16,11,10,16,24,40,51,61,
    12,12,14,19,26,58,60,55,
    14,13,16,24,40,57,69,56,
    14,17,22,29,51,87,80,62,
    18,22,37,56,68,109,103,77,
    24,35,55,64,81,104,113,92,
    49,64,78,87,103,121,120,101,
    72,92,95,98,112,100,103,99,
]

BASE_QT_CHROMA = [
    17,18,24,47,99,99,99,99,
    18,21,26,66,99,99,99,99,
    24,26,56,99,99,99,99,99,
    47,66,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
]

def qtables_from_quality(q: int):
    """RFC 2435 / JPEG scaling. q in [1..99]; clamp as needed."""
    q = max(1, min(99, int(q)))
    S = 5000 // q if q < 50 else 200 - 2 * q
    def scale(tbl):
        out = []
        for t in tbl:
            v = (t * S + 50) // 100
            out.append(1 if v < 1 else (255 if v > 255 else v))
        return bytes(out)
    return scale(BASE_QT_LUMA), scale(BASE_QT_CHROMA)

class RtpPacket:
    def __init__(self, data):
        self.data = data
        vpxcc, m_pt, self.sequence_number, self.timestamp, self.ssrc = struct.unpack('!BBHII', data[:12])
        self.version = (vpxcc >> 6) & 0x03
        self.padding = (vpxcc >> 5) & 0x01
        self.extension = (vpxcc >> 4) & 0x01
        self.csrc_count = vpxcc & 0x0F
        self.marker = (m_pt >> 7) & 0x01
        self.payload_type = m_pt & 0x7F

        off = 12 + 4 * self.csrc_count
        if self.extension:
            ext_profile, ext_len = struct.unpack('!HH', data[off:off+4])
            off += 4 + 4 * ext_len

        self.payload = data[off:]

    def __repr__(self):
        return f"RtpPacket(pt={self.payload_type}, M={self.marker}, seq={self.sequence_number}, ts={self.timestamp})"

    def get_payload(self):
        return self.payload

    def get_payload_type(self):
        return self.payload_type

    def get_marker(self):
        return self.marker

class RtpJpegPacket(RtpPacket):
    def __init__(self, data):
        super().__init__(data)
        rp = self.get_payload()

        self.type_specific, off3, self.frag_type, self.q, w8, h8 = struct.unpack('!B3sBBBB', rp[:8])
        self.frag_offset = int.from_bytes(off3, 'big')
        self.width  = int(w8) * 8
        self.height = int(h8) * 8

        p = 8
        self.q0 = None
        self.q1 = None

        if self.q >= 128:
            mbz, precision, qlen = struct.unpack('!BBH', rp[p:p+4]); p += 4
            qt = rp[p:p+qlen]; p += qlen
            if qlen >= 128:
                self.q0 = qt[:64]
                self.q1 = qt[64:128]
            else:
                self.q0, self.q1 = qtables_from_quality(75)
        else:
            self.q0, self.q1 = qtables_from_quality(self.q if self.q > 0 else 75)

        self.jpeg_data = rp[p:]

    def __repr__(self):
        return (f"RtpJpegPacket(pt={self.payload_type}, M={self.marker}, seq={self.sequence_number}, "
                f"ts={self.timestamp}, off={self.frag_offset}, q={self.q}, {self.width}x{self.height})")

    def get_width(self): return self.width
    def get_height(self): return self.height
    def get_frag_offset(self): return self.frag_offset
    def get_jpeg_data(self): return self.jpeg_data
    def get_q0(self): return self.q0
    def get_q1(self): return self.q1

class JpegHeader:
    def __init__(self, width, height, q0_quantization_table, q1_quantization_table):
        self.width = width
        self.height = height

        self.data = io.BytesIO()
        self.data.write(b'\xFF\xD8')  # Start Of Image (SOI) marker
        
        jfif_app0_marker = bytearray([
            0xFF, 0xE0,  # APP0 marker
            0x00, 0x10,  # Length (16 bytes)
            0x4A, 0x46, 0x49, 0x46, 0x00,  # JFIF identifier
            0x01, 0x01,  # JFIF version 1.1
            0x01,        # Units: DPI
            0x00, 0x00,  # X density (2 bytes)
            0x00, 0x00,  # Y density (2 bytes)
            0x00, 0x00   # No thumbnail (width 0, height 0)
        ])
        self.data.write(jfif_app0_marker)

        # Quantization table (DQT) marker for luminance
        self.data.write(b'\xFF\xDB\x00\x43\x00')
        self.data.write(bytearray(q0_quantization_table))

        # Quantization table (DQT) marker for chrominance
        self.data.write(b'\xFF\xDB\x00\x43\x01')
        self.data.write(bytearray(q1_quantization_table))

        self.data.write(bytes(huffman_table))

        # Frame header (SOF0) marker
        sof0_marker = bytearray([
            0xFF, 0xC0,  # SOF0 marker
            0x00, 0x11,  # Length (17 bytes)
            0x08,        # Data precision: 8 bits
            *self.height.to_bytes(2, 'big'),
            *self.width.to_bytes(2, 'big'),
            0x03,        # Number of components: 3 (YCbCr)
            0x01, 0x21, 0x00,  # Component 1 (Y)
            0x02, 0x11, 0x01,  # Component 2 (Cb)
            0x03, 0x11, 0x01   # Component 3 (Cr)
        ])
        self.data.write(sof0_marker)

        # Scan header (SOS) marker
        self.data.write(b'\xFF\xDA\x00\x0C\x03\x01\x00\x02\x11\x03\x11\x00\x3F\x00')

    def __repr__(self):
        return f"JpegHeader(width={self.width}, height={self.height})"

    def get_data(self):
        return self.data.getvalue()

class JpegFrame:
    def __init__(self, first_pkt: RtpJpegPacket):
        self.jpeg_header = JpegHeader(first_pkt.get_width(), first_pkt.get_height(),
                                      first_pkt.get_q0(), first_pkt.get_q1())
        self.buf = bytearray()
        self.last_off = 0
        self.ok = True
        self.add_packet(first_pkt)

    def add_packet(self, pkt: RtpJpegPacket):
        off = pkt.get_frag_offset()
        data = pkt.get_jpeg_data()
        if off != self.last_off:
            self.ok = False
        self.last_off = off + len(data)
        self.buf.extend(data)

    def get_data(self):
        out = io.BytesIO()
        out.write(self.jpeg_header.get_data())
        out.write(self.buf)
        out.write(b'\xFF\xD9')  # EOI
        return out.getvalue()

class FrameStats:
    def __init__(self, window_size=100):
        self.window_size = window_size
        self.frame_times = deque(maxlen=window_size)
        self.frame_sizes = deque(maxlen=window_size)
        self.decode_times = deque(maxlen=window_size)
        self.packet_counts = deque(maxlen=window_size)
        self.start_time = time.time()
        self.total_frames = 0
        self.total_bytes = 0
        self.corrupted_frames = 0
        self.last_log_time = time.time()

    def add_frame(self, frame_size, decode_time, packet_count, corrupted=False):
        current_time = time.time()
        self.frame_times.append(current_time)
        self.frame_sizes.append(frame_size)
        self.decode_times.append(decode_time)
        self.packet_counts.append(packet_count)
        
        self.total_frames += 1
        self.total_bytes += frame_size
        if corrupted:
            self.corrupted_frames += 1

        # Log detailed stats every 5 seconds
        if current_time - self.last_log_time >= 5.0:
            self.log_stats()
            self.last_log_time = current_time

    def get_fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        time_span = self.frame_times[-1] - self.frame_times[0]
        return (len(self.frame_times) - 1) / time_span if time_span > 0 else 0.0

    def get_avg_frame_size(self):
        return statistics.mean(self.frame_sizes) if self.frame_sizes else 0

    def get_avg_decode_time(self):
        return statistics.mean(self.decode_times) if self.decode_times else 0

    def get_avg_packets_per_frame(self):
        return statistics.mean(self.packet_counts) if self.packet_counts else 0

    def get_total_bitrate(self):
        elapsed = time.time() - self.start_time
        return (self.total_bytes * 8) / (elapsed * 1000) if elapsed > 0 else 0  # kbps

    def log_stats(self):
        logger.info("=== FRAME STATISTICS ===")
        logger.info(f"Current FPS: {self.get_fps():.2f}")
        logger.info(f"Average Frame Size: {self.get_avg_frame_size():.0f} bytes")
        logger.info(f"Average Decode Time: {self.get_avg_decode_time()*1000:.2f} ms")
        logger.info(f"Average Packets/Frame: {self.get_avg_packets_per_frame():.1f}")
        logger.info(f"Total Bitrate: {self.get_total_bitrate():.1f} kbps")
        logger.info(f"Total Frames: {self.total_frames}")
        logger.info(f"Corrupted Frames: {self.corrupted_frames} ({100*self.corrupted_frames/max(1,self.total_frames):.1f}%)")
        logger.info(f"Total Data: {self.total_bytes/1024:.1f} KB")
        
        if self.frame_sizes:
            logger.info(f"Frame Size Range: {min(self.frame_sizes)} - {max(self.frame_sizes)} bytes")
        if self.decode_times:
            logger.info(f"Decode Time Range: {min(self.decode_times)*1000:.1f} - {max(self.decode_times)*1000:.1f} ms")
        logger.info("========================")

class PacketStats:
    def __init__(self):
        self.packets_received = 0
        self.bytes_received = 0
        self.sequence_errors = 0
        self.last_sequence = None
        self.duplicate_packets = 0
        self.out_of_order_packets = 0
        self.start_time = time.time()

    def add_packet(self, packet):
        self.packets_received += 1
        self.bytes_received += len(packet.data)
        
        # Check sequence number
        if self.last_sequence is not None:
            expected_seq = (self.last_sequence + 1) % 65536
            if packet.sequence_number != expected_seq:
                if packet.sequence_number < self.last_sequence:
                    self.out_of_order_packets += 1
                    logger.debug(f"Out-of-order packet: got {packet.sequence_number}, expected {expected_seq}")
                elif packet.sequence_number == self.last_sequence:
                    self.duplicate_packets += 1
                    logger.debug(f"Duplicate packet: {packet.sequence_number}")
                else:
                    self.sequence_errors += 1
                    missed = packet.sequence_number - expected_seq
                    logger.warning(f"Sequence gap: missed {missed} packets (got {packet.sequence_number}, expected {expected_seq})")
        
        self.last_sequence = packet.sequence_number

    def get_packet_loss_rate(self):
        if self.packets_received == 0:
            return 0.0
        return (self.sequence_errors * 100.0) / self.packets_received

    def get_packet_rate(self):
        elapsed = time.time() - self.start_time
        return self.packets_received / elapsed if elapsed > 0 else 0

    def log_stats(self):
        logger.info("=== PACKET STATISTICS ===")
        logger.info(f"Packets Received: {self.packets_received}")
        logger.info(f"Bytes Received: {self.bytes_received}")
        logger.info(f"Packet Rate: {self.get_packet_rate():.1f} pkt/s")
        logger.info(f"Sequence Errors: {self.sequence_errors}")
        logger.info(f"Packet Loss Rate: {self.get_packet_loss_rate():.2f}%")
        logger.info(f"Duplicate Packets: {self.duplicate_packets}")
        logger.info(f"Out-of-order Packets: {self.out_of_order_packets}")
        logger.info("=========================")

class VideoRecorder:
    def __init__(self, output_dir="recordings"):
        self.output_dir = output_dir
        self.video_writer = None
        self.raw_file = None
        self.recording = False
        self.frame_count = 0
        self.start_time = None
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate timestamp-based filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_filename = os.path.join(output_dir, f"rtsp_stream_{timestamp}.avi")
        self.raw_filename = os.path.join(output_dir, f"rtsp_stream_{timestamp}.mjpeg")
        
        logger.info(f"VIDEO: Will be saved to: {self.video_filename}")
        logger.info(f"RAW MJPEG: Will be saved to: {self.raw_filename}")

    def start_recording(self, width, height, fps=15.0):
        if self.recording:
            return
            
        # Initialize video writer (AVI format with XVID codec)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(
            self.video_filename, fourcc, fps, (width, height)
        )
        
        # Initialize raw MJPEG file
        self.raw_file = open(self.raw_filename, 'wb')
        
        self.recording = True
        self.start_time = time.time()
        self.frame_count = 0
        
        logger.info(f"RECORDING STARTED: {width}x{height} @ {fps} FPS")

    def add_frame(self, frame, raw_jpeg_data):
        if not self.recording:
            return
            
        # Write processed frame to video file
        if self.video_writer is not None and frame is not None:
            self.video_writer.write(frame)
        
        # Write raw JPEG data to raw file with MJPEG boundary
        if self.raw_file is not None:
            self.raw_file.write(b'--FRAME\r\n')
            self.raw_file.write(f'Content-Length: {len(raw_jpeg_data)}\r\n\r\n'.encode())
            self.raw_file.write(raw_jpeg_data)
            self.raw_file.write(b'\r\n')
        
        self.frame_count += 1
        
        if self.frame_count % 100 == 0:
            elapsed = time.time() - self.start_time
            logger.info(f"RECORDED: {self.frame_count} frames in {elapsed:.1f}s")

    def stop_recording(self):
        if not self.recording:
            return
            
        self.recording = False
        
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        
        if self.raw_file is not None:
            self.raw_file.close()
            self.raw_file = None
        
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        logger.info(f"RECORDING STOPPED")
        logger.info(f"Total frames recorded: {self.frame_count}")
        logger.info(f"Recording duration: {elapsed:.1f} seconds")
        logger.info(f"Average FPS: {self.frame_count/elapsed:.1f}" if elapsed > 0 else "N/A")
        
        # Check file sizes
        if os.path.exists(self.video_filename):
            size_mb = os.path.getsize(self.video_filename) / (1024 * 1024)
            logger.info(f"Video file size: {size_mb:.1f} MB")
        
        if os.path.exists(self.raw_filename):
            size_mb = os.path.getsize(self.raw_filename) / (1024 * 1024)
            logger.info(f"Raw MJPEG file size: {size_mb:.1f} MB")

class EnhancedJpegFrame(JpegFrame):
    def __init__(self, first_pkt: RtpJpegPacket):
        self.packet_count = 0
        self.start_time = time.time()
        self.first_packet_time = time.time()

        super().__init__(first_pkt)

    def add_packet(self, pkt: RtpJpegPacket):
        super().add_packet(pkt)
        self.packet_count += 1

    def get_assembly_time(self):
        return time.time() - self.first_packet_time

class RtspClient:
    def __init__(self, server, port, rtsp_uri):
        self.server = server
        self.port = port
        self.cseq = 0
        self.session_id = ""
        self.rtsp_uri = rtsp_uri

    def connect(self):
        self.sock = socket.create_connection((self.server, self.port))
        self.send_request("OPTIONS", "*")

    def send_request(self, method, uri, headers=None):
        if headers is None:
            headers = {}

        request = f"{method} {uri} RTSP/1.0\r\n"
        request += f"CSeq: {self.cseq}\r\n"
        if self.session_id:
            request += f"Session: {self.session_id}\r\n"

        for key, value in headers.items():
            request += f"{key}: {value}\r\n"

        request += "User-Agent: RtspClient\r\n"
        request += "\r\n"

        self.sock.sendall(request.encode())
        response = self.sock.recv(4096)
        print("Response:", response.decode())

        self.cseq += 1

    def describe(self):
        self.send_request("DESCRIBE", self.rtsp_uri, {"Accept": "application/sdp"})

    def setup(self, transport):
        self.send_request("SETUP", self.rtsp_uri, {"Transport": transport})

    def play(self):
        self.send_request("PLAY", self.rtsp_uri)

    def pause(self):
        self.send_request("PAUSE", self.rtsp_uri)

    def teardown(self):
        self.send_request("TEARDOWN", self.rtsp_uri)

    def start_receiving_video_stream(self, rtp_port, rtcp_port):
        self.rtp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)  # 1MB recv buf
        self.rtp_socket.bind(("0.0.0.0", rtp_port))

        self.rtcp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)  # 1MB recv buf
        self.rtcp_socket.bind(("0.0.0.0", rtcp_port))

        print(f"Listening for RTP packets on port {rtp_port}")
        print(f"Listening for RTCP packets on port {rtcp_port}")

        self.handle_rtp_packet()

    def handle_rtp_packet(self):
        # Show the received video in an OpenCV window (same as original)
        window_name = 'MJPEG Stream'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

        jpeg_frame = None
        last_ts = None

        while True:
            # Read one RTP packet (UDP)
            rtp_data, _ = self.rtp_socket.recvfrom(8192)
            pkt = RtpJpegPacket(rtp_data)

            # Only care about JPEG PT=26
            if pkt.get_payload_type() != 26:
                continue

            # Start a new frame on timestamp change or when fragment offset == 0
            if last_ts is None or pkt.timestamp != last_ts or pkt.get_frag_offset() == 0:
                jpeg_frame = JpegFrame(pkt)
                last_ts = pkt.timestamp
            else:
                if jpeg_frame is not None:
                    jpeg_frame.add_packet(pkt)
                else:
                    # We missed the first fragment; wait for the next frame start
                    continue

            # Marker bit => last fragment of the frame
            if pkt.get_marker():
                if jpeg_frame and jpeg_frame.ok:
                    buf = jpeg_frame.get_data()
                    frame = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        frame = cv2.flip(frame, 0)  # your stream is vertically flipped
                        cv2.imshow(window_name, frame)
                        if (cv2.waitKey(1) & 0xFF) == ord('q'):
                            break
                    # else: decode failed; drop silently
                # Reset for next frame
                jpeg_frame = None
                last_ts = None

    def handle_rtcp_packet(self):
        while True:
            rtcp_data, addr = self.rtcp_socket.recvfrom(8192)
            print("Received rtcp packet:", rtcp_data)

class EnhancedRtspClient(RtspClient):
    def __init__(self, server, port, rtsp_uri, enable_recording=True):
        super().__init__(server, port, rtsp_uri)
        self.frame_stats = FrameStats()
        self.packet_stats = PacketStats()
        self.recorder = VideoRecorder() if enable_recording else None
        self.recording_started = False

    def handle_rtp_packet(self):
        logger.info("RTSP CLIENT: Starting enhanced client with recording and diagnostics")
        logger.info(f"TARGET: {self.rtsp_uri}")
        
        window_name = 'MJPEG Stream - Enhanced with Recording'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

        jpeg_frame = None
        last_ts = None

        try:
            while True:
                # Read one RTP packet (UDP)
                rtp_data, addr = self.rtp_socket.recvfrom(8192)
                pkt = RtpJpegPacket(rtp_data)
                
                # Update packet statistics
                self.packet_stats.add_packet(pkt)

                # Only care about JPEG PT=26
                if pkt.get_payload_type() != 26:
                    continue

                # Log detailed packet info
                logger.debug(f"RTP PACKET: seq={pkt.sequence_number}, ts={pkt.timestamp}, "
                           f"off={pkt.get_frag_offset()}, size={len(pkt.get_jpeg_data())} bytes, "
                           f"marker={pkt.get_marker()}")

                # Start a new frame on timestamp change or when fragment offset == 0
                if last_ts is None or pkt.timestamp != last_ts or pkt.get_frag_offset() == 0:
                    if jpeg_frame is not None:
                        logger.warning(f"INCOMPLETE FRAME: discarded (ts={last_ts})")
                    
                    jpeg_frame = EnhancedJpegFrame(pkt)
                    last_ts = pkt.timestamp
                    
                    logger.debug(f"NEW FRAME: ts={pkt.timestamp}, "
                               f"resolution={pkt.get_width()}x{pkt.get_height()}, q={pkt.q}")
                else:
                    if jpeg_frame is not None:
                        jpeg_frame.add_packet(pkt)
                    else:
                        logger.debug("ORPHANED PACKET: no frame context")
                        continue

                # Marker bit => last fragment of the frame
                if pkt.get_marker():
                    if jpeg_frame and jpeg_frame.ok:
                        # Measure decode time
                        decode_start = time.time()
                        buf = jpeg_frame.get_data()
                        
                        frame = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
                        decode_time = time.time() - decode_start
                        
                        if frame is not None:
                            frame = cv2.flip(frame, 0)
                            
                            # Start recording on first successful frame
                            if self.recorder and not self.recording_started:
                                height, width = frame.shape[:2]
                                self.recorder.start_recording(width, height, fps=15.0)
                                self.recording_started = True
                            
                            # Record frame
                            if self.recorder and self.recording_started:
                                self.recorder.add_frame(frame, buf)
                            
                            # Add frame statistics overlay
                            fps = self.frame_stats.get_fps()
                            frame_size = len(buf)
                            
                            # Draw stats on frame
                            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(frame, f"Size: {frame_size} bytes", (10, 60), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(frame, f"Packets: {jpeg_frame.packet_count}", (10, 90), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(frame, f"Loss: {self.packet_stats.get_packet_loss_rate():.1f}%", (10, 120), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            if self.recorder and self.recording_started:
                                cv2.putText(frame, "[REC]", (10, 150), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                cv2.putText(frame, f"Frames: {self.recorder.frame_count}", (10, 180), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            
                            cv2.imshow(window_name, frame)
                            
                            # Update frame statistics
                            self.frame_stats.add_frame(frame_size, decode_time, 
                                                     jpeg_frame.packet_count, corrupted=False)
                            
                            logger.debug(f"FRAME OK: {frame.shape[1]}x{frame.shape[0]}, "
                                       f"{frame_size} bytes, {jpeg_frame.packet_count} packets, "
                                       f"decode: {decode_time*1000:.1f}ms")
                            
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                break
                            elif key == ord('r'):  # Toggle recording
                                if self.recorder:
                                    if self.recording_started:
                                        self.recorder.stop_recording()
                                        self.recording_started = False
                                        logger.info("RECORDING PAUSED (press 'r' to resume)")
                                    else:
                                        height, width = frame.shape[:2]
                                        self.recorder.start_recording(width, height, fps=15.0)
                                        self.recording_started = True
                                        logger.info("RECORDING RESUMED")
                        else:
                            logger.error(f"FRAME DECODE FAILED: {len(buf)} bytes")
                            self.frame_stats.add_frame(len(buf), decode_time, 
                                                     jpeg_frame.packet_count, corrupted=True)
                    else:
                        logger.warning(f"CORRUPTED FRAME: discarded (ts={last_ts})")
                        if jpeg_frame:
                            self.frame_stats.add_frame(len(jpeg_frame.buf), 0, 
                                                     jpeg_frame.packet_count, corrupted=True)
                    
                    # Reset for next frame
                    jpeg_frame = None
                    last_ts = None

        except KeyboardInterrupt:
            logger.info("STOPPING CLIENT...")
        finally:
            cv2.destroyAllWindows()
            
            # Stop recording
            if self.recorder and self.recording_started:
                self.recorder.stop_recording()
            
            # Log final statistics
            logger.info("=== FINAL SESSION STATISTICS ===")
            self.frame_stats.log_stats()
            self.packet_stats.log_stats()
            
            total_time = time.time() - self.frame_stats.start_time
            logger.info(f"Total Session Time: {total_time:.1f} seconds")
            logger.info("================================")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python rtsp_saver.py <server> <port> [rtp_port] [--no-record]")
        sys.exit(1)

    server, port = sys.argv[1], int(sys.argv[2])
    rtsp_uri = f"rtsp://{server}:{port}/mjpeg/1"
    
    # Check if recording is disabled
    enable_recording = "--no-record" not in sys.argv
    
    logger.info(f"Enhanced RTSP Client with Recording starting...")
    logger.info(f"Target: {rtsp_uri}")
    logger.info(f"Recording: {'Enabled' if enable_recording else 'Disabled'}")
    logger.info("Controls: 'q' = quit, 'r' = toggle recording")
    
    client = EnhancedRtspClient(server, port, rtsp_uri, enable_recording=enable_recording)
    client.connect()
    client.describe()

    rtp_port = 5000
    rtcp_port = 5001
    if len(sys.argv) >= 4 and sys.argv[3].isdigit():
        rtp_port = int(sys.argv[3])
        rtcp_port = rtp_port + 1

    transport_header = f"RTP/AVP;unicast;client_port={rtp_port}-{rtcp_port}"
    client.setup(transport_header)
    client.play()

    logger.info(f"Starting video reception and recording on ports {rtp_port}/{rtcp_port}")
    client.start_receiving_video_stream(rtp_port, rtcp_port)

    client.pause()
    client.teardown()
    logger.info("Session ended")