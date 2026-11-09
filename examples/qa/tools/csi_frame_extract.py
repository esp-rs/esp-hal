#!/usr/bin/env python3
"""Extract a RAW8 frame exported by `mipi-csi-sc2336-first-frame` (`CSI_FRAME_EXPORT=1`).

Reads a console log, checks the `@@SC2336FRAME` framing, every chunk CRC, the
chunk order and the whole-frame CRC, and only then writes:

- `<stem>.raw`: the exact frame bytes, row-major, no header.
- `<stem>-mosaic-gray.png`: the RAW8 mosaic shown as grayscale, one pixel per byte.
  This is not a grayscale image of the scene, every pixel is one Bayer color.
- `<stem>-bggr-halfres.png`: one RGB pixel per 2x2 BGGR quad (G averaged), 512x300,
  no interpolation, no white balance. Only written when the header says BGGR and
  register 0x3221 (mirror/flip) reads 00, the state the official mode table assumes.

Only the Python standard library is used.
"""

import argparse
import base64
import re
import struct
import sys
import zlib
from pathlib import Path

PREFIX = "@@SC2336FRAME"
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
SUPPORTED_VERSION = "1"


class ExportError(Exception):
    pass


def parse_fields(text):
    fields = {}
    for token in text.split():
        if "=" not in token:
            raise ExportError(f"malformed field {token!r}")
        key, value = token.split("=", 1)
        fields[key] = value
    return fields


def device_checksum32(data):
    """Same as `checksum32` in the QA binary, which its verdict logs."""
    acc = 0
    for byte in data:
        acc = ((acc * 16777619) & 0xFFFFFFFF) ^ byte
    return acc


def read_frames(log_path):
    """Returns {frame_id: {"begin": fields, "chunks": {index: bytes}, "end": fields}}."""
    frames = {}
    current = None
    raw = Path(log_path).read_bytes().decode("utf-8", errors="replace")
    for line_number, line in enumerate(raw.splitlines(), start=1):
        line = ANSI_ESCAPE.sub("", line).replace("\r", "")
        start = line.find(PREFIX)
        if start < 0:
            continue
        body = line[start + len(PREFIX) :].strip()
        kind, _, rest = body.partition(" ")
        if kind == "BEGIN":
            fields = parse_fields(rest)
            frame_id = fields.get("id")
            if frame_id in frames:
                print(f"note: frame id {frame_id} begins again at line {line_number}, keeping the later one")
            current = {"begin": fields, "chunks": {}, "end": None, "errors": [], "line": line_number}
            frames[frame_id] = current
        elif kind == "D":
            parts = rest.split(" ")
            if current is None or len(parts) != 5:
                continue
            frame_id, index, offset, crc, payload = parts
            if frame_id != current["begin"].get("id"):
                current["errors"].append(f"line {line_number}: chunk of frame {frame_id} inside frame {current['begin'].get('id')}")
                continue
            try:
                index = int(index)
                offset = int(offset, 16)
                crc = int(crc, 16)
                data = base64.b64decode(payload, validate=True)
            except ValueError as err:
                current["errors"].append(f"line {line_number}: malformed chunk line: {err}")
                continue
            if zlib.crc32(data) != crc:
                current["errors"].append(f"line {line_number}: chunk {index} CRC mismatch")
                continue
            expected_offset = index * int(current["begin"]["chunk_bytes"])
            if offset != expected_offset:
                current["errors"].append(f"line {line_number}: chunk {index} offset {offset:x} != {expected_offset:x}")
                continue
            previous = current["chunks"].get(index)
            if previous is not None and previous != data:
                current["errors"].append(f"line {line_number}: chunk {index} repeated with different data")
                continue
            current["chunks"][index] = data
        elif kind == "END":
            if current is not None:
                current["end"] = parse_fields(rest)
                current = None
    return frames


def assemble(frame):
    begin = frame["begin"]
    if begin.get("v") != SUPPORTED_VERSION:
        raise ExportError(f"unsupported export version {begin.get('v')!r}")
    if frame["errors"]:
        raise ExportError("corrupted chunks:\n  " + "\n  ".join(frame["errors"][:20]))
    if frame["end"] is None:
        raise ExportError("no END line, the export is truncated")

    width = int(begin["width"])
    height = int(begin["height"])
    length = int(begin["len"])
    chunk_bytes = int(begin["chunk_bytes"])
    chunks = int(begin["chunks"])
    if begin.get("format") != "RAW8":
        raise ExportError(f"format {begin.get('format')!r} is not RAW8")
    if length != width * height:
        raise ExportError(f"len={length} does not match {width}x{height} RAW8")
    if chunks != -(-length // chunk_bytes):
        raise ExportError(f"chunks={chunks} does not match len={length} chunk_bytes={chunk_bytes}")

    end = frame["end"]
    for key in ("id", "len", "chunks", "crc32"):
        if end.get(key) != begin.get(key):
            raise ExportError(f"END {key}={end.get(key)} does not match BEGIN {key}={begin.get(key)}")

    missing = [index for index in range(chunks) if index not in frame["chunks"]]
    if missing:
        raise ExportError(f"{len(missing)} of {chunks} chunks missing, first {missing[:10]}")
    extra = sorted(index for index in frame["chunks"] if index >= chunks)
    if extra:
        raise ExportError(f"chunk indices beyond {chunks - 1}: {extra[:10]}")

    data = b"".join(frame["chunks"][index] for index in range(chunks))
    for index in range(chunks - 1):
        if len(frame["chunks"][index]) != chunk_bytes:
            raise ExportError(f"chunk {index} has {len(frame['chunks'][index])} bytes, expected {chunk_bytes}")
    if len(data) != length:
        raise ExportError(f"payload has {len(data)} bytes, expected {length}")
    if zlib.crc32(data) != int(begin["crc32"], 16):
        raise ExportError("whole-frame CRC32 mismatch")
    if device_checksum32(data) != int(begin["checksum32"], 16):
        raise ExportError("device checksum32 mismatch")
    return data, width, height


def write_png(path, width, height, rows, color):
    """Writes 8-bit grayscale (color=False) or RGB (color=True) rows as PNG."""
    color_type = 2 if color else 0
    raw = b"".join(b"\x00" + bytes(row) for row in rows)

    def chunk(tag, payload):
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    Path(path).write_bytes(png)


def bggr_halfres_rows(data, width, height):
    for y in range(0, height - 1, 2):
        top = data[y * width : (y + 1) * width]
        bottom = data[(y + 1) * width : (y + 2) * width]
        row = bytearray()
        for x in range(0, width - 1, 2):
            blue = top[x]
            green = (top[x + 1] + bottom[x] + 1) // 2
            red = bottom[x + 1]
            row += bytes((red, green, blue))
        yield row


def report_statistics(data, width, height):
    print(f"pixels={len(data)} min={min(data)} max={max(data)} mean={sum(data) / len(data):.1f}")
    print(f"saturated(255)={data.count(255) / len(data):.2%} zero={data.count(0) / len(data):.2%}")

    planes = {"B (even row, even col)": [], "Gb (even row, odd col)": [], "Gr (odd row, even col)": [], "R (odd row, odd col)": []}
    names = list(planes)
    for y in range(height):
        row = data[y * width : (y + 1) * width]
        planes[names[(y % 2) * 2]].append(row[0::2])
        planes[names[(y % 2) * 2 + 1]].append(row[1::2])
    for name, rows in planes.items():
        values = b"".join(rows)
        print(f"  {name}: mean={sum(values) / len(values):.1f} min={min(values)} max={max(values)}")

    rows = [data[y * width : (y + 1) * width] for y in range(height)]
    duplicate = sum(1 for y in range(1, height) if rows[y] == rows[y - 1])
    constant = sum(1 for row in rows if min(row) == max(row))
    print(f"rows identical to the previous row={duplicate} constant rows={constant}")
    if duplicate > height // 10:
        print("warning: many repeated rows, the DMA may have written the same line more than once")
    if constant > height // 10:
        print("warning: many constant rows, part of the frame may be missing or saturated")

    row_means = [sum(row) / width for row in rows]
    jumps = [(y, row_means[y] - row_means[y - 1]) for y in range(1, height)]
    largest = max(jumps, key=lambda jump: abs(jump[1]))
    print(f"largest row-to-row mean change: {largest[1]:+.1f} at row {largest[0]}")
    if abs(largest[1]) > 40:
        print("warning: a large brightness step between two rows can mean a frame splice or a lost line")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="console log that contains the export")
    parser.add_argument("--out-dir", default=".", help="directory for the output files")
    parser.add_argument("--stem", default="sc2336-1024x600", help="output file name stem")
    parser.add_argument("--id", help="frame id to extract, default: the last complete one")
    args = parser.parse_args()

    frames = read_frames(args.log)
    if not frames:
        print(f"error: no {PREFIX} BEGIN line in {args.log}", file=sys.stderr)
        return 1

    candidates = [args.id] if args.id else list(frames)[::-1]
    errors = []
    for frame_id in candidates:
        frame = frames.get(frame_id)
        if frame is None:
            errors.append(f"frame {frame_id}: not in the log")
            continue
        try:
            data, width, height = assemble(frame)
        except ExportError as err:
            errors.append(f"frame {frame_id} (BEGIN at line {frame['line']}): {err}")
            continue
        break
    else:
        for error in errors:
            print(f"error: {error}", file=sys.stderr)
        print("error: no valid frame, nothing written", file=sys.stderr)
        return 1

    begin = frame["begin"]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_path = out_dir / f"{args.stem}.raw"
    raw_path.write_bytes(data)
    print(f"frame id={begin['id']} {width}x{height} RAW8 len={len(data)} crc32={begin['crc32']} checksum32={begin['checksum32']}: valid")
    print(f"wrote {raw_path}")

    mosaic_path = out_dir / f"{args.stem}-mosaic-gray.png"
    write_png(mosaic_path, width, height, (data[y * width : (y + 1) * width] for y in range(height)), color=False)
    print(f"wrote {mosaic_path} (Bayer mosaic shown as grayscale, not a demosaiced image)")

    if begin.get("test_pattern") == "1":
        print("note: the sensor test pattern was enabled for this frame")

    bayer_confirmed = begin.get("bayer") == "BGGR" and begin.get("reg3221") == "00"
    if bayer_confirmed:
        rgb_path = out_dir / f"{args.stem}-bggr-halfres.png"
        write_png(rgb_path, width // 2, height // 2, bggr_halfres_rows(data, width, height), color=True)
        print(f"wrote {rgb_path} (BGGR, 2x2 binned, no white balance, {begin.get('bayer_source')})")
    else:
        print(
            f"note: no RGB preview, Bayer order not confirmed (bayer={begin.get('bayer')} reg3221={begin.get('reg3221')}, "
            "the mode table order assumes 0x3221=00)"
        )

    report_statistics(data, width, height)
    return 0


if __name__ == "__main__":
    sys.exit(main())
