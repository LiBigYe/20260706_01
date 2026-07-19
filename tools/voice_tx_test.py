#!/usr/bin/env python3
"""Generate an audio test frame compatible with the current STM32 v5 sender.

The waveform matches Core/Src/transmitter.c and Core/Src/voice_fec.c:
  * 16 kHz mono PCM
  * 200 ms continuous 1500/2400 Hz preamble, changing every 40 ms
  * 30 ms 1800 Hz sync slot (20 ms tone + 10 ms silence)
  * 4-FSK data slots (20 ms tone + 10 ms silence)
  * Hamming(7,4), block interleaving, triplicated LEN and CRC-8
  * 120 ms continuous 2400 Hz postamble

No third-party package is required. On Windows, --play sends the generated
WAV to the default output device through winsound.
"""

from __future__ import annotations

import argparse
import array
import math
import sys
import wave
from pathlib import Path


SAMPLE_RATE = 16_000
TONE_SAMPLES = 320
GUARD_SAMPLES = 160
PILOT_PERIOD_SAMPLES = 640
POSTAMBLE_SAMPLES = 1_920
FREQUENCIES = (1500, 1800, 2100, 2400)
MAX_TEXT_BYTES = 48


def crc8(data: bytes) -> int:
    """CRC-8, polynomial 0x07, initial value 0x00."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def hamming_encode(nibble: int) -> int:
    """Bit-exact equivalent of VoiceFEC_HammingEncode()."""
    d1 = (nibble >> 0) & 1
    d2 = (nibble >> 1) & 1
    d3 = (nibble >> 2) & 1
    d4 = (nibble >> 3) & 1
    p1 = d1 ^ d2 ^ d4
    p2 = d1 ^ d3 ^ d4
    p3 = d2 ^ d3 ^ d4
    return (p1 << 6) | (p2 << 5) | (d1 << 4) | (p3 << 3) | (d2 << 2) | (d3 << 1) | d4


def encode_bytes_to_symbols(data: bytes) -> list[int]:
    """Hamming encode, row-write/column-read interleave, then pack 2 bits."""
    codewords: list[int] = []
    for byte in data:
        codewords.append(hamming_encode(byte >> 4))
        codewords.append(hamming_encode(byte & 0x0F))

    bits: list[int] = []
    for column in range(7):
        for codeword in codewords:
            bits.append((codeword >> (6 - column)) & 1)

    return [(bits[index] << 1) | bits[index + 1] for index in range(0, len(bits), 2)]


def build_data_symbols(payload: bytes) -> list[int]:
    """Exact equivalent of VoiceFEC_BuildDataSymbols()."""
    payload_len = len(payload)
    if not 1 <= payload_len <= 51:
        raise ValueError("payload length must be in the range 1..51")

    length_symbols = encode_bytes_to_symbols(bytes((payload_len,))) * 3
    crc = crc8(bytes((payload_len,)) + payload)
    body_symbols = encode_bytes_to_symbols(payload + bytes((crc,)))
    return length_symbols + body_symbols


def append_tone(samples: array.array, frequency: int, count: int, amplitude: float, phase: float) -> float:
    """Append DDS-like sine samples. The STM32 advances phase before output."""
    phase_step = frequency / SAMPLE_RATE
    scale = int(round(amplitude * 32767))
    for _ in range(count):
        phase += phase_step
        if phase >= 1.0:
            phase -= 1.0
        samples.append(int(round(scale * math.sin(math.tau * phase))))
    return phase


def append_silence(samples: array.array, count: int) -> None:
    samples.extend([0] * count)


def build_waveform(symbols: list[int], amplitude: float) -> array.array:
    samples = array.array("h")
    phase = 0.0

    # transmitter.c: five 40 ms continuous pilot sections, no guards.
    for digit in (0, 3, 0, 3, 0):
        phase = append_tone(samples, FREQUENCIES[digit], PILOT_PERIOD_SAMPLES, amplitude, phase)

    # Sync retains phase from the preamble. Its guard resets the DDS phase.
    phase = append_tone(samples, FREQUENCIES[1], TONE_SAMPLES, amplitude, phase)
    append_silence(samples, GUARD_SAMPLES)
    phase = 0.0

    for digit in symbols:
        phase = append_tone(samples, FREQUENCIES[digit], TONE_SAMPLES, amplitude, phase)
        append_silence(samples, GUARD_SAMPLES)
        phase = 0.0  # PWM_DDS_OutputMidscale() resets phase during every guard.

    # The current transmitter starts the postamble immediately after the final guard.
    append_tone(samples, FREQUENCIES[3], POSTAMBLE_SAMPLES, amplitude, phase)
    return samples


def parse_target_mask(value: str) -> int:
    """Parse '0' for broadcast or a comma-separated device-ID list (1..9)."""
    value = value.strip()
    if value == "0":
        return 0

    mask = 0
    for item in value.split(","):
        device_id = int(item.strip(), 10)
        if not 1 <= device_id <= 9:
            raise argparse.ArgumentTypeError("target IDs must be in the range 1..9, or use 0 for broadcast")
        mask |= 1 << (device_id - 1)
    return mask


def write_wav(path: Path, samples: array.array) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if sys.byteorder != "little":
        samples.byteswap()
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(SAMPLE_RATE)
        output.writeframes(samples.tobytes())


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a STM32 v5 4-FSK test frame.")
    parser.add_argument("text", help="ASCII message body, at most 48 bytes")
    parser.add_argument("--source", type=int, default=1, help="sender ID, 1..9 (default: 1)")
    parser.add_argument("--to", default="0", help="0 for broadcast, or target IDs such as 2 or 2,5")
    parser.add_argument("--amplitude", type=float, default=0.70, help="PCM peak fraction, 0.05..0.95 (default: 0.70)")
    parser.add_argument("--output", type=Path, default=Path("tmp/voice_tx_test.wav"), help="output WAV path")
    parser.add_argument("--play", action="store_true", help="play the WAV through the Windows default output device")
    arguments = parser.parse_args()

    if not 1 <= arguments.source <= 9:
        parser.error("--source must be in the range 1..9")
    if not 0.05 <= arguments.amplitude <= 0.95:
        parser.error("--amplitude must be in the range 0.05..0.95")

    try:
        text = arguments.text.encode("ascii")
    except UnicodeEncodeError as error:
        parser.error(f"text must be ASCII: {error}")
    if len(text) > MAX_TEXT_BYTES:
        parser.error("text exceeds the firmware limit of 48 bytes")

    target_mask = parse_target_mask(arguments.to)
    payload = bytes((arguments.source, target_mask & 0xFF, target_mask >> 8)) + text
    symbols = build_data_symbols(payload)
    samples = build_waveform(symbols, arguments.amplitude)
    write_wav(arguments.output, samples)

    duration = len(samples) / SAMPLE_RATE
    print(f"wrote {arguments.output} ({duration:.3f} s, {len(symbols)} data symbols)")
    print(f"source={arguments.source}, target_mask=0x{target_mask:03X}, payload={payload!r}, crc=0x{crc8(bytes((len(payload),)) + payload):02X}")

    if arguments.play:
        if sys.platform != "win32":
            parser.error("--play currently requires Windows winsound")
        import winsound

        winsound.PlaySound(str(arguments.output.resolve()), winsound.SND_FILENAME)


if __name__ == "__main__":
    main()
