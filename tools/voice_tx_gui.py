#!/usr/bin/env python3
"""Simple Windows GUI sender for the STM32 4-FSK voice protocol."""

from __future__ import annotations

import threading
import tempfile
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import winsound

from voice_tx_test import (
    MAX_TEXT_BYTES,
    build_data_symbols,
    build_waveform,
    parse_target_mask,
    write_wav,
)


# A user-writable absolute location prevents failures when launched from a
# shortcut, IDE, or another process with a different current directory.
DEFAULT_OUTPUT = Path(tempfile.gettempdir()) / "stm32_voice_tx_gui.wav"


class VoiceTxGui(ttk.Frame):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master, padding=16)
        self.master = master
        self.sending = False
        self.text = tk.StringVar(value="TEST")
        self.source = tk.StringVar(value="1")
        self.targets = tk.StringVar(value="0")
        self.amplitude = tk.DoubleVar(value=0.70)
        self.status = tk.StringVar(value="Ready")
        self._build()

    def _build(self) -> None:
        self.grid(sticky="nsew")
        self.columnconfigure(1, weight=1)
        self.master.columnconfigure(0, weight=1)

        ttk.Label(self, text="Message").grid(row=0, column=0, sticky="w", pady=(0, 8))
        entry = ttk.Entry(self, textvariable=self.text, width=42)
        entry.grid(row=0, column=1, sticky="ew", pady=(0, 8))
        entry.focus_set()

        ttk.Label(self, text="Source ID").grid(row=1, column=0, sticky="w", pady=8)
        ttk.Spinbox(self, from_=1, to=9, textvariable=self.source, width=8).grid(
            row=1, column=1, sticky="w", pady=8
        )

        ttk.Label(self, text="Target IDs").grid(row=2, column=0, sticky="w", pady=8)
        ttk.Entry(self, textvariable=self.targets, width=20).grid(row=2, column=1, sticky="w", pady=8)
        ttk.Label(self, text="Use 0 for broadcast, or e.g. 2,5").grid(
            row=3, column=1, sticky="w", pady=(0, 8)
        )

        ttk.Label(self, text="Output level").grid(row=4, column=0, sticky="w", pady=8)
        level = ttk.Scale(self, from_=0.05, to=0.95, variable=self.amplitude, orient="horizontal")
        level.grid(row=4, column=1, sticky="ew", pady=8)
        self.level_label = ttk.Label(self, width=5)
        self.level_label.grid(row=4, column=2, padx=(8, 0))
        self._update_level_label()
        level.configure(command=lambda _: self._update_level_label())

        self.send_button = ttk.Button(self, text="Send", command=self._start_send)
        self.send_button.grid(row=5, column=1, sticky="e", pady=(16, 8))
        ttk.Label(self, textvariable=self.status).grid(row=6, column=0, columnspan=3, sticky="w")

        self.master.bind("<Return>", lambda _: self._start_send())

    def _update_level_label(self) -> None:
        self.level_label.configure(text=f"{self.amplitude.get():.2f}")

    def _start_send(self) -> None:
        if self.sending:
            return
        try:
            text = self.text.get().encode("ascii")
            if len(text) > MAX_TEXT_BYTES:
                raise ValueError(f"message is limited to {MAX_TEXT_BYTES} ASCII bytes")
            source = int(self.source.get(), 10)
            if not 1 <= source <= 9:
                raise ValueError("source ID must be 1 through 9")
            target_mask = parse_target_mask(self.targets.get())
        except (UnicodeEncodeError, ValueError) as error:
            messagebox.showerror("Invalid frame", str(error))
            return

        payload = bytes((source, target_mask & 0xFF, target_mask >> 8)) + text
        symbols = build_data_symbols(payload)
        samples = build_waveform(symbols, self.amplitude.get())
        try:
            write_wav(DEFAULT_OUTPUT, samples)
        except OSError as error:
            messagebox.showerror("File error", str(error))
            return

        self.sending = True
        self.send_button.configure(state="disabled")
        self.status.set(f"Sending {len(samples) / 16000:.3f} s, {len(symbols)} data symbols")
        threading.Thread(target=self._play, daemon=True).start()

    def _play(self) -> None:
        try:
            winsound.PlaySound(str(DEFAULT_OUTPUT.resolve()), winsound.SND_FILENAME)
            self.master.after(0, lambda: self.status.set("Sent"))
        except RuntimeError as error:
            self.master.after(0, lambda: messagebox.showerror("Audio error", str(error)))
            self.master.after(0, lambda: self.status.set("Audio output failed"))
        finally:
            self.master.after(0, self._send_finished)

    def _send_finished(self) -> None:
        self.sending = False
        self.send_button.configure(state="normal")


def main() -> None:
    root = tk.Tk()
    root.title("STM32 4-FSK Test Sender")
    root.minsize(440, 250)
    VoiceTxGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
