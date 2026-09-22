"""Terminal dashboard implemented as a public IMU consumer."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import sys
import time
from typing import Optional, TextIO

from .events import (
    ClockDiagnostic,
    ConnectionDiagnostic,
    ConnectionState,
    Diagnostic,
    ImuConsumer,
    ImuSample,
    PacketDiagnostic,
    TimingDiagnostic,
    Vector3,
)

class AnalysisConsumer(ImuConsumer):
    def __init__(self):
        self.data = []
        self.saving = False

    def start(self):
        self.data = []
        self.saving = True

    def end(self) -> list[ImuSample]:
        self.saving = False
        return self.data

    def on_sample(self, sample: ImuSample) -> None:
        if self.saving:
            self.data.append(sample)

    def on_diagnostic(self, diagnostic: Diagnostic) -> None:
        if isinstance(diagnostic, (PacketDiagnostic, TimingDiagnostic)):
            return
        else:
            print("Diagnostic:", diagnostic)

    async def render_forever(self) -> None:
        print("Hello")
