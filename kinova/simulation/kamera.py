"""Farb- und Tiefenstream des Kinova-Kameramoduls über GStreamer (separates Programm, LGPL-2.1).

Pipelines nach Kinovas ``ros_kortex_vision`` (RTSP vom Arm): Farbe H.264 1280×720, Tiefe als GStreamer-Rohformat
(``rtpgstdepay``) GRAY16_LE 480×270. ``gst-launch-1.0`` schreibt die Rohbilder nach stdout; ein Lesethread hält das
jeweils letzte Bild mit dem Zeitstempel des Mac (die Uhr des Arms ist nicht gestellt).
Tiefenwerte: Millimeter (ANNAHME aus dem Tiefenbild der Home-Pose, am Arm gegen die Kamerahöhe geprüft).
"""

from __future__ import annotations

import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass

import numpy as np

STANDARD_IP = "192.168.1.10"


@dataclass(frozen=True)
class Strom:
    name: str
    breite: int
    hoehe: int
    kanaele: int
    dtype: str
    pipeline: str

    @property
    def bildgroesse(self) -> int:
        return self.breite * self.hoehe * self.kanaele * np.dtype(self.dtype).itemsize


def stroeme(ip: str = STANDARD_IP) -> dict[str, Strom]:
    return {
        "farbe": Strom("farbe", 1280, 720, 3, "u1",
                       f"rtspsrc location=rtsp://{ip}/color latency=30 ! rtph264depay ! avdec_h264 ! videoconvert ! "
                       "video/x-raw,format=BGR,width=1280,height=720 ! fdsink fd=1"),
        "tiefe": Strom("tiefe", 480, 270, 1, "<u2",
                       f"rtspsrc location=rtsp://{ip}/depth latency=30 ! rtpgstdepay ! "
                       "video/x-raw,format=GRAY16_LE,width=480,height=270 ! fdsink fd=1"),
    }


class Leser:
    """Startet/stoppt einen GStreamer-Prozess und liefert das letzte Bild."""

    def __init__(self, strom: Strom):
        self.strom = strom
        self._proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._bild: np.ndarray | None = None
        self._zeit = 0.0
        self._zaehler = 0
        self._sperre = threading.Lock()
        self._neu = threading.Condition(self._sperre)
        self.fehler: str | None = None

    @property
    def laeuft(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self) -> None:
        if self.laeuft:
            return
        gst = shutil.which("gst-launch-1.0")
        if gst is None:
            self.fehler = "gst-launch-1.0 nicht gefunden (brew install gstreamer)"
            raise RuntimeError(self.fehler)
        self.fehler = None
        self._proc = subprocess.Popen([gst, "-q", *self.strom.pipeline.split()], stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, bufsize=0)
        self._thread = threading.Thread(target=self._lesen, name=f"kamera-{self.strom.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Mit SIGINT beenden: gst-launch schließt dann die RTSP-Sitzung (TEARDOWN). Das Kameramodul bedient je
        Stream nur einen Empfänger (sonst RTSP 503, gemessen 9.11.2026); ein hart beendeter Prozess hielte die
        Sitzung bis zum Zeitablauf des Arms belegt."""
        proc, self._proc = self._proc, None
        if proc is not None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
        with self._sperre:
            self._bild = None
            self._neu.notify_all()

    def _lesen(self) -> None:
        proc = self._proc
        n = self.strom.bildgroesse
        puffer = bytearray(n)
        sicht = memoryview(puffer)
        while proc is not None and proc.poll() is None:
            gelesen = 0
            while gelesen < n:
                stueck = proc.stdout.readinto(sicht[gelesen:])
                if not stueck:
                    self.fehler = "Stream beendet"
                    return
                gelesen += stueck
            bild = np.frombuffer(bytes(puffer), dtype=self.strom.dtype)
            form = (self.strom.hoehe, self.strom.breite) + ((self.strom.kanaele,) if self.strom.kanaele > 1 else ())
            with self._sperre:
                self._bild = bild.reshape(form)
                self._zeit = time.time()
                self._zaehler += 1
                self._neu.notify_all()

    def letztes(self) -> tuple[np.ndarray | None, float, int]:
        with self._sperre:
            return self._bild, self._zeit, self._zaehler

    def naechstes(self, nach: int, zeitlimit_s: float = 3.0) -> tuple[np.ndarray | None, float, int]:
        """Wartet auf ein Bild mit Zähler > nach (frisches Bild nach einer Bewegung)."""
        ende = time.time() + zeitlimit_s
        with self._neu:
            while self._zaehler <= nach and self.laeuft:
                rest = ende - time.time()
                if rest <= 0:
                    break
                self._neu.wait(rest)
            return self._bild, self._zeit, self._zaehler

    def frisch(self, anzahl: int = 1, zeitlimit_s: float = 5.0, anlauf_s: float = 30.0) -> list[np.ndarray]:
        """Startet bei Bedarf und liefert ``anzahl`` neue Bilder ab jetzt. Endet der Stream sofort (RTSP 503: Sitzung
        eines hart beendeten Vorgängers noch belegt), wird bis ``anlauf_s`` lang neu verbunden."""
        ende = time.time() + anlauf_s
        while True:
            self.start()
            _, _, z = self.letztes()
            b, _, z_neu = self.naechstes(z, zeitlimit_s)
            if b is not None and z_neu != z:
                break
            if time.time() > ende:
                raise RuntimeError(f"kein Bild vom Stream {self.strom.name} ({self.fehler or 'Zeitlimit'})")
            self.stop()
            time.sleep(2.0)
        _, _, z = self.letztes()
        bilder = []
        while len(bilder) < anzahl:
            b, _, z_neu = self.naechstes(z, zeitlimit_s)
            if b is None or z_neu == z:
                raise RuntimeError(f"kein Bild vom Stream {self.strom.name} ({self.fehler or 'Zeitlimit'})")
            bilder.append(b.copy())
            z = z_neu
        return bilder


def tiefe_einfaerben(tiefe_mm: np.ndarray, nah_mm: int = 150, fern_mm: int = 1000) -> np.ndarray:
    import cv2

    skaliert = np.clip((tiefe_mm.astype(np.float32) - nah_mm) * (255.0 / (fern_mm - nah_mm)), 0, 255).astype(np.uint8)
    bild = cv2.applyColorMap(255 - skaliert, cv2.COLORMAP_TURBO)  # nah = rot, fern = blau
    bild[tiefe_mm == 0] = 0  # keine Messung: schwarz
    return bild
