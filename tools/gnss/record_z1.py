#!/usr/bin/env python3
"""Record Z1 WebUI position telemetry without changing receiver configuration."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import ipaddress
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from z1_protocol import parse_position


ROOT = Path(__file__).resolve().parents[2]
MAX_RESPONSE_BYTES = 262144
FIX_FIELDS = [
    "gps_week", "gps_seconds", "utc_timestamp", "solution_status", "position_type",
    "fix_type", "position_valid", "latitude_deg", "longitude_deg", "height_reported_m",
    "undulation_m", "height_ellipsoid_m", "datum", "sigma_latitude_m",
    "sigma_longitude_m", "sigma_height_m", "correction_age_s", "solution_age_s",
    "satellites_tracked", "satellites_used", "epoch_coherent", "time_status",
    "time_valid", "quality_usable", "height_reference_verified", "datum_reference_verified",
]
RECEIPT_FIELDS = ["request_started_monotonic_ns", "received_monotonic_ns",
                  "received_unix_ns", "round_trip_ms"]


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def invalid_json_constant(value):
    raise ValueError(f"La respuesta JSON contiene un número no finito: {value}.")


def finite_json_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("La respuesta JSON contiene un número fuera de rango.")
    return number


class NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if fp is not None:
            fp.close()
        raise ValueError("El receptor intentó redirigir la consulta; no se seguirá otra dirección.")


class Z1Client:
    """Only the three read queries observed in the receiver's own WebUI."""
    def __init__(self, host="192.168.10.12", port=80, timeout=4):
        try:
            address = ipaddress.IPv4Address(host)
        except ipaddress.AddressValueError as error:
            raise ValueError("Usa la IPv4 local del receptor, sin URL ni nombre de dominio.") from error
        if not address.is_private or address.is_unspecified or address.is_multicast:
            raise ValueError("La dirección debe pertenecer a la red local del receptor.")
        if not 1 <= port <= 65535:
            raise ValueError("Puerto fuera de rango.")
        if not math.isfinite(timeout) or not 0.5 <= timeout <= 10:
            raise ValueError("El tiempo de espera debe estar entre 0,5 y 10 segundos.")
        self.base_url = f"http://{address}:{port}/cgi-bin/"
        self.timeout = timeout
        # Never forward a local receiver request through a configured web proxy.
        self.opener = build_opener(ProxyHandler({}), NoRedirects())

    def read(self, query):
        if query not in {"device", "status", "position"}:
            raise ValueError("Sólo se permiten consultas de lectura del receptor.")
        request = Request(self.base_url + query + ".cgi", data=b"", method="POST",
                          headers={"Accept": "application/json", "Cache-Control": "no-cache"})
        start = time.monotonic_ns()
        with self.opener.open(request, timeout=self.timeout) as response:
            payload = response.read(MAX_RESPONSE_BYTES + 1)
        received = time.monotonic_ns()
        receipt = dict(request_started_monotonic_ns=start, received_monotonic_ns=received,
                       received_unix_ns=time.time_ns(), round_trip_ms=(received - start) / 1e6)
        if len(payload) > MAX_RESPONSE_BYTES:
            raise ValueError("La respuesta excede el límite de telemetría.")
        data = json.loads(payload.decode("utf-8"), parse_constant=invalid_json_constant,
                          parse_float=finite_json_number)
        if not isinstance(data, dict):
            raise ValueError("El receptor no devolvió un objeto JSON de telemetría.")
        return data, receipt


def write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    temporary.replace(path)


def record(client, destination, *, seconds=60, interval=1):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    metadata = {
        "schema_version": 1, "started_utc": utc_now(), "state": "recording",
        "transport": "Z1 WebUI HTTP polling", "endpoint": client.base_url + "position.cgi",
        "receiver_configuration_modified": False,
        "poll_interval_s": interval, "requested_duration_s": seconds,
        "clock_alignment_verified": False, "lever_arm_verified": False,
        "georeferenced_lidar": False,
        "measurement_role": "gnss_control_observation",
        "anchor_note": "Candidate observations for GNSS map anchors; not surface points. No anchor is applied until sensor timing, mounting and coordinate references are verified.",
        "time_note": "Receiver GPS/UTC epochs and Mac reception times are distinct. HTTP arrival is not LiDAR time alignment.",
        "height_note": "Preserve receiver height and undulation. The configured geoid model and antenna reference point must be verified before surveying use.",
        "uncertainty_note": "Receiver-reported standard deviations are estimates, not an independent accuracy validation.",
        "samples": 0, "unique_epochs": 0, "duplicate_epochs": 0,
        "out_of_order_epochs": 0, "fixed_epochs": 0, "reported_fixed_epochs": 0, "errors": 0,
        "fix_type_counts": {},
    }
    manifest = destination / "session.json"
    write_json(manifest, metadata)
    print(f"ATLAS GNSS · Registro: {destination.resolve()}", flush=True)
    started = time.monotonic()
    deadline = started + seconds if seconds else math.inf
    previous_epoch = None
    stop_reason = "duration"
    try:
        for query in ("device", "status"):
            response, _ = client.read(query)
            metadata[query] = response
        if metadata["device"].get("Device") != "Z1":
            raise ValueError("El equipo no se identifica como Z1; no se supondrá compatibilidad con este protocolo.")
        print(f"Z1 · {metadata['status'].get('WorkMode', 'modo no indicado')} · "
              f"firmware {metadata['device'].get('SWV', 'desconocido')}", flush=True)
        with (destination / "raw.jsonl").open("x", encoding="utf-8") as raw, \
                (destination / "fixes.csv").open("x", newline="", encoding="utf-8") as fixes:
            writer = csv.DictWriter(fixes, fieldnames=RECEIPT_FIELDS + FIX_FIELDS)
            writer.writeheader()
            while time.monotonic() < deadline:
                poll_start = time.monotonic()
                event = {"requested_utc": utc_now()}
                try:
                    response, receipt = client.read("position")
                    event.update(receipt, response=response)
                    metadata["samples"] += 1
                    fix = parse_position(response)
                    event["parsed"] = fix
                    epoch = (fix["gps_week"], fix["gps_seconds"])
                    if previous_epoch is not None and epoch <= previous_epoch:
                        duplicate = epoch == previous_epoch
                        metadata["duplicate_epochs" if duplicate else "out_of_order_epochs"] += 1
                        event["excluded_from_csv"] = "duplicate_epoch" if duplicate else "out_of_order_epoch"
                        print("ATLAS GNSS · época repetida o anterior; se conserva sólo en el registro crudo.", flush=True)
                    else:
                        previous_epoch = epoch
                        metadata["unique_epochs"] += 1
                        reported_type = fix.get("fix_type", "unknown")
                        metadata["fix_type_counts"][reported_type] = metadata["fix_type_counts"].get(reported_type, 0) + 1
                        if fix.get("fix_type") == "rtk_fixed" and fix.get("position_valid"):
                            metadata["reported_fixed_epochs"] += 1
                            if fix.get("quality_usable"):
                                metadata["fixed_epochs"] += 1
                        writer.writerow({**receipt, **{key: fix.get(key) for key in FIX_FIELDS}})
                        fixes.flush()
                        print(f"{fix.get('utc_timestamp') or 'UTC no confirmada'} · "
                              f"{fix.get('position_type', '?')}"
                              f"{' (calidad pendiente)' if not fix.get('quality_usable') else ''} · "
                              f"satélites {fix.get('satellites_used', '?')} · "
                              f"edad corrección {fix.get('correction_age_s', '?')} s · "
                              f"consulta {receipt['round_trip_ms']:.0f} ms", flush=True)
                except (ValueError, OSError, HTTPError, URLError) as error:
                    metadata["errors"] += 1
                    event["error"] = str(error)
                    print(f"ATLAS GNSS · lectura inválida: {error}", file=sys.stderr, flush=True)
                raw.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
                raw.flush()
                write_json(manifest, metadata)
                delay = min(deadline - time.monotonic(), interval - (time.monotonic() - poll_start))
                if delay > 0:
                    time.sleep(delay)
    except KeyboardInterrupt:
        stop_reason = "user_interrupt"
    except Exception as error:
        metadata["fatal_error"] = str(error)
        stop_reason = "error"
        print(f"ATLAS GNSS · {error}", file=sys.stderr, flush=True)
    finally:
        metadata.update(finished_utc=utc_now(), elapsed_s=time.monotonic() - started,
                        stop_reason=stop_reason,
                        state="failed" if stop_reason == "error" or not metadata["unique_epochs"]
                              else "partial" if metadata["errors"] else "recorded")
        write_json(manifest, metadata)
    print(f"ATLAS GNSS · {metadata['unique_epochs']} épocas distintas; "
          f"{metadata['fixed_epochs']} FIX que pasan los controles de telemetría; {metadata['errors']} lecturas inválidas. "
          "Datos guardados. Sincronización con LiDAR pendiente.", flush=True)
    return 0 if metadata["state"] == "recorded" else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.10.12", help="IPv4 local del Z1.")
    parser.add_argument("--port", type=int, default=80)
    parser.add_argument("--seconds", type=float, default=60,
                        help="Duración en segundos; 0 continúa hasta Ctrl+C (por defecto 60).")
    parser.add_argument("--interval", type=float, default=1, help="Intervalo mínimo entre consultas; al menos 1 s.")
    parser.add_argument("--timeout", type=float, default=4)
    parser.add_argument("--output", type=Path, help="Carpeta nueva; por defecto output/gnss/fecha-única.")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 <= args.seconds <= 86400:
        parser.error("La duración debe estar entre 0 y 86400 segundos.")
    if not math.isfinite(args.interval) or not 1 <= args.interval <= 60:
        parser.error("El intervalo debe estar entre 1 y 60 segundos.")
    try:
        client = Z1Client(args.host, args.port, args.timeout)
        if args.output is None:
            parent = ROOT / "output" / "gnss"
            parent.mkdir(parents=True, exist_ok=True)
            # Reserve a unique name, then let record exclusively create it.
            destination = Path(tempfile.mkdtemp(prefix=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-"), dir=parent))
            destination.rmdir()
        else:
            destination = args.output
        return record(client, destination, seconds=args.seconds, interval=args.interval)
    except (ValueError, OSError) as error:
        print(f"ATLAS GNSS · {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
