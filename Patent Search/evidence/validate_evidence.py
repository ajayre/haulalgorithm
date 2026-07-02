"""Validate SHA-256 hashes and RFC 3161 timestamps for Patent Search evidence."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

OPENSSL = Path(r"C:\Program Files\Git\usr\bin\openssl.exe")
EVIDENCE_EXTENSIONS = {".pdf", ".png"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_timestamp(tsr: Path, tsq: Path, ca_file: Path, tsa_cert: Path) -> bool:
    result = subprocess.run(
        [
            str(OPENSSL),
            "ts",
            "-verify",
            "-in",
            str(tsr),
            "-queryfile",
            str(tsq),
            "-CAfile",
            str(ca_file),
            "-untrusted",
            str(tsa_cert),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.returncode == 0


def main() -> int:
    if not OPENSSL.is_file():
        print(f"ERROR: OpenSSL not found at {OPENSSL}", file=sys.stderr)
        return 1

    folder = Path(__file__).resolve().parent.parent
    evidence_dir = folder / "evidence"
    manifest_path = evidence_dir / "evidence_manifest.json"
    ca_file = evidence_dir / "freetsa-cacert.pem"
    tsa_cert = evidence_dir / "freetsa-tsa.crt"

    for required in (manifest_path, ca_file, tsa_cert):
        if not required.exists():
            print(f"ERROR: Required file not found: {required}", file=sys.stderr)
            return 1

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = manifest.get("files", [])

    print()
    print("Patent Search Evidence Validation")
    print("=================================")
    print()

    passed = 0
    failed = 0

    for entry in sorted(entries, key=lambda item: item["source_file"].lower()):
        source_name = entry["source_file"]
        expected_hash = entry["sha256"]
        source_path = folder / source_name
        tsq_path = evidence_dir / entry["timestamp_request"]
        tsr_path = evidence_dir / entry["timestamp_response"]
        hash_path = evidence_dir / entry["hash_file"]

        if not source_path.is_file():
            print(f"[FAIL] {source_name} - source file not found")
            failed += 1
            continue

        missing = [path.name for path in (hash_path, tsq_path, tsr_path) if not path.is_file()]
        if missing:
            print(f"[FAIL] {source_name} - missing evidence artifact(s): {', '.join(missing)}")
            failed += 1
            continue

        actual_hash = sha256_file(source_path)
        if actual_hash.lower() != expected_hash.lower():
            print(f"[FAIL] {source_name} - SHA-256 mismatch")
            print(f"         expected: {expected_hash}")
            print(f"         actual:   {actual_hash}")
            failed += 1
            continue

        if not verify_timestamp(tsr_path, tsq_path, ca_file, tsa_cert):
            print(f"[FAIL] {source_name} - RFC 3161 timestamp verification failed")
            failed += 1
            continue

        print(f"[OK]   {source_name}")
        passed += 1

    manifest_names = {entry["source_file"] for entry in entries}
    source_files = sorted(
        path.name
        for path in folder.iterdir()
        if path.is_file() and path.suffix.lower() in EVIDENCE_EXTENSIONS
    )
    missing_evidence = [name for name in source_files if name not in manifest_names]

    if missing_evidence:
        print()
        print("Untimestamped evidence files:")
        for name in missing_evidence:
            print(f"[WARN] {name} - no manifest entry (run timestamp_evidence.py)")
        failed += len(missing_evidence)

    print()
    print("---------------------------------")
    print(f"Passed: {passed}   Failed: {failed}")
    if missing_evidence:
        print(f"Untimestamped: {len(missing_evidence)}")
    print("---------------------------------")
    print()

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
