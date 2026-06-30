"""Generate SHA-256 hashes and RFC 3161 timestamps for Patent Search evidence files."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import Request, urlopen

OPENSSL = Path(r"C:\Program Files\Git\usr\bin\openssl.exe")
TSA_URL = "https://freetsa.org/tsr"
CA_URL = "https://freetsa.org/files/cacert.pem"
TSA_CERT_URL = "https://freetsa.org/files/tsa.crt"
EVIDENCE_EXTENSIONS = {".pdf", ".png"}


def run_openssl(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(OPENSSL), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_name(name: str) -> str:
    stem = Path(name).stem
    return re.sub(r"[^\w\-]", "_", stem)


def download(url: str, dest: Path) -> None:
    if dest.exists():
        return
    with urlopen(url, timeout=60) as response:
        dest.write_bytes(response.read())


def timestamp_file(source: Path, evidence_dir: Path, ca_file: Path, tsa_cert: Path) -> dict:
    base = safe_name(source.name)
    hash_path = evidence_dir / f"{base}.sha256"
    tsq_path = evidence_dir / f"{base}.tsq"
    tsr_path = evidence_dir / f"{base}.tsr"

    digest = sha256_file(source)
    hash_path.write_text(f"{digest}  {source.name}", encoding="utf-8")

    query = run_openssl(
        ["ts", "-query", "-data", str(source), "-no_nonce", "-sha256", "-cert", "-out", str(tsq_path)]
    )
    if query.returncode != 0:
        raise RuntimeError(f"ts -query failed for {source.name}: {query.stderr}")

    request = Request(
        TSA_URL,
        data=tsq_path.read_bytes(),
        headers={"Content-Type": "application/timestamp-query"},
        method="POST",
    )
    with urlopen(request, timeout=60) as response:
        tsr_path.write_bytes(response.read())

    verify = run_openssl(
        [
            "ts",
            "-verify",
            "-in",
            str(tsr_path),
            "-queryfile",
            str(tsq_path),
            "-CAfile",
            str(ca_file),
            "-untrusted",
            str(tsa_cert),
        ]
    )
    if verify.returncode != 0:
        raise RuntimeError(f"ts -verify failed for {source.name}: {verify.stderr}")

    reply = run_openssl(["ts", "-reply", "-in", str(tsr_path), "-text"])
    if reply.returncode != 0:
        raise RuntimeError(f"ts -reply failed for {source.name}: {reply.stderr}")

    match = re.search(r"Time stamp:\s*(.+?)\s*GMT", reply.stdout)
    timestamp_utc = f"{match.group(1).strip()} GMT" if match else "unknown"

    return {
        "source_file": source.name,
        "sha256": digest,
        "rfc3161_timestamp_utc": timestamp_utc,
        "timestamp_request": tsq_path.name,
        "timestamp_response": tsr_path.name,
        "hash_file": hash_path.name,
        "verified": True,
    }


def load_existing_manifest(manifest_path: Path) -> list[dict]:
    if not manifest_path.exists():
        return []
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    return data.get("files", [])


def write_manifest(manifest_path: Path, manifest: list[dict]) -> None:
    generated_at = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S %Z")
    manifest_path.write_text(
        json.dumps(
            {
                "generated_at_utc": generated_at,
                "tsa": "freetsa.org",
                "protocol": "RFC 3161",
                "hash_algorithm": "SHA-256",
                "files": manifest,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    lines = [
        "Patent Search Evidence Manifest",
        f"Generated (UTC): {generated_at}",
        "TSA: freetsa.org (RFC 3161)",
        "Hash algorithm: SHA-256",
        "",
    ]
    for entry in manifest:
        lines.extend(
            [
                f"File: {entry['source_file']}",
                f"SHA-256: {entry['sha256']}",
                f"RFC 3161 timestamp (UTC): {entry['rfc3161_timestamp_utc']}",
                f"Timestamp response: evidence\\{entry['timestamp_response']}",
                "",
            ]
        )
    (manifest_path.parent / "evidence_manifest.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    if not OPENSSL.is_file():
        print(f"OpenSSL not found at {OPENSSL}", file=sys.stderr)
        return 1

    folder = Path(__file__).resolve().parent.parent
    evidence_dir = folder / "evidence"
    evidence_dir.mkdir(exist_ok=True)
    manifest_path = evidence_dir / "evidence_manifest.json"

    ca_file = evidence_dir / "freetsa-cacert.pem"
    tsa_cert = evidence_dir / "freetsa-tsa.crt"
    download(CA_URL, ca_file)
    download(TSA_CERT_URL, tsa_cert)

    existing = load_existing_manifest(manifest_path)
    indexed = {entry["source_file"]: entry for entry in existing}

    sources = sorted(
        path
        for path in folder.iterdir()
        if path.is_file() and path.suffix.lower() in EVIDENCE_EXTENSIONS and path.name not in indexed
    )
    if not sources:
        print("No new evidence files to timestamp.")
        return 0

    new_entries = [timestamp_file(source, evidence_dir, ca_file, tsa_cert) for source in sources]
    for entry in new_entries:
        indexed[entry["source_file"]] = entry

    manifest = sorted(indexed.values(), key=lambda entry: entry["source_file"].lower())

    write_manifest(manifest_path, manifest)

    for entry in new_entries:
        print(f"OK: {entry['source_file']} -> {entry['sha256']}")
    print(f"\nManifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
