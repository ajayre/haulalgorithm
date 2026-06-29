# Patent Search Evidence Preservation

This folder contains patent-related documents preserved as cryptographic evidence. Each source file has a **SHA-256 hash** and an **RFC 3161 trusted timestamp** proving the file existed in its current form at a specific date and time.

## What was done

For every file in this folder (excluding subdirectories), the following steps were performed:

1. **Compute SHA-256 hash locally** — The file is read on this machine and hashed. The original file is never uploaded.
2. **Create an RFC 3161 timestamp request** (`.tsq`) — OpenSSL builds a Time-Stamp Protocol request containing the SHA-256 digest of the file.
3. **Submit the request to a Time Stamping Authority (TSA)** — Only the hash request is sent to [freetsa.org](https://freetsa.org/tsr), not the document itself.
4. **Save the signed timestamp response** (`.tsr`) — The TSA returns a cryptographically signed token attesting that the hash existed at the stated UTC time.
5. **Verify the response** — Each `.tsr` file is checked immediately against the TSA's certificate chain before being recorded in the manifest.

All artifacts are stored in the `evidence/` subdirectory.

## Folder layout

```
Patent Search/
├── README.md                          ← this file
├── <source documents>                 ← original evidence files
└── evidence/
    ├── evidence_manifest.json         ← machine-readable index
    ├── evidence_manifest.txt          ← human-readable index
    ├── freetsa-cacert.pem             ← TSA root CA certificate
    ├── freetsa-tsa.crt                ← TSA signing certificate
    ├── timestamp_evidence.py          ← script to re-run the process
    └── <basename>.*
        ├── .sha256                    ← SHA-256 hash of the source file
        ├── .tsq                       ← timestamp request sent to TSA
        └── .tsr                       ← signed timestamp response from TSA
```

Evidence file basenames are derived from the source filename with spaces and special characters replaced by underscores (e.g. `35 USC 292 False Marking.pdf` → `35_USC_292_False_Marking.*`).

## Prerequisites

Validation and re-running the process require **OpenSSL**. On this machine it is available via Git for Windows:

```
C:\Program Files\Git\usr\bin\openssl.exe
```

Python 3 is required only to re-run `evidence/timestamp_evidence.py`.

## How to validate

Validation has two parts: confirm the file has not changed, and confirm the timestamp is authentic.

### 1. Check the file hash

Recompute the SHA-256 of a source file and compare it to the value in `evidence/evidence_manifest.txt` or the corresponding `.sha256` file.

**PowerShell (OpenSSL):**

```powershell
$openssl = "C:\Program Files\Git\usr\bin\openssl.exe"
& $openssl dgst -sha256 "35 USC 292 False Marking.pdf"
```

**PowerShell (built-in):**

```powershell
Get-FileHash "35 USC 292 False Marking.pdf" -Algorithm SHA256
```

The hash must match exactly. Any difference means the file has been modified since it was timestamped.

### 2. Verify the RFC 3161 timestamp

This confirms the `.tsr` response is a valid signature from the TSA over the hash recorded in the `.tsq` request.

```powershell
$openssl = "C:\Program Files\Git\usr\bin\openssl.exe"
$evidence = "evidence"

& $openssl ts -verify `
    -in "$evidence\35_USC_292_False_Marking.tsr" `
    -queryfile "$evidence\35_USC_292_False_Marking.tsq" `
    -CAfile "$evidence\freetsa-cacert.pem" `
    -untrusted "$evidence\freetsa-tsa.crt"
```

A successful verification prints:

```
Verification: OK
```

### 3. Inspect timestamp details

To view the UTC time and embedded hash from a timestamp response:

```powershell
& $openssl ts -reply -in "evidence\35_USC_292_False_Marking.tsr" -text
```

Look for `Status: Granted` and the `Time stamp:` line. The `Message data:` block is the SHA-256 digest that was timestamped; it must correspond to the hash of the source file.

## Re-running for new files

After adding documents to this folder, run:

```powershell
python evidence\timestamp_evidence.py
```

The script processes every file directly in `Patent Search/` (not files inside `evidence/`) and regenerates the manifests. Existing evidence files for unchanged filenames will be overwritten with new timestamps.

## What to keep together

For evidence to remain verifiable in the future, preserve all of the following as a set:

- The original source files in this folder
- The matching `.sha256`, `.tsq`, and `.tsr` files in `evidence/`
- `freetsa-cacert.pem` and `freetsa-tsa.crt` (needed to verify the TSA signature offline)
- `evidence_manifest.json` or `evidence_manifest.txt`

## References

- [RFC 3161 — Internet X.509 Public Key Infrastructure Time-Stamp Protocol](https://www.rfc-editor.org/rfc/rfc3161)
- [Free TSA (freetsa.org)](https://freetsa.org/index_en.php) — the Time Stamping Authority used for these timestamps
