#!/usr/bin/env python3
"""
Cargo runner that converts the produced ELF into a UF2 file using elf2flash.
Avoids PowerShell/batch so it works across platforms with Python available.
"""

import os
import pathlib
import subprocess
import sys


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("elf2flash_convert.py: missing ELF path argument from Cargo runner.", file=sys.stderr)
        return 1

    elf_arg = pathlib.Path(argv[1]).resolve()
    if not elf_arg.exists():
        print(f"elf2flash_convert.py: ELF file '{elf_arg}' not found.", file=sys.stderr)
        return 1

    if elf_arg.suffix == ".elf":
        uf2_path = elf_arg.with_suffix(".uf2")
    else:
        uf2_path = elf_arg.with_suffix(elf_arg.suffix + ".uf2")

    board = os.environ.get("ELF2FLASH_BOARD", "rp2040")

    print("Converting ELF to UF2...")
    print(f"  ELF: {elf_arg}")
    print(f"  UF2: {uf2_path}")
    print(f"  Board: {board}")

    cmd = [
        "elf2flash",
        "convert",
        "--board",
        board,
        str(elf_arg),
        str(uf2_path),
    ]

    try:
        result = subprocess.run(cmd, check=False)
    except OSError as exc:
        print(f"elf2flash_convert.py: failed to execute elf2flash: {exc}", file=sys.stderr)
        return 1

    if result.returncode != 0:
        print(f"elf2flash convert failed with exit code {result.returncode}.", file=sys.stderr)
        return result.returncode

    print(f"UF2 image generated at {uf2_path}.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
