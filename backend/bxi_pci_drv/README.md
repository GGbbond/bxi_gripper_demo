# BXI PCI user-space dependency

This directory contains the two files required to build the standalone gripper backend:

- `bxi_pci_drv.h` — public user-space API declarations
- `libbxi_pci_drv.a` — prebuilt x86_64 static library

The files were copied unchanged from `bxi_motor_test_bench/bxi_pci_drv/lib` so this repository can build without referring to the original test-bench directory.

SHA-256 checksums at the time of vendoring:

```text
ba19a5dca54c84748777f70a813c92499e2c214da3da9a1835151c83caee5f2f  bxi_pci_drv.h
2f8ec45cd98fa28d3313fc311ae408ec691eeeb799d3224bc8b4f6ee635abf64  libbxi_pci_drv.a
```

The original dependency directory did not contain a license file. Confirm redistribution rights before publishing this archive in a public repository.

