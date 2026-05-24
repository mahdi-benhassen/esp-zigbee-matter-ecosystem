# CI/CD Pipeline & Build Matrix Guide

The project utilizes GitHub Actions to automate code validation and firmware compilation on every change. The workflow configurations are located in [.github/workflows/build-and-release.yml](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/.github/workflows/build-and-release.yml).

---

## Build Automation Pipeline

### Environment
The build job runs in a virtual Ubuntu environment using the official Espressif Docker container:
- **Image**: `espressif/idf:v5.5.4`
- **Compiler**: Xtensa Toolchain (GCC 14.2.0 for ESP-IDF v5.5)

### Steps Executed
1. **Repository Checkout**: Checks out the code and pulls git submodules recursively.
2. **Build End Node**: Sets target `esp32h2`, runs `idf.py build`, and copies output binaries (`bootloader.bin`, `partition-table.bin`, `end_node.bin`) to an artifact folder.
3. **Build Gateway**: Sets target `esp32s3`, runs `idf.py build`, and copies output binaries (`bootloader.bin`, `partition-table.bin`, `gateway.bin`) to the artifact folder.
4. **Publish Pipeline Artifacts**: Uploads the compiled binaries as raw pipeline run attachments (`firmware-binaries`) for manual verification.

---

## Automatic Releases on Git Tags

When you push a Git tag starting with `v` (e.g. `v1.1.0`), a specialized release job is triggered:
- **Asset Packaging**: Compresses the `end_node` and `gateway` firmware folders into separate ZIP files (`end_node_firmware.zip` and `gateway_firmware.zip`).
- **Release Creation**: Automatically drafts and publishes a GitHub Release corresponding to the tagged version.
- **Artifact Upload**: Attaches both the ZIP archives and individual binary files to the release assets for direct OTA (Over-The-Air) deployment or flashing.
