# Guide on minimal reproducers

A Minimal Reproducer is a small, self-contained, and buildable code example that demonstrates a specific bug or issue you've encountered while using esp-hal (and related crates in this repository).

Providing a good reproducer is essential for the maintainers to quickly understand, verify, and fix your issue.

## Key Requirements for a Minimal Reproducer

To be effective, your reproducer should adhere to the following principles:

- Self-Contained and Buildable: The code must be a complete project (use `esp-generate` to create the skeleton) that can be built and run without any modifications by the person trying to reproduce the issue.
- Minimal: It should contain only the code strictly necessary to demonstrate the issue. Remove any unrelated features, dependencies, or verbose logging that doesn't contribute to the problem.
- No Unrelated Dependencies: Avoid including external crates or libraries unless they are directly necessary for the specific feature you are reporting a bug on.
- Avoid referencing a crate modified by you - even if it's just added logging. Ideally reference the latest released version of a crate _or_ use a git-reference pointing to a specific (latest as of the time of writing) commit
- External Hardware and Wiring: If the issue requires external components (e.g., an LED, an I2C sensor), explicitly state the required hardware and provide a clear, simple description or diagram of the wiring.

## Special Considerations for Connectivity Issues

If your issue involves Wi-Fi, Bluetooth, or other networking components:

- Avoid API Keys/Registration: The code must not access web APIs that require an API key, registration, or a paid subscription. Use public, open, and readily available endpoints (e.g., public time servers, simple HTTP test sites) or focus on local network interaction if possible. This ensures anyone can test the code immediately.
- Provide Environment Details: Since connectivity issues are often environment-specific, you must include:
    - The model of the access point (AP) or router you are using (e.g., "TP-Link AX1500", "FritzBox 7590").
    - Any relevant configuration details (e.g., "WPA2 Personal", "using 2.4GHz band").
