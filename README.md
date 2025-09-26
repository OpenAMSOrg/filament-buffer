# OpenAMS filament pressure sensor (FPS), rear umbilical, and filament path jointer

![v4.0](/images/v4.0_fps.png)

This repository contains the CAD and STL files for the OpenAMS FPS (filament pressure sensor). Use these files to adapt an existing AMS/Bambulab filament buffer slide to the OpenAMS system, or to print your own slide.

## Bill of materials (BOM)

If you already have an original Bambulab filament buffer assembly (it contains the spring, magnet, and two ABS shells), you only need the following to integrate it with OpenAMS:

| Part | Quantity |
|------:|:--------:|
| M2 x 16 mm screws | 2 |
| M3 heat-set insert (4 mm tall) | 1 |
| Original Bambulab slide assembly | 1 |
| FPS board ([si-forge.com](https://si-forge.com)) | 1 |
| FPS case STL | 1 |
| FPS bambulab cover STL | 1 |

If you plan to print the slide and parts yourself, use this BOM as a baseline:

| Part | Quantity |
|------:|:--------:|
| M2 x 16 mm screws | 2 |
| M3 heat-set insert (4 mm tall) | 1 |
| ECAS 04 insert | 2 |
| 5 mm diameter × 15 mm long magnet* | 1 |
| FPS board ([si-forge.com](https://si-forge.com)) | 1 |
| FPS case STL | 1 |
| FPS DIY cover STL | 1 |

*Magnet note: if a single 5×15 mm magnet is hard to source, stacking five 5×3 mm magnets end-to-end is an acceptable alternative. Ensure the stacked magnets are secured in the pocket (CA glue or a light press fit) so they don't shift during filament movement.

## Other recommended parts

- PTFE tubing: 3 mm ID / 4 mm OD (standard for many MMU configurations to avoid friction)
- Optional mounting hardware or printed brackets depending on your printer and chosen mounting location

## Images and CAD

Please check the CAD files and the example images for suggested filament paths and mounting locations.


![v4.0](/images/v4_0.png)

## Files provided

- v1.x – v3.x: STL/CAD files compatible with the FPS boards labeled with matching version numbers. These releases assume you are using the original AMS/Bambulab slide or the FPS board version listed on the board silk-screen (see "Versioning" below).
- v4.x (beta): a self-sourced variant intended for users who want to fully print and source their own slide components (in other words, for installations where the original AMS/Bambulab slide is not available). v4.x is feature-equivalent but expects the builder to supply inserts, magnets, and fasteners.

## Versioning

The repository and the FPS hardware use a simple version mapping to help you pick the correct STL/CAD files:

- Board silk-screen versions 1.x through 3.x correspond to the sets in the `v1.x`, `v2.x`, and `v3.x` folders respectively. If your FPS board has a silk-printed version number (for example, "v2.0" or "v3.0"), use the matching folder for mechanical parts and the recommended assembly for that board revision.
- Version 4.x is marked beta and is intended for users who will self-source the slide components (magnets, inserts, springs, etc.) or who do not have the original AMS/Bambulab filament buffer that came with AMS1. Expect minor fitment or assembly adjustments when using `v4.x` files; please report back any improvements.

## Printing and assembly guidance

- Material:  ABS is recommended for parts near heat or for added durability; PLA is usable for prototypes but may soften in warm environments. Do NOT print the FPS in anything rough, such as any filament containing glass or carbon fibers.

- Print orientation: print the slide and case so the magnet pocket and heat-set insert holes are printed with clean walls (minimal bridging). Use 100% infill or a high infill around the magnet pocket if you need extra strength.

- Tolerances: the magnet pocket is designed for a press-fit magnet; if your magnets are slightly oversized, sand the pocket carefully. For heat-set inserts, a snug hole size (per insert manufacturer's recommendation) and a controlled installation temperature will give the best result.
- Securing magnets: use a small dab superglue if you’re stacking smaller magnets.

## Installation notes

- The FPS case can be installed anywhere along the filament path, but the provided rear-umbilical and adapter plate are designed to integrate into the rear of Voron printers.

## Contributing

If you find fitment issues, typos, or improvements (for example better magnet pocket dimensions or heat-set insert recommendations), please open an issue or submit a pull request. v4.x is beta — community feedback is appreciated.

## License and credits

See the top-level `LICENSE` file for license details.