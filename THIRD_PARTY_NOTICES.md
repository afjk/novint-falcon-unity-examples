# Third-Party Notices

This project is released under the MIT License (see `LICENSE`). The project also makes
use of the following third-party components. Their license terms are summarized
below and the full license texts are included in the referenced locations within
this repository or upstream packages. When redistributing this project, ensure
these notices are preserved and that any additional obligations (for example,
documentation acknowledgements) are fulfilled.

## Native Dependencies

### libnifalcon
- **Description:** Device driver/library used by the native FalconBridge plugin to
  communicate with Novint Falcon hardware.
- **Upstream:** https://github.com/libnifalcon/libnifalcon
- **License:** BSD 3-Clause License (`libnifalcon/license/LICENSE_LIBNIF_BSD.txt`)
- **Notices:**
  - Redistribution must retain the original copyright notice, conditions, and
    disclaimer.
  - Names of libnifalcon contributors may not be used to endorse derived products
    without prior permission.
  - libnifalcon bundles additional dependencies with their own licenses; see below.

### Generic Math Template Library (GMTL)
- **Description:** Header-only math library bundled with libnifalcon.
- **License:** GNU Lesser General Public License v2.1 (or later) with explicit
  static-linking addendum (`libnifalcon/license/LICENSE_GMTL.txt` and
  `libnifalcon/license/LICENSE_GMTL_ADDENDUM.txt`).
- **Notices:**
  - The addendum permits static linking without imposing additional copyleft
    requirements, but still requires acknowledgement.
  - Applications must include the statement: “This software uses GMTL
    (http://ggt.sourceforge.net).”

### Novint SDK Materials
- **Description:** Firmware blobs and related utilities distributed with libnifalcon.
- **License:** Novint Noncommercial SDK License (`libnifalcon/license/LICENSE_NOVINT.txt`).
- **Notices:**
  - The Novint SDK is licensed for non-commercial use only. Ensure your usage and
    redistribution comply with Novint’s terms.

## Unity Packages & Tools

### Unity Editor and Built-in Packages
- **Provider:** Unity Technologies
- **License:** Unity Editor and official Unity packages are provided under the
  Unity Software License / Unity Companion License as applicable. See
  https://unity.com/legal for details. Redistributing Unity binaries is subject to
  Unity’s license terms.

### MCP for Unity (`com.coplaydev.unity-mcp`)
- **Version:** 6.3.0 (via OpenUPM)
- **Upstream:** https://github.com/CoplayDev/unity-mcp
- **License:** MIT License (see upstream LICENSE)
- **Notes:** Package includes the MCP for Unity Editor tooling used to automate
  bridge setup. When redistributing, retain the upstream license information.

### Newtonsoft.Json (`com.unity.nuget.newtonsoft-json` dependency)
- **License:** MIT License (copyright © James Newton-King). Unity ships the
  package via its NuGet bridge. Full text available at
  https://licenses.nuget.org/MIT.

## Usage Guidance

- Keep this notice file together with any redistributed binaries/source.
- Provide end users with access to the license texts linked above.
- Honour attribution requirements (e.g., GMTL acknowledgement statement) in any
  published documentation or in-app “About” dialogs.

For questions about these notices or to report missing information, please open
an issue in the project repository.
