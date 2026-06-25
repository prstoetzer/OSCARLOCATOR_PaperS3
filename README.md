# OSCARLOCATOR for PaperS3 🛰️

A **live**, on-device OSCARLOCATOR azimuthal-equidistant satellite display for
the **M5Stack M5Paper S3** (ESP32-S3, 960×540 e-ink, GT911 touch).

This is a port of the web-based **OSCARLOCATOR Simulator**
([oscarlocator.n8hm.radio](https://oscarlocator.n8hm.radio)), itself modeled on
**OrbitDeck**. It renders the classic OSCARLOCATOR geometry — a polar or
QTH-centred azimuthal-equidistant map with the satellite's ground-track arc, a
range circle over your station, the satellite footprint at its sub-point, and
the live sub-satellite position — directly on the e-ink panel, plus two
companion tables (next passes and reference orbits) drawn straight from the same
logic as the website.

**Credit: Paul Stoetzer, N8HM** — author of OrbitDeck and the OSCARLOCATOR
Simulator. This M5Paper S3 port carries that work to a standalone e-ink device.

---

## What it does

- **Captive web configuration page** for satellite, QTH (Maidenhead grid or
  lat/lon), min pass elevation, reference-orbit day count, and default view.
  WiFi credentials are handled by a WiFiManager captive portal.
- **NTP time** on boot (`pool.ntp.org` / `time.nist.gov`).
- **On-device AMSAT GP element processing.** Downloads the AMSAT GP
  (General Perturbations) data set —
  `daily-bulletin.json` from `newark192.amsat.org/gpdata/current/` — caches it
  in LittleFS, parses the OMM/GP JSON on the device, synthesises SGP4 elements
  from the GP fields, and propagates. No phone or laptop needed after first
  setup.
- **Smooth e-ink fonts and graphics.** Renders to an off-screen 4-bit
  grayscale canvas. Screen changes use the crisp full `epd_quality` waveform;
  per-tick live updates use fast partial refreshes of just the changed regions
  (see "E-ink refresh" below). Anti-aliased FreeSans type, clean vector arcs.
- **Map outlines.** World coastlines are baked into `coastline.h` (a compact
  int16 table in flash, derived from the same outline data the website ships)
  and drawn under the graticule in every projection.

### Three screens

1. **Map** — the live OSCARLOCATOR. Ground-track arc, range circle over the
   QTH, satellite footprint, live sub-satellite dot, and a readout
   (sub-point, altitude, Az/El, range, visible).
2. **Next passes** — the next ten times the satellite rises above your
   horizon: AOS, peak, LOS, duration, and maximum elevation. Computed the same
   way as
   the website's pass list (SGP4 `nextpass` + backward AOS refinement).
3. **Reference orbits** — the first equator crossing of each UTC day: the time
   and longitude you would dial into a real OSCARLOCATOR. Northern-hemisphere
   stations use the **ascending** node; southern stations use the
   **descending** node — matching the website.

### On-device interaction (intentionally minimal)

This is a **live display only**. The web simulator's manual manipulation modes
(drag the disc, pin an EQX longitude, step minutes-after-crossing) are **not**
ported. On the device you can only:

- **Choose the view:** Polar — auto N/S, Polar — North, Polar — South, or
  QTH-centred (azimuthal). Tap **View** on the map screen to cycle.
- **Choose the satellite:** tap **Satellite** on a list screen to open the
  paged picker.

Everything else (QTH, WiFi, default view, etc.) is set from the web page.

---

## Hardware

- **M5Paper S3** (M5Stack PaperS3, ESP32-S3R8, 8 MB PSRAM, 16 MB flash)
- 960×540 4.7" e-ink, 16-level grayscale, GT911 capacitive touch
- WiFi for the initial GP download and occasional refreshes

> ⚠️ Per M5Stack's notice, avoid QC3.0/2.0 chargers with the PaperS3.

---

## Libraries & board support

Install via the Arduino IDE **Library Manager** / **Boards Manager**.

**Board package** — add this Boards Manager URL, then install **M5Stack** and
select board **M5PaperS3**:

```
https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
```

Set **PSRAM: OPI / Octal** in the board options.

| Library | Purpose | Notes |
| --- | --- | --- |
| **M5Unified** (≥ 0.2.5) | M5Paper S3 core + e-ink + touch | pulls in M5GFX |
| **SparkFun SGP4 Arduino Library** | SGP4 propagation + pass prediction | the Hopperpop port: `findsat`, `nextpass`, `initpredpoint`, `satAz`/`satEl`/`satLat`/`satLon` |
| **WiFiManager** (tzapu) | captive portal for first-time WiFi | |
| **ArduinoJson** (≥ 6.x) | parse the AMSAT GP JSON | v7 elastic `JsonDocument` recommended |

> **M5PaperS3 display note.** M5GFX drives the PaperS3 panel through the
> **EPDiy** library, which (as of this writing) isn't in the Library Manager —
> install it from `github.com/vroland/epdiy`. Build with **PSRAM enabled, set to
> OPI / Octal**. See `M5GFX/docs/M5PaperS3.md`.

> Use the **SparkFun/Hopperpop** SGP4 library specifically — other Sgp4 ports
> have different method signatures and will not compile.

---

## Build & upload

1. Install Arduino IDE 2.x.
2. Add the M5Stack board URL and install the board package; select
   **M5PaperS3**.
3. Install the three libraries above.
4. Open `OSCARLOCATOR_PaperS3.ino`.
5. Select the correct USB serial port. If upload fails, long-press the power
   button until the rear LED blinks red (download mode).
6. Upload.

## First-time setup

1. On first boot the device opens a WiFi captive portal
   (SSID **`OSCARLOCATOR-Setup`**). Join it and enter your home WiFi.
2. After it connects and syncs NTP, it fetches the AMSAT GP JSON, synthesises
   SGP4 elements, and starts tracking the default satellite.
3. Browse to the device's IP (shown in the config page) and open the
   **configuration page** to set your satellite, QTH grid/lat-lon, and default
   view. You can also reach **Fetch AMSAT GP elements now** and **Reconfigure
   WiFi** from there.
4. On the device, use **View** and **Satellite** to drive the live display.

Subsequent boots use the cached GP set from LittleFS and only refresh over WiFi
when the data is older than ~24 hours. The display works fully offline after the
first successful download.

---

## How it works (brief)

- **Projection.** Ported 1:1 from the website. Polar views centre on a pole
  (`rho = 90 ∓ lat`, `t = lon`); the QTH view centres on the station
  (`rho`, `t` from central angle + bearing). The map edge is 90° for polar and
  `clamp(|qlat|+25, 50, 80)` for the QTH view, and the screen-axis orientation
  matches each mode (North: `+sin,+cos`; South: `−sin,+cos`; QTH: `+sin,−cos`).
- **Coastlines.** Drawn from `coastline.h`, projected the same way as the track.
- **E-ink refresh.** The live tick runs every 20 s when the satellite is up and
  60 s otherwise. Full-quality (`epd_quality`) clears are reserved for screen
  changes, satellite/view changes, table recomputes, and a periodic de-ghost
  (every 30 partials). In between, only the changed regions are refreshed with
  the fast waveform (`epd_fast`): on the map that's the header clock, the readout
  panel, and a small box covering the satellite dot's previous and current
  positions, so the static base map and coastlines are never re-flashed; on the
  list screens it's just the header strip. This keeps the always-on display from
  flashing the whole panel every tick and greatly reduces power and panel wear.
- **Ground-track arc.** The sub-satellite track of roughly one orbit, sampled
  with `findsat` and centred on the current time so the live dot sits on its
  arc. Period comes from the propagator's `revpday`.
- **Footprint / range circle.** The amber range circle is the footprint at the
  satellite's *mean* altitude centred on the QTH (inflated 6.5% on polar maps,
  as the site does); the green dashed footprint is at the *instantaneous*
  altitude about the sub-point. Both are projected geodesic small-circles.
- **GP data.** The device reads AMSAT's GP (General Perturbations) JSON
  (`daily-bulletin.json`), an array of OMM-style records. Because the SGP4
  library is initialised from TLE strings, the firmware synthesises
  column-exact TLE line1/line2 from the GP fields on the device (epoch
  `YYDDD.DDDDDDDD`, assumed-decimal exponent encoding for BSTAR/ndot, mod-10
  checksum) and feeds them to the verified propagator.
- **Passes.** `initpredpoint` + `nextpass` (Hopperpop) give peak/LOS/max-el;
  AOS is refined by walking backward from the peak (robust on eccentric orbits).
- **Reference orbits.** For each UTC day, the first ascending (N) or descending
  (S) node is found by a 60-second scan and 30-iteration bisection, reported in
  West-positive longitude as OSCARLOCATOR boards use.

---

## Credits

- **Paul Stoetzer, N8HM** — OrbitDeck and the OSCARLOCATOR Simulator
  ([oscarlocator.n8hm.radio](https://oscarlocator.n8hm.radio),
  [github.com/prstoetzer/OrbitDeck](https://github.com/prstoetzer/OrbitDeck)).
- **Hopperpop / SparkFun** — SGP4 Arduino library.
- **tzapu** — WiFiManager.
- **M5Stack** — M5Paper S3 hardware and M5Unified / M5GFX.
- **AMSAT** — the GP (`daily-bulletin.json`) distribution. If you find this
  useful, please consider supporting **AMSAT** at
  [www.amsat.org](https://www.amsat.org).
- Dr. T.S. Kelso (CelesTrak) and David Vallado for the underlying algorithms.

## License

MIT — see [LICENSE](LICENSE).
