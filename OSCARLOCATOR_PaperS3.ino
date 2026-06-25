/*
 * ============================================================================
 *  OSCARLOCATOR for PaperS3
 * ============================================================================
 *  A live, on-device OSCARLOCATOR azimuthal-equidistant satellite display for
 *  the M5Stack M5Paper S3 (ESP32-S3, 960x540 e-ink, GT911 touch).
 *
 *  Ported from the web-based OSCARLOCATOR Simulator (oscarlocator.n8hm.radio),
 *  itself modeled on OrbitDeck.
 *
 *  Credit: Paul Stoetzer, N8HM
 *          oscarlocator.n8hm.radio  /  github.com/prstoetzer/OrbitDeck
 *
 *  Features
 *    - Captive web page for configuration (satellite, QTH, WiFi)
 *    - NTP time
 *    - On-device AMSAT GP (daily-bulletin.json) parsing + SGP4 propagation
 *    - Live azimuthal-equidistant OSCARLOCATOR map:
 *        ground-track arc from the day's equator crossing,
 *        range circle over the QTH, satellite footprint at the sub-point,
 *        live sub-satellite dot.
 *    - Four projections, selectable on-device:
 *        Polar (auto N/S), Polar North, Polar South, QTH-centred azimuthal
 *    - Next-pass list screen (AOS / peak / LOS / duration / max el)
 *    - Reference-orbits screen (first equator crossing of each UTC day:
 *        the time + longitude you would dial into a real OSCARLOCATOR)
 *    - This is a LIVE display only. No manual EQX/minutes manipulation modes
 *      are ported -- on-device interaction is limited to choosing the view
 *      and the satellite.
 *
 *  Required libraries (Arduino IDE -> Library Manager / Boards Manager):
 *    - M5Unified                 (>= 0.2.5; pulls in M5GFX, board: M5PaperS3)
 *    - SparkFun SGP4 Arduino Lib (Hopperpop port: nextpass/findsat/satAz/satEl)
 *    - WiFiManager (tzapu)        captive portal for first-time WiFi
 *    - ArduinoJson (>= 6.x)       streaming parse of the AMSAT GP JSON
 *  Board package:
 *    https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
 *    Board: M5Stack -> M5PaperS3   (PSRAM: OPI / Octal enabled)
 *
 *  License: MIT
 * ============================================================================
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <time.h>
#include <math.h>
#include <ArduinoJson.h>
#include <Sgp4.h>
#include "coastline.h"

// ---------------------------------------------------------------------------
//  Constants & globals
// ---------------------------------------------------------------------------
static const char* AMSAT_GP_URL =
    "https://newark192.amsat.org/gpdata/current/daily-bulletin.json";
static const char* GP_CACHE = "/gpdata.json";

static const char* CONFIG_AP_SSID = "OSCARLOCATOR-Setup";
static const uint16_t WEB_PORT    = 80;

// e-ink full canvas after setRotation(1): 960 wide x 540 tall (landscape)
static const int SCREEN_W = 960;
static const int SCREEN_H = 540;

// Grayscale levels (0 = black, 15 = white on M5PaperS3 / M5GFX EPD).
// We use M5GFX color values; for EPD these map to 16-level grayscale.
static const int C_BLACK = 0;
static const int C_DGRAY = 4;
static const int C_GRAY  = 8;
static const int C_LGRAY = 11;
static const int C_WHITE = 15;

#define MAXSAT 220
#define MAXPASS 10
#define MAXREF 14

// Earth model
static const double EARTH_R_KM = 6378.137;
static const double DEG2RAD = M_PI / 180.0;
static const double RAD2DEG = 180.0 / M_PI;

// ---------------------------------------------------------------------------
//  Application state
// ---------------------------------------------------------------------------
Sgp4 sat;
Preferences prefs;
WebServer  server(WEB_PORT);
DNSServer  dnsServer;

struct SatEntry {
  char name[26];
  long norad;
};
SatEntry satList[MAXSAT];
int   satCount = 0;

char  currentTLE1[80] = "";
char  currentTLE2[80] = "";
char  selName[26]     = "ISS";
long  selNorad        = 25544;

double qthLat = 38.90;     // default: Washington, DC-ish (FM18)
double qthLon = -77.00;
double qthAltKm = 0.0;

// Projection / view selection
enum ViewMode { V_POLAR_AUTO = 0, V_POLAR_NORTH, V_POLAR_SOUTH, V_QTH };
ViewMode viewMode = V_POLAR_AUTO;
const char* viewName(ViewMode v) {
  switch (v) {
    case V_POLAR_AUTO:  return "Polar - auto N/S";
    case V_POLAR_NORTH: return "Polar - North";
    case V_POLAR_SOUTH: return "Polar - South";
    case V_QTH:         return "QTH-centred";
  }
  return "";
}

// Screen state machine
enum Screen { SCR_MAP = 0, SCR_PASSES, SCR_REFORBITS };
Screen screen = SCR_MAP;

// Pass list
struct PassRec { double aos, peak, los; double maxEl; };
PassRec passes[MAXPASS];
int passCount = 0;
double minPassEl = 0.0;   // degrees

// Reference orbits (first equator crossing of each UTC day)
struct RefRec { long ymd; double utc; double lon; };
RefRec refOrbits[MAXREF];
int refCount = 0;
int refDays = 7;

// Live sub-point cache
double subLat = 0, subLon = 0, subAltKm = 0;
double liveAz = 0, liveEl = -90, liveRangeKm = 0;
bool   liveValid = false;

// TLE freshness
time_t lastTLEEpoch = 0;
unsigned long lastDrawMs = 0;
String statusMsg = "Booting...";

bool wifiOK   = false;
bool timeOK   = false;

// Refresh cadence
unsigned long lastTickMs = 0;
const unsigned long TICK_VISIBLE_MS = 20000;   // 20 s when bird is up
const unsigned long TICK_HIDDEN_MS  = 60000;   // 60 s otherwise

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
static double wrap180(double d) {
  while (d > 180.0)  d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}
static double wrap360(double d) {
  while (d >= 360.0) d -= 360.0;
  while (d < 0.0)    d += 360.0;
  return d;
}

// Maidenhead grid -> lat/lon (center of square). Supports 4 or 6 chars.
bool gridToLatLon(const String& gridIn, double& lat, double& lon) {
  String g = gridIn; g.trim(); g.toUpperCase();
  if (g.length() < 4) return false;
  char A = g[0], B = g[1], C = g[2], D = g[3];
  if (A < 'A' || A > 'R' || B < 'A' || B > 'R') return false;
  if (C < '0' || C > '9' || D < '0' || D > '9') return false;
  lon = (A - 'A') * 20.0 - 180.0;
  lat = (B - 'A') * 10.0 - 90.0;
  lon += (C - '0') * 2.0;
  lat += (D - '0') * 1.0;
  if (g.length() >= 6) {
    char E = g[4], F = g[5];
    if (E >= 'A' && E <= 'X' && F >= 'A' && F <= 'X') {
      lon += (E - 'A') * (2.0 / 24.0);
      lat += (F - 'A') * (1.0 / 24.0);
      lon += (2.0 / 24.0) / 2.0;
      lat += (1.0 / 24.0) / 2.0;
      return true;
    }
  }
  lon += 1.0;   // center of 2deg cell
  lat += 0.5;   // center of 1deg cell
  return true;
}

// Footprint angular radius (deg) for a satellite at altitude h (km).
static double footprintDeg(double hKm) {
  double rho = acos(EARTH_R_KM / (EARTH_R_KM + hKm));   // radians
  return rho * RAD2DEG;
}

// Great-circle distance (deg of arc) between two lat/lon (deg).
static double gcArcDeg(double lat1, double lon1, double lat2, double lon2) {
  double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
  double dl = (lon2 - lon1) * DEG2RAD;
  double c = sin(p1) * sin(p2) + cos(p1) * cos(p2) * cos(dl);
  if (c > 1)  c = 1;
  if (c < -1) c = -1;
  return acos(c) * RAD2DEG;
}

// Azimuth (deg, from North CW) from point 1 to point 2.
static double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
  double dl = (lon2 - lon1) * DEG2RAD;
  double y = sin(dl) * cos(p2);
  double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  return wrap360(atan2(y, x) * RAD2DEG);
}

// ---------------------------------------------------------------------------
//  Projection: azimuthal-equidistant  (ported 1:1 from the OSCARLOCATOR site)
//
//  Three concrete modes; "polar-auto" resolves to North or South by hemisphere.
//  Screen-axis orientation matches the website (verified there against
//  matplotlib set_theta_zero_location / set_theta_direction):
//    polar  (North): rho = 90 - lat, t = lon ; [CX + r sin a, CY + r cos a]
//    polar-south   : rho = 90 + lat, t = lon ; [CX - r sin a, CY + r cos a]
//    qth           : rho,t from central-angle+bearing; [CX + r sin a, CY - r cos a]
//
//  Map edge in great-circle degrees (rmax): polar maps span pole->equator (90),
//  the QTH map is clamped to |qlat|+25 in [50,80] like OrbitDeck.
// ---------------------------------------------------------------------------
enum ResolvedMode { RM_POLAR_NORTH = 0, RM_POLAR_SOUTH, RM_QTH };

struct Proj {
  ResolvedMode rm;
  int    cx, cy, R;    // canvas geometry
  double rmaxDeg;
};

Proj gProj;

ResolvedMode resolveMode() {
  if (viewMode == V_QTH)          return RM_QTH;
  if (viewMode == V_POLAR_NORTH)  return RM_POLAR_NORTH;
  if (viewMode == V_POLAR_SOUTH)  return RM_POLAR_SOUTH;
  // polar-auto: follow the live sub-satellite latitude if we have it, else QTH
  if (liveValid) return (subLat < 0) ? RM_POLAR_SOUTH : RM_POLAR_NORTH;
  return (qthLat < 0) ? RM_POLAR_SOUTH : RM_POLAR_NORTH;
}

void setupProjection(Proj& P) {
  // Clear area is y in [58, 476] (header 56, tab bar starts at 478).
  P.cx = 268; P.cy = 268; P.R = 205;   // map disc on the left, clear of bars
  P.rm = resolveMode();
  if (P.rm == RM_QTH) {
    double v = fabs(qthLat) + 25.0;
    if (v < 50.0) v = 50.0;
    if (v > 80.0) v = 80.0;
    P.rmaxDeg = v;
  } else {
    P.rmaxDeg = 90.0;
  }
}

// (rhoDeg, tval) for the current mode. tval is azimuth (qth) or longitude.
static void toPolar(const Proj& P, double lat, double lon,
                    double& rhoDeg, double& tval) {
  if (P.rm == RM_POLAR_NORTH) { rhoDeg = 90.0 - lat; tval = wrap360(lon); return; }
  if (P.rm == RM_POLAR_SOUTH) { rhoDeg = 90.0 + lat; tval = wrap360(lon); return; }
  rhoDeg = gcArcDeg(qthLat, qthLon, lat, lon);
  tval   = bearingDeg(qthLat, qthLon, lat, lon);
}

// Place a (rhoDeg, tval) on the disc honouring each mode's axis orientation.
static bool placeOnDisc(const Proj& P, double rhoDeg, double tval,
                        int& px, int& py) {
  if (rhoDeg > P.rmaxDeg) return false;
  double r = rhoDeg / P.rmaxDeg * P.R;
  double a = tval * DEG2RAD;
  double x, y;
  if (P.rm == RM_POLAR_NORTH)      { x = P.cx + r * sin(a); y = P.cy + r * cos(a); }
  else if (P.rm == RM_POLAR_SOUTH) { x = P.cx - r * sin(a); y = P.cy + r * cos(a); }
  else                             { x = P.cx + r * sin(a); y = P.cy - r * cos(a); }
  px = (int)lround(x); py = (int)lround(y);
  return true;
}

// Project (lat,lon) -> canvas (x,y). Returns false if outside the disc.
bool project(const Proj& P, double lat, double lon, int& px, int& py) {
  double rhoDeg, tval;
  toPolar(P, lat, lon, rhoDeg, tval);
  return placeOnDisc(P, rhoDeg, tval, px, py);
}

// Screen point for a rim/value angle (deg) at radius fraction rFrac (0..1+).
static void rimPoint(const Proj& P, double tvalDeg, double rFrac,
                     int& px, int& py) {
  double a = tvalDeg * DEG2RAD, r = rFrac * P.R;
  double x, y;
  if (P.rm == RM_POLAR_NORTH)      { x = P.cx + r * sin(a); y = P.cy + r * cos(a); }
  else if (P.rm == RM_POLAR_SOUTH) { x = P.cx - r * sin(a); y = P.cy + r * cos(a); }
  else                             { x = P.cx + r * sin(a); y = P.cy - r * cos(a); }
  px = (int)lround(x); py = (int)lround(y);
}

// Mean altitude (km) from mean motion via Kepler's third law (matches site).
static double meanAltKm() {
  double revPerDay = sat.revpday;          // rev/day from the propagator
  if (revPerDay <= 0.1) return 800.0;
  double nRadSec = revPerDay * 2.0 * M_PI / 86400.0;
  double mu = 398600.4418;
  double a = cbrt(mu / (nRadSec * nRadSec));
  double h = a - EARTH_R_KM;
  return (h < 1.0) ? 1.0 : h;
}

// ---------------------------------------------------------------------------
//  Persistent config
// ---------------------------------------------------------------------------
void loadConfig() {
  prefs.begin("oscarloc", true);
  qthLat   = prefs.getDouble("lat", qthLat);
  qthLon   = prefs.getDouble("lon", qthLon);
  selNorad = prefs.getLong("norad", selNorad);
  String n = prefs.getString("sname", String(selName));
  n.toCharArray(selName, sizeof(selName));
  viewMode = (ViewMode)prefs.getUChar("view", (uint8_t)viewMode);
  minPassEl= prefs.getDouble("minel", minPassEl);
  refDays  = prefs.getInt("refdays", refDays);
  lastTLEEpoch = (time_t)prefs.getLong("tletime", 0);
  prefs.end();
}
void saveConfig() {
  prefs.begin("oscarloc", false);
  prefs.putDouble("lat", qthLat);
  prefs.putDouble("lon", qthLon);
  prefs.putLong("norad", selNorad);
  prefs.putString("sname", String(selName));
  prefs.putUChar("view", (uint8_t)viewMode);
  prefs.putDouble("minel", minPassEl);
  prefs.putInt("refdays", refDays);
  prefs.putLong("tletime", (long)lastTLEEpoch);
  prefs.end();
}

// ---------------------------------------------------------------------------
//  AMSAT GP element acquisition + parsing (daily-bulletin.json, OMM/GP form)
//
//  AMSAT now distributes "GP" (General Perturbations) data as JSON: an array
//  of OMM-style records with fields AMSAT_NAME, NORAD_CAT_ID, EPOCH (ISO),
//  INCLINATION, RA_OF_ASC_NODE, ECCENTRICITY, ARG_OF_PERICENTER, MEAN_ANOMALY,
//  MEAN_MOTION (rev/day), BSTAR, MEAN_MOTION_DOT/DDOT, etc.
//
//  The SGP4 propagator (Hopperpop) is initialised from two TLE strings, so we
//  synthesise standard, column-exact TLE line1/line2 from the GP fields on the
//  device (correct epoch yyddd.dddddddd, assumed-decimal exponent encoding for
//  BSTAR/ndot/nddot, and the mod-10 checksum), then hand them to sat.init().
//  This keeps the verified SGP4 path intact while consuming GP JSON natively.
//
//  We cache the raw JSON in LittleFS and parse it into satList[]; for the
//  selected NORAD we synthesise currentTLE1/2.
// ---------------------------------------------------------------------------

// mod-10 TLE checksum over the first 68 chars (digits sum; '-' counts as 1).
static int tleChecksum(const char* line) {
  int s = 0;
  for (int i = 0; i < 68 && line[i]; i++) {
    char c = line[i];
    if (c >= '0' && c <= '9') s += c - '0';
    else if (c == '-') s += 1;
  }
  return s % 10;
}

// Assumed-decimal-point exponential field, 8 chars: sign,5 mantissa,exp sign,
// exp digit. e.g. 2.6888938e-05 -> " 26889-4"; 0 -> " 00000+0".
static void fmtExp(double val, char* out) {
  if (val == 0.0) { strcpy(out, " 00000+0"); return; }
  char sign = (val < 0) ? '-' : ' ';
  double a = fabs(val);
  int exp = 0;
  while (a >= 1.0) { a /= 10.0; exp++; }
  while (a < 0.1)  { a *= 10.0; exp--; }
  long m = lround(a * 100000.0);
  if (m >= 100000) { m = 10000; exp++; }
  int ed = abs(exp);
  if (ed > 9) ed = 9;                 // TLE exponent is a single digit
  char es = (exp < 0) ? '-' : '+';
  out[0] = sign;
  snprintf(out + 1, 8, "%05ld%c%d", m, es, ed);
  out[8] = 0;
}

// First time derivative of mean motion, 10 chars: sign + .xxxxxxxx (8 dp).
static void fmtNdot(double val, char* out) {
  char sign = (val < 0) ? '-' : ' ';
  char tmp[16];
  snprintf(tmp, sizeof(tmp), "%.8f", fabs(val));   // "0.00000042"
  // drop the leading '0', keep the dot and 8 decimals
  out[0] = sign;
  strncpy(out + 1, tmp + 1, 9);                    // ".00000042"
  out[10] = 0;
}

// EPOCH ISO "YYYY-MM-DDThh:mm:ss[.ffffff]" -> "YYDDD.DDDDDDDD" (14 chars).
static void fmtEpoch(const char* iso, char* out) {
  int Y, Mo, D, h, mi; double s = 0;
  // tolerate fractional seconds
  if (sscanf(iso, "%d-%d-%dT%d:%d:%lf", &Y, &Mo, &D, &h, &mi, &s) < 6) {
    sscanf(iso, "%d-%d-%dT%d:%d", &Y, &Mo, &D, &h, &mi);
  }
  static const int cum[] = {0,31,59,90,120,151,181,212,243,273,304,334};
  int doy = cum[(Mo - 1) % 12] + D;
  bool leap = (Y % 4 == 0 && (Y % 100 != 0 || Y % 400 == 0));
  if (leap && Mo > 2) doy += 1;
  double dayfrac = (h * 3600.0 + mi * 60.0 + s) / 86400.0;
  double ddd = doy + dayfrac;
  if (ddd >= 367.0) ddd = 366.99999999;
  char tmp[24];
  snprintf(tmp, sizeof(tmp), "%02d%012.8f", Y % 100, ddd);
  memcpy(out, tmp, 14); out[14] = 0;
}

// International designator from OBJECT_ID "1974-089B" -> "74089B  " (8 cols).
static void fmtIntl(const char* objid, char* out) {
  memset(out, ' ', 8); out[8] = 0;
  // yy = chars 2..3
  if (strlen(objid) >= 4) { out[0] = objid[2]; out[1] = objid[3]; }
  const char* dash = strchr(objid, '-');
  const char* rest = dash ? dash + 1 : (objid + 5);
  for (int i = 0; i < 6 && rest[i]; i++) out[2 + i] = rest[i];
}

// Synthesise both TLE lines from GP fields. line1/line2 buffers >= 70 chars.
void gpToTle(long norad, const char* objid, const char* epochIso,
             double ndot, double nddot, double bstar, long elset,
             double incl, double raan, double ecc, double argp,
             double ma, double mm, long revAtEpoch,
             char* line1, char* line2) {
  char L1[70]; memset(L1, ' ', 69); L1[69] = 0;
  char buf[20];
  L1[0] = '1';
  snprintf(buf, sizeof(buf), "%05ld", norad); memcpy(L1 + 2, buf, 5);
  L1[7] = 'U';
  char intl[9]; fmtIntl(objid, intl); memcpy(L1 + 9, intl, 8);     // 10-17
  char ep[15]; fmtEpoch(epochIso, ep); memcpy(L1 + 18, ep, 14);    // 19-32
  char nd[12]; fmtNdot(ndot, nd); memcpy(L1 + 33, nd, 10);         // 34-43
  char ndd[9]; fmtExp(nddot, ndd); memcpy(L1 + 44, ndd, 8);        // 45-52
  char bs[9];  fmtExp(bstar, bs);  memcpy(L1 + 53, bs, 8);         // 54-61
  L1[62] = '0';                                                    // ephem type
  snprintf(buf, sizeof(buf), "%4ld", elset); memcpy(L1 + 64, buf, 4); // 65-68
  L1[68] = '0' + tleChecksum(L1);
  L1[69] = 0;
  strncpy(line1, L1, 70);

  char L2[70]; memset(L2, ' ', 69); L2[69] = 0;
  L2[0] = '2';
  snprintf(buf, sizeof(buf), "%05ld", norad); memcpy(L2 + 2, buf, 5);
  snprintf(buf, sizeof(buf), "%8.4f", incl); memcpy(L2 + 8, buf, 8);   // 9-16
  snprintf(buf, sizeof(buf), "%8.4f", raan); memcpy(L2 + 17, buf, 8);  // 18-25
  long e7 = lround(ecc * 1e7);
  snprintf(buf, sizeof(buf), "%07ld", e7); memcpy(L2 + 26, buf, 7);    // 27-33
  snprintf(buf, sizeof(buf), "%8.4f", argp); memcpy(L2 + 34, buf, 8);  // 35-42
  snprintf(buf, sizeof(buf), "%8.4f", ma);  memcpy(L2 + 43, buf, 8);   // 44-51
  snprintf(buf, sizeof(buf), "%11.8f", mm); memcpy(L2 + 52, buf, 11);  // 53-63
  snprintf(buf, sizeof(buf), "%5ld", revAtEpoch); memcpy(L2 + 63, buf, 5); // 64-68
  L2[68] = '0' + tleChecksum(L2);
  L2[69] = 0;
  strncpy(line2, L2, 70);
}

// Parse the GP JSON array. Fills satList[]; for the selected NORAD also
// synthesises currentTLE1/2. Uses a filtered streaming ArduinoJson parse so
// only the fields we need are kept (the full file is large).
bool parseGPJson(Stream& stream) {
  satCount = 0;
  JsonDocument filter;          // keep only needed keys
  filter[0]["AMSAT_NAME"]        = true;
  filter[0]["NORAD_CAT_ID"]      = true;
  filter[0]["OBJECT_ID"]         = true;
  filter[0]["EPOCH"]             = true;
  filter[0]["INCLINATION"]       = true;
  filter[0]["RA_OF_ASC_NODE"]    = true;
  filter[0]["ECCENTRICITY"]      = true;
  filter[0]["ARG_OF_PERICENTER"] = true;
  filter[0]["MEAN_ANOMALY"]      = true;
  filter[0]["MEAN_MOTION"]       = true;
  filter[0]["BSTAR"]             = true;
  filter[0]["MEAN_MOTION_DOT"]   = true;
  filter[0]["MEAN_MOTION_DDOT"]  = true;
  filter[0]["ELEMENT_SET_NO"]    = true;
  filter[0]["REV_AT_EPOCH"]      = true;

  JsonDocument doc;             // ESP32 PSRAM-backed; file fits comfortably
  DeserializationError err =
      deserializeJson(doc, stream, DeserializationOption::Filter(filter));
  if (err) return false;

  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject o : arr) {
    if (satCount >= MAXSAT) break;
    long norad = o["NORAD_CAT_ID"] | 0L;
    if (norad == 0) continue;
    const char* nm = o["AMSAT_NAME"] | "";
    if (!nm[0]) nm = o["OBJECT_ID"] | "SAT";
    SatEntry& e = satList[satCount];
    strlcpy(e.name, nm, sizeof(e.name));
    e.norad = norad;
    if (norad == selNorad) {
      gpToTle(norad, o["OBJECT_ID"] | "0000-000A", o["EPOCH"] | "",
              o["MEAN_MOTION_DOT"] | 0.0, o["MEAN_MOTION_DDOT"] | 0.0,
              o["BSTAR"] | 0.0, o["ELEMENT_SET_NO"] | 999L,
              o["INCLINATION"] | 0.0, o["RA_OF_ASC_NODE"] | 0.0,
              o["ECCENTRICITY"] | 0.0, o["ARG_OF_PERICENTER"] | 0.0,
              o["MEAN_ANOMALY"] | 0.0, o["MEAN_MOTION"] | 0.0,
              o["REV_AT_EPOCH"] | 0L, currentTLE1, currentTLE2);
      strlcpy(selName, nm, sizeof(selName));
    }
    satCount++;
  }
  return satCount > 0;
}

bool loadCachedGP() {
  if (!LittleFS.exists(GP_CACHE)) return false;
  File f = LittleFS.open(GP_CACHE, "r");
  if (!f) return false;
  bool ok = parseGPJson(f);
  f.close();
  return ok;
}

bool fetchGP() {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure client;
  client.setInsecure();                 // AMSAT cert not pinned on-device
  HTTPClient http;
  http.setReuse(false);
  if (!http.begin(client, AMSAT_GP_URL)) return false;
  http.setTimeout(20000);
  int code = http.GET();
  if (code != 200) { http.end(); return false; }
  // Stream the body straight to LittleFS so we never hold the whole file twice.
  File f = LittleFS.open(GP_CACHE, "w");
  if (!f) { http.end(); return false; }
  http.writeToStream(&f);
  f.close();
  http.end();
  bool ok = loadCachedGP();
  if (ok && timeOK) { lastTLEEpoch = time(nullptr); saveConfig(); }
  return ok;
}

// (Re)load the propagator with the currently selected satellite.
bool loadSelectedIntoSat() {
  if (currentTLE1[0] == 0 || currentTLE2[0] == 0) return false;
  sat.site(qthLat, qthLon, qthAltKm);
  sat.init(selName, currentTLE1, currentTLE2);
  return true;
}

// Select a satellite by NORAD from the cached GP JSON; synthesise its TLE.
bool selectSatByNorad(long norad) {
  selNorad = norad;
  if (!LittleFS.exists(GP_CACHE)) return false;
  File f = LittleFS.open(GP_CACHE, "r");
  if (!f) return false;
  bool ok = parseGPJson(f);          // re-parse; fills currentTLE1/2 for norad
  f.close();
  return ok && currentTLE1[0] != 0;
}

// ---------------------------------------------------------------------------
//  Orbital computation
// ---------------------------------------------------------------------------
void updateLivePosition() {
  liveValid = false;
  if (currentTLE1[0] == 0) return;
  time_t now = time(nullptr);
  if (now < 1700000000) return;     // time not set
  sat.findsat((unsigned long)now);
  subLat  = sat.satLat;
  subLon  = wrap180(sat.satLon);
  subAltKm= sat.satAlt;
  liveAz  = sat.satAz;
  liveEl  = sat.satEl;
  liveRangeKm = sat.satDist;
  liveValid = true;
}

// Pass prediction -- mirrors PaperSat's hybrid approach: trust the library's
// peak/LOS/maxEl, but refine AOS by searching backward from the peak.
void predictPasses() {
  passCount = 0;
  if (currentTLE1[0] == 0 || !timeOK) return;
  sat.site(qthLat, qthLon, qthAltKm);
  sat.init(selName, currentTLE1, currentTLE2);
  time_t now = time(nullptr);
  passinfo overpass;
  sat.initpredpoint((unsigned long)now, minPassEl);   // unix overload

  for (int p = 0; p < MAXPASS; p++) {
    bool found = sat.nextpass(&overpass, 40, false, minPassEl);
    if (!found) break;

    // All pass times are Julian dates in the passinfo struct.
    double jdstart = overpass.jdstart;   // library AOS estimate
    double jdmax   = overpass.jdmax;     // peak
    double jdstop  = overpass.jdstop;    // LOS
    double maxEl   = overpass.maxelevation;

    // Julian date -> unix seconds.
    double peakUnix = (jdmax  - 2440587.5) * 86400.0;
    double losUnix  = (jdstop - 2440587.5) * 86400.0;
    double aosSeed  = (jdstart - 2440587.5) * 86400.0;

    // Refine AOS: start from the library estimate, walk backward from the
    // peak in 30 s steps until El <= minEl, then 1 s fine steps forward to
    // the precise crossing. Robust even on eccentric orbits (e.g. RS-44).
    double aosUnix = (aosSeed > 0 && aosSeed < peakUnix) ? aosSeed : peakUnix;
    {
      double t = peakUnix;
      for (int s = 0; s < 600; s++) {           // up to 5 hours back
        t -= 30.0;
        sat.findsat((unsigned long)t);
        if (sat.satEl <= minPassEl) break;
        aosUnix = t;
      }
      double t2 = aosUnix - 30.0;
      for (int s = 0; s < 30; s++) {
        t2 += 1.0;
        sat.findsat((unsigned long)t2);
        if (sat.satEl > minPassEl) { aosUnix = t2; break; }
      }
    }

    passes[passCount].aos   = aosUnix;
    passes[passCount].peak  = peakUnix;
    passes[passCount].los   = losUnix;
    passes[passCount].maxEl = maxEl;
    passCount++;
  }

  // restore propagator default site after using initpredpoint scans
  sat.site(qthLat, qthLon, qthAltKm);
}

// Reference orbits: first equator crossing of each UTC day.
// Northern stations use the ascending node; southern use the descending node.
// We scan forward day by day, find the first node crossing of the chosen
// direction at/after 00:00 UTC, refine by bisection, and record UTC + lon.
bool firstNodeOfDay(time_t dayStart, bool ascending, double& outUtcHrs,
                    double& outLon) {
  // Scan in 60 s steps across the UTC day for the first sign change of
  // latitude in the chosen direction.
  double prevLat = 0; bool havePrev = false;
  for (long s = 0; s <= 86400; s += 60) {
    double t = (double)dayStart + s;
    sat.findsat((unsigned long)t);
    double lat = sat.satLat;
    if (havePrev) {
      bool cross = ascending ? (prevLat < 0 && lat >= 0)
                             : (prevLat > 0 && lat <= 0);
      if (cross) {
        // bisection between t-60 and t
        double a = t - 60, b = t;
        for (int it = 0; it < 30; it++) {
          double m = 0.5 * (a + b);
          sat.findsat((unsigned long)m);
          double lm = sat.satLat;
          bool upper = ascending ? (lm >= 0) : (lm <= 0);
          if (upper) b = m; else a = m;
        }
        double m = 0.5 * (a + b);
        sat.findsat((unsigned long)m);
        outLon = wrap180(sat.satLon);
        outUtcHrs = (m - (double)dayStart) / 3600.0;
        return true;
      }
    }
    prevLat = lat; havePrev = true;
  }
  return false;
}

void buildReferenceOrbits() {
  refCount = 0;
  if (currentTLE1[0] == 0 || !timeOK) return;
  sat.site(qthLat, qthLon, qthAltKm);
  sat.init(selName, currentTLE1, currentTLE2);
  bool ascending = (qthLat >= 0.0);   // N hemi -> ascending node

  time_t now = time(nullptr);
  // Floor 'now' to the start of the current UTC day without timegm()
  // (which is not reliably available in the ESP32 toolchain).
  time_t day0 = (now / 86400) * 86400;

  for (int d = 0; d < refDays && refCount < MAXREF; d++) {
    time_t dayStart = day0 + (time_t)d * 86400;
    double utcHrs, lon;
    if (firstNodeOfDay(dayStart, ascending, utcHrs, lon)) {
      struct tm dt; gmtime_r(&dayStart, &dt);
      refOrbits[refCount].ymd = (dt.tm_year + 1900) * 10000 +
                                (dt.tm_mon + 1) * 100 + dt.tm_mday;
      refOrbits[refCount].utc = utcHrs;
      refOrbits[refCount].lon = lon;
      refCount++;
    }
  }
}

// ---------------------------------------------------------------------------
//  Ground-track arc for the current orbit (for the map view).
//  The OSCARLOCATOR arc is the sub-satellite track of ONE orbit, drawn from
//  its equator crossing. For a live display we draw the track spanning roughly
//  one orbital period centred on the current time, so the bird sits on its arc.
// ---------------------------------------------------------------------------
#define ARC_PTS 121
double arcLat[ARC_PTS], arcLon[ARC_PTS];
int    arcN = 0;

void buildGroundArc() {
  arcN = 0;
  if (currentTLE1[0] == 0 || !timeOK) return;
  time_t now = time(nullptr);
  // mean motion (rev/day) from the propagator -> period seconds.
  double periodS = 95.0 * 60.0;          // fallback ~95 min
  if (sat.revpday > 0.1) periodS = 86400.0 / sat.revpday;
  double half = periodS * 0.6;       // a little over half an orbit each side
  for (int k = 0; k < ARC_PTS; k++) {
    double frac = (double)k / (ARC_PTS - 1);
    double t = (double)now - half + frac * (2.0 * half);
    sat.findsat((unsigned long)t);
    arcLat[arcN] = sat.satLat;
    arcLon[arcN] = wrap180(sat.satLon);
    arcN++;
  }
  // restore live position afterwards
  updateLivePosition();
}

// ===========================================================================
//  RENDERING
// ===========================================================================
M5Canvas canvas(&M5.Display);   // off-screen buffer for smooth e-ink output
bool canvasReady = false;

void ensureCanvas() {
  if (canvasReady) return;
  canvas.setColorDepth(4);                 // 16-level grayscale
  canvas.createSprite(SCREEN_W, SCREEN_H);
  canvas.setTextWrap(false);
  canvasReady = true;
}

// ---------------------------------------------------------------------------
//  E-ink refresh strategy
//
//  The whole frame is always composited into the off-screen `canvas` (cheap,
//  in PSRAM). What differs is how much of it we push to the panel and with
//  which waveform:
//
//   - FULL  (epd_quality): a complete black-white-black clear. Best quality,
//     no ghosting, ~1 s, hard on the panel. Used only on screen changes,
//     satellite/view/QTH changes, and a periodic de-ghost.
//   - PARTIAL (epd_fast): white/black partial update of a sub-rectangle only.
//     Fast and low-wear, mild ghosting. Used for the per-tick live updates
//     (clock, readout, moving sat dot, table header) where the static base
//     map / table body is unchanged.
//
//  After several partial updates ghosting accumulates, so we force a FULL
//  refresh every PARTIALS_BEFORE_DEGHOST partials (or once an hour, whichever
//  comes first).
// ---------------------------------------------------------------------------
static const int PARTIALS_BEFORE_DEGHOST = 30;
int  partialsSinceFull = 0;
bool fullRedrawNeeded  = true;     // first paint is always full

// Push the entire canvas with a full-quality clear.
void flushFull() {
  M5.Display.setEpdMode(epd_quality);
  canvas.pushSprite(0, 0);
  partialsSinceFull = 0;
  fullRedrawNeeded  = false;
}

// Partial update of a sub-rectangle using the fast waveform.
//
//  M5GFX (Panel_EPD) on the M5PaperS3 does not expose a clean per-rectangle
//  display(x,y,w,h) refresh, so to limit what the panel actually refreshes we
//  copy the dirty rectangle of the main canvas into a scratch sprite sized
//  exactly to that rectangle and pushSprite() it at (x,y) in epd_fast mode.
//  The EPD then refreshes only the region the pushed sprite covers -- fast
//  waveform, low wear -- while the static base map / table body is untouched.
//  Partials are infrequent (every 20-60 s) so allocating the scratch sprite
//  per call in PSRAM is negligible.
M5Canvas patchSprite(&M5.Display);

void flushPartial(int x, int y, int w, int h) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y + h > SCREEN_H) h = SCREEN_H - y;
  if (w <= 0 || h <= 0) return;

  patchSprite.setColorDepth(4);
  if (!patchSprite.createSprite(w, h)) return;   // PSRAM alloc
  // Blit the dirty region of the main canvas into the scratch sprite by
  // pushing the whole canvas onto it at offset (-x,-y); LovyanGFX clips to the
  // scratch sprite's bounds, so region [x,y,w,h] lands at the sprite origin.
  canvas.pushSprite(&patchSprite, -x, -y);
  // Refresh just that region with the fast waveform.
  M5.Display.setEpdMode(epd_fast);
  patchSprite.pushSprite(x, y);
  patchSprite.deleteSprite();
  if (++partialsSinceFull >= PARTIALS_BEFORE_DEGHOST) fullRedrawNeeded = true;
}

// Dirty-region rectangles used for partial flushes.
// Header strip carries the clock/battery; readout panel carries live numbers;
// table header carries the per-tick clock on the list screens.
static const int RECT_HEADER_X = 540, RECT_HEADER_Y = 0,
                 RECT_HEADER_W = SCREEN_W - 540, RECT_HEADER_H = 56;
static const int RECT_READOUT_X = 564, RECT_READOUT_Y = 64,
                 RECT_READOUT_W = SCREEN_W - 564, RECT_READOUT_H = SCREEN_H - 126;

// The moving sat dot + its footprint are flushed via a bounding box that
// covers the previous and current positions so the old dot is cleared.
int  prevDotX = -1, prevDotY = -1;
bool havePrevDot = false;

void drawHeader(const char* title) {
  canvas.fillRect(0, 0, SCREEN_W, 56, C_WHITE);
  canvas.drawFastHLine(0, 56, SCREEN_W, C_BLACK);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_BLACK, C_WHITE);
  canvas.setTextSize(1);
  canvas.setFont(&fonts::FreeSansBold18pt7b);
  canvas.drawString(title, 16, 12);

  // clock + sat + battery, right aligned
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextDatum(top_right);
  char buf[64];
  time_t now = time(nullptr);
  struct tm t; gmtime_r(&now, &t);
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d  %02d:%02d:%02d UTC",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  canvas.drawString(buf, SCREEN_W - 16, 8);

  int bat = M5.Power.getBatteryLevel();
  snprintf(buf, sizeof(buf), "%s   bat %d%%   %s",
           selName, bat, wifiOK ? "wifi" : "no-wifi");
  canvas.drawString(buf, SCREEN_W - 16, 30);
  canvas.setTextDatum(top_left);
}

// Footer tab bar with the three screens + (on map) view button.
struct Btn { int x, y, w, h; const char* label; };
Btn tabMap   = {  16, SCREEN_H - 56, 150, 44, "Map" };
Btn tabPass  = { 176, SCREEN_H - 56, 200, 44, "Next passes" };
Btn tabRef   = { 386, SCREEN_H - 56, 230, 44, "Reference orbits" };
Btn btnView  = { 632, SCREEN_H - 56, 180, 44, "View" };
Btn btnSat   = { 632, SCREEN_H - 56, 180, 44, "Satellite" };  // reused per screen
Btn btnRefr  = { 822, SCREEN_H - 56, 122, 44, "Refresh" };

void drawButton(const Btn& b, bool active) {
  int fg = active ? C_WHITE : C_BLACK;
  int bg = active ? C_BLACK : C_WHITE;
  canvas.fillRoundRect(b.x, b.y, b.w, b.h, 8, bg);
  canvas.drawRoundRect(b.x, b.y, b.w, b.h, 8, C_BLACK);
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(fg, bg);
  canvas.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_BLACK, C_WHITE);
}

void drawTabBar() {
  canvas.drawFastHLine(0, SCREEN_H - 62, SCREEN_W, C_BLACK);
  drawButton(tabMap,  screen == SCR_MAP);
  drawButton(tabPass, screen == SCR_PASSES);
  drawButton(tabRef,  screen == SCR_REFORBITS);
  if (screen == SCR_MAP) drawButton(btnView, false);
  else                   drawButton(btnSat, false);
  drawButton(btnRefr, false);
}

// Plot a poly-line of (lat,lon) handling disc clipping & antimeridian jumps.
void drawTrack(const Proj& P, double* lat, double* lon, int n, int color) {
  int px0 = 0, py0 = 0; bool have = false;
  for (int k = 0; k < n; k++) {
    int px, py;
    bool in = project(P, lat[k], lon[k], px, py);
    if (in && have) {
      if (abs(px - px0) < P.R && abs(py - py0) < P.R)
        canvas.drawLine(px0, py0, px, py, color);
    }
    if (in) { px0 = px; py0 = py; have = true; }
    else have = false;
  }
}

// great-circle small circle of angular radius distDeg about (clat,clon),
// drawn as a projected polyline (matches the site's smallCircle()).
void drawGeoCircle(const Proj& P, double clat, double clon, double radDeg,
                   int color, bool dashed) {
  int px0 = 0, py0 = 0; bool have = false; int seg = 0;
  double p1 = clat * DEG2RAD, l1 = clon * DEG2RAD, d = radDeg * DEG2RAD;
  for (int a = 0; a <= 360; a += 3) {
    double br = a * DEG2RAD;
    double lat2 = asin(sin(p1) * cos(d) + cos(p1) * sin(d) * cos(br));
    double lon2 = l1 + atan2(sin(br) * sin(d) * cos(p1),
                             cos(d) - sin(p1) * sin(lat2));
    double La = lat2 * RAD2DEG;
    double Lo = wrap180(lon2 * RAD2DEG);
    int px, py;
    bool in = project(P, La, Lo, px, py);
    if (in && have && abs(px - px0) < P.R && abs(py - py0) < P.R) {
      if (!dashed || (seg++ & 1) == 0)
        canvas.drawLine(px0, py0, px, py, color);
    }
    if (in) { px0 = px; py0 = py; have = true; }
    else have = false;
  }
}

// Draw the coastlines from the embedded compact int16 table.
void drawCoastlines(const Proj& P) {
  int px0 = 0, py0 = 0; bool have = false;
  for (int i = 0; i + 1 < COAST_LEN; i += 2) {
    int16_t la = (int16_t)pgm_read_word(&COAST[i]);
    int16_t lo = (int16_t)pgm_read_word(&COAST[i + 1]);
    if (la == -32768 && lo == -32768) { have = false; continue; }  // ring break
    double lat = la / 100.0, lon = lo / 100.0;
    int px, py;
    bool in = project(P, lat, lon, px, py);
    if (in && have && abs(px - px0) < P.R && abs(py - py0) < P.R)
      canvas.drawLine(px0, py0, px, py, C_GRAY);
    if (in) { px0 = px; py0 = py; have = true; }
    else have = false;
  }
}

// Graticule + concentric rings + spokes + labels, matching the website layout.
void drawGraticule(const Proj& P) {
  bool polar = (P.rm != RM_QTH);
  bool south = (P.rm == RM_POLAR_SOUTH);

  // sea disc outline
  canvas.drawCircle(P.cx, P.cy, P.R, C_BLACK);

  // lat/lon graticule (faint)
  double latLo = south ? -90 : (polar ? 0 : -80);
  double latHi = south ?   0 : (polar ? 90 :  88);
  int lonStep = polar ? 30 : 15;
  for (int lon = -180; lon < 180; lon += lonStep) {
    int px0 = 0, py0 = 0; bool have = false;
    for (double lat = latLo; lat <= latHi; lat += 4) {
      int px, py;
      bool in = project(P, lat, (double)lon, px, py);
      if (in && have && abs(px-px0) < P.R && abs(py-py0) < P.R)
        canvas.drawLine(px0, py0, px, py, C_LGRAY);
      if (in) { px0 = px; py0 = py; have = true; } else have = false;
    }
  }
  double parLo = south ? -75 : (polar ? 0 : -75), parHi = south ? 0 : 75;
  for (double lat = parLo; lat <= parHi; lat += 15) {
    int px0 = 0, py0 = 0; bool have = false;
    for (int lon = -180; lon <= 180; lon += 4) {
      int px, py;
      bool in = project(P, lat, (double)lon, px, py);
      if (in && have && abs(px-px0) < P.R && abs(py-py0) < P.R)
        canvas.drawLine(px0, py0, px, py, C_LGRAY);
      if (in) { px0 = px; py0 = py; have = true; } else have = false;
    }
  }

  // coastlines
  drawCoastlines(P);

  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextColor(C_DGRAY, C_WHITE);
  if (polar) {
    // latitude rings every 15 deg + meridian-60 labels
    for (int latAbs = 15; latAbs < 90; latAbs += 15) {
      double ring = 90 - latAbs;
      int rr = (int)lround(ring / P.rmaxDeg * P.R);
      canvas.drawCircle(P.cx, P.cy, rr, C_LGRAY);
    }
    // longitude spokes every 30 deg + labels at the rim
    canvas.setTextDatum(middle_center);
    for (int lon = 0; lon < 360; lon += 30) {
      int x2, y2; rimPoint(P, (double)lon, 1.0, x2, y2);
      canvas.drawLine(P.cx, P.cy, x2, y2, C_LGRAY);
      int lx, ly; rimPoint(P, (double)lon, 1.0 + 16.0 / P.R, lx, ly);
      int disp = lon <= 180 ? lon : lon - 360;
      char buf[8];
      const char* hemi = (disp > 0 && disp < 180) ? "E" : (disp < 0 ? "W" : "");
      snprintf(buf, sizeof(buf), "%d%s", abs(disp), hemi);
      if (lx > P.cx - P.R - 30 && lx < P.cx + P.R + 30)
        canvas.drawString(buf, lx, ly);
    }
    canvas.fillCircle(P.cx, P.cy, 3, C_DGRAY);   // pole marker
  } else {
    // QTH: great-circle range rings every 30 deg + azimuth spokes + cardinals
    for (int rho = 30; rho <= (int)P.rmaxDeg; rho += 30) {
      int rr = (int)lround(rho / P.rmaxDeg * P.R);
      canvas.drawCircle(P.cx, P.cy, rr, C_LGRAY);
    }
    canvas.setTextDatum(middle_center);
    for (int az = 0; az < 360; az += 30) {
      int x2, y2; rimPoint(P, (double)az, 1.0, x2, y2);
      canvas.drawLine(P.cx, P.cy, x2, y2, C_LGRAY);
    }
    const char* card[4] = {"N", "E", "S", "W"};
    for (int i = 0; i < 4; i++) {
      int lx, ly; rimPoint(P, i * 90.0, 1.0 + 18.0 / P.R, lx, ly);
      canvas.drawString(card[i], lx, ly);
    }
  }
  canvas.setTextColor(C_BLACK, C_WHITE);
  canvas.setTextDatum(top_left);
}

// 5-point star (for the QTH marker on polar maps).
void drawStar(int cx, int cy, int rO, int rI) {
  int lastx = 0, lasty = 0;
  for (int i = 0; i <= 10; i++) {
    double r = (i % 2 == 0) ? rO : rI;
    double a = (i / 10.0) * 2 * M_PI - M_PI / 2;
    int x = (int)lround(cx + r * cos(a));
    int y = (int)lround(cy + r * sin(a));
    if (i > 0) canvas.drawLine(lastx, lasty, x, y, C_BLACK);
    lastx = x; lasty = y;
  }
}

void drawMapScreen() {
  ensureCanvas();
  canvas.fillSprite(C_WHITE);
  drawHeader("OSCARLOCATOR");

  setupProjection(gProj);
  Proj& P = gProj;
  drawGraticule(P);

  // Ground-track arc (the OSCARLOCATOR path arc)
  drawTrack(P, arcLat, arcLon, arcN, C_BLACK);

  // QTH range circle (amber): footprint radius at MEAN altitude, centred on the
  // station; inflated 6.5% on polar maps as the site does.
  {
    double reticleDeg = footprintDeg(meanAltKm());
    if (P.rm != RM_QTH) reticleDeg *= 1.065;
    drawGeoCircle(P, qthLat, qthLon, reticleDeg, C_DGRAY, false);
  }

  // Satellite footprint (green, dashed) at instantaneous altitude.
  if (liveValid) {
    drawGeoCircle(P, subLat, subLon, footprintDeg(subAltKm), C_GRAY, true);
  }

  // QTH marker: star on polar maps (off-centre), cross on the QTH map (centre).
  {
    int px, py;
    if (project(P, qthLat, qthLon, px, py)) {
      if (P.rm != RM_QTH) {
        drawStar(px, py, 9, 4);
      } else {
        canvas.drawLine(px - 7, py, px + 7, py, C_BLACK);
        canvas.drawLine(px, py - 7, px, py + 7, C_BLACK);
        canvas.drawCircle(px, py, 4, C_BLACK);
      }
    }
  }

  // Live sub-satellite dot
  int dotX = -1, dotY = -1; bool dotOn = false;
  if (liveValid) {
    int px, py;
    if (project(P, subLat, subLon, px, py)) {
      canvas.fillCircle(px, py, 7, C_BLACK);
      canvas.drawCircle(px, py, 9, C_WHITE);
      canvas.drawCircle(px, py, 10, C_BLACK);
      dotX = px; dotY = py; dotOn = true;
    }
  }

  // Readout panel (right third)
  int rx = 580, ry = 80;
  canvas.drawFastVLine(rx - 16, 64, SCREEN_H - 130, C_LGRAY);
  canvas.setFont(&fonts::FreeSansBold12pt7b);
  canvas.setTextColor(C_BLACK, C_WHITE);
  canvas.drawString(viewName(viewMode), rx, ry); ry += 36;

  canvas.setFont(&fonts::FreeSans9pt7b);
  char line[64];
  if (liveValid) {
    snprintf(line, sizeof(line), "Sub-sat: %6.2f, %7.2f", subLat, subLon);
    canvas.drawString(line, rx, ry); ry += 26;
    snprintf(line, sizeof(line), "Alt: %.0f km", subAltKm);
    canvas.drawString(line, rx, ry); ry += 26;
    snprintf(line, sizeof(line), "Az/El: %.1f / %.1f", liveAz, liveEl);
    canvas.drawString(line, rx, ry); ry += 26;
    snprintf(line, sizeof(line), "Range: %.0f km", liveRangeKm);
    canvas.drawString(line, rx, ry); ry += 26;
    snprintf(line, sizeof(line), "Visible: %s", liveEl > 0 ? "YES" : "no");
    canvas.drawString(line, rx, ry); ry += 30;
  } else {
    canvas.drawString("Acquiring elements / time...", rx, ry); ry += 30;
  }
  canvas.setTextColor(C_DGRAY, C_WHITE);
  canvas.drawString("Live display - azimuthal", rx, ry); ry += 22;
  canvas.drawString("equidistant projection", rx, ry); ry += 30;
  canvas.setTextColor(C_BLACK, C_WHITE);
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.drawString("Paul Stoetzer, N8HM", rx, SCREEN_H - 92);
  canvas.drawString("oscarlocator.n8hm.radio", rx, SCREEN_H - 70);

  drawTabBar();

  // Flush. On a full redraw (screen/view change or de-ghost) push everything
  // with the quality waveform. Otherwise update only the dynamic regions with
  // the fast waveform: header clock, readout panel, and a box covering the
  // dot's old + new positions (the static base map is left untouched).
  if (fullRedrawNeeded) {
    flushFull();
  } else {
    flushPartial(RECT_HEADER_X, RECT_HEADER_Y, RECT_HEADER_W, RECT_HEADER_H);
    flushPartial(RECT_READOUT_X, RECT_READOUT_Y, RECT_READOUT_W, RECT_READOUT_H);
    // dot bounding box spanning previous + current positions
    int pad = 14;
    if (dotOn || havePrevDot) {
      int xs[4], ys[4]; int n = 0;
      if (havePrevDot) { xs[n] = prevDotX; ys[n] = prevDotY; n++; }
      if (dotOn)       { xs[n] = dotX;     ys[n] = dotY;     n++; }
      int minx = xs[0], maxx = xs[0], miny = ys[0], maxy = ys[0];
      for (int i = 1; i < n; i++) {
        if (xs[i] < minx) minx = xs[i];
        if (xs[i] > maxx) maxx = xs[i];
        if (ys[i] < miny) miny = ys[i];
        if (ys[i] > maxy) maxy = ys[i];
      }
      flushPartial(minx - pad, miny - pad,
                   (maxx - minx) + 2 * pad, (maxy - miny) + 2 * pad);
    }
  }
  prevDotX = dotX; prevDotY = dotY; havePrevDot = dotOn;
}

void fmtClock(double unix, char* out, int n) {
  time_t t = (time_t)llround(unix);
  struct tm tmv; gmtime_r(&t, &tmv);
  snprintf(out, n, "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

void drawPassesScreen() {
  ensureCanvas();
  canvas.fillSprite(C_WHITE);
  drawHeader("Next passes");

  int x = 24, y = 80;
  canvas.setFont(&fonts::FreeSansBold12pt7b);
  canvas.setTextColor(C_BLACK, C_WHITE);
  char hdr[96];
  snprintf(hdr, sizeof(hdr), "%s over %.2f, %.2f  (min el %.0f deg)",
           selName, qthLat, qthLon, minPassEl);
  canvas.drawString(hdr, x, y); y += 38;

  // column headers
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.drawString("AOS (UTC)", x,        y);
  canvas.drawString("Peak",      x + 230,  y);
  canvas.drawString("LOS",       x + 380,  y);
  canvas.drawString("Dur",       x + 530,  y);
  canvas.drawString("Max El",    x + 660,  y);
  y += 8;
  canvas.drawFastHLine(x, y + 16, SCREEN_W - 2 * x, C_BLACK);
  y += 26;

  canvas.setFont(&fonts::FreeSans9pt7b);
  if (passCount == 0) {
    canvas.drawString("No passes computed yet -- load a satellite / wait for time sync.",
                      x, y);
  }
  for (int i = 0; i < passCount; i++) {
    char a[16], p[16], l[16];
    fmtClock(passes[i].aos,  a, sizeof(a));
    fmtClock(passes[i].peak, p, sizeof(p));
    fmtClock(passes[i].los,  l, sizeof(l));
    int durMin = (int)lround((passes[i].los - passes[i].aos) / 60.0);
    char buf[16];
    // AOS w/ date
    time_t at = (time_t)passes[i].aos; struct tm tmv; gmtime_r(&at, &tmv);
    char ad[28];
    snprintf(ad, sizeof(ad), "%02d/%02d %s", tmv.tm_mon + 1, tmv.tm_mday, a);
    canvas.drawString(ad, x, y);
    canvas.drawString(p, x + 230, y);
    canvas.drawString(l, x + 380, y);
    snprintf(buf, sizeof(buf), "%d min", durMin);
    canvas.drawString(buf, x + 530, y);
    snprintf(buf, sizeof(buf), "%.1f", passes[i].maxEl);
    canvas.drawString(buf, x + 660, y);
    y += 32;
    if (y > SCREEN_H - 90) break;
  }

  drawTabBar();
  // The table body only changes on recompute (which forces a full redraw);
  // per-tick, only the header clock changes, so flush just the header strip.
  if (fullRedrawNeeded) flushFull();
  else flushPartial(RECT_HEADER_X, RECT_HEADER_Y, RECT_HEADER_W, RECT_HEADER_H);
}

void drawRefScreen() {
  ensureCanvas();
  canvas.fillSprite(C_WHITE);
  drawHeader("Reference orbits");

  int x = 24, y = 80;
  canvas.setFont(&fonts::FreeSansBold12pt7b);
  canvas.setTextColor(C_BLACK, C_WHITE);
  char hdr[96];
  bool asc = (qthLat >= 0.0);
  snprintf(hdr, sizeof(hdr), "%s - first %s crossing of each UTC day",
           selName, asc ? "ascending" : "descending");
  canvas.drawString(hdr, x, y); y += 38;

  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.drawString("UTC date", x,       y);
  canvas.drawString(asc ? "Asc. UTC" : "Desc. UTC", x + 300, y);
  canvas.drawString("Longitude", x + 560, y);
  y += 8;
  canvas.drawFastHLine(x, y + 16, SCREEN_W - 2 * x, C_BLACK);
  y += 26;

  canvas.setFont(&fonts::FreeSans9pt7b);
  if (refCount == 0) {
    canvas.drawString("Building table -- load a satellite / wait for time sync.",
                      x, y);
  }
  for (int i = 0; i < refCount; i++) {
    long ymd = refOrbits[i].ymd;
    int yy = ymd / 10000, mm = (ymd / 100) % 100, dd = ymd % 100;
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d", yy, mm, dd);
    canvas.drawString(buf, x, y);
    int hh = (int)refOrbits[i].utc;
    int mn = (int)lround((refOrbits[i].utc - hh) * 60.0);
    if (mn == 60) { mn = 0; hh++; }
    snprintf(buf, sizeof(buf), "%02d:%02d UTC", hh, mn);
    canvas.drawString(buf, x + 300, y);
    // OSCARLOCATOR boards label longitude West-positive
    double lonW = -refOrbits[i].lon;
    snprintf(buf, sizeof(buf), "%.1f%s", fabs(lonW), lonW >= 0 ? " W" : " E");
    canvas.drawString(buf, x + 560, y);
    y += 30;
    if (y > SCREEN_H - 90) break;
  }

  drawTabBar();
  if (fullRedrawNeeded) flushFull();
  else flushPartial(RECT_HEADER_X, RECT_HEADER_Y, RECT_HEADER_W, RECT_HEADER_H);
}

void redraw() {
  switch (screen) {
    case SCR_MAP:       drawMapScreen();   break;
    case SCR_PASSES:    drawPassesScreen();break;
    case SCR_REFORBITS: drawRefScreen();   break;
  }
}

// ===========================================================================
//  CAPTIVE CONFIG WEB PAGE
//  Brought up on demand (button or first boot w/o WiFi). Serves a single page
//  to set satellite, QTH (grid or lat/lon), and trigger a GP refresh. WiFi
//  credentials themselves are handled by WiFiManager's own portal.
// ===========================================================================
String htmlConfigPage() {
  String s;
  s += "<!doctype html><html><head><meta name=viewport "
       "content='width=device-width,initial-scale=1'>"
       "<title>OSCARLOCATOR for PaperS3</title>"
       "<style>body{font-family:system-ui,Arial;margin:0;background:#f4f4f4;color:#111}"
       "header{background:#111;color:#fff;padding:16px 20px}"
       "main{max-width:640px;margin:0 auto;padding:20px}"
       "label{display:block;margin:14px 0 4px;font-weight:600}"
       "input,select{width:100%;padding:10px;font-size:16px;box-sizing:border-box;"
       "border:1px solid #bbb;border-radius:8px}"
       "button{margin-top:18px;padding:12px 18px;font-size:16px;border:0;"
       "border-radius:8px;background:#111;color:#fff;cursor:pointer}"
       ".row{display:flex;gap:12px}.row>div{flex:1}"
       "small{color:#555}</style></head><body>";
  s += "<header><b>OSCARLOCATOR</b> for PaperS3 &mdash; configuration</header><main>";
  s += "<form method='POST' action='/save'>";

  s += "<label>Satellite (name as in AMSAT GP)</label>";
  s += "<input name='sname' value='" + String(selName) + "'>";
  s += "<label>NORAD catalog number</label>";
  s += "<input name='norad' value='" + String(selNorad) + "'>";
  s += "<small>Tip: set the NORAD number for an exact match; the name is for display.</small>";

  s += "<label>Maidenhead grid (overrides lat/lon if 4/6 chars)</label>";
  s += "<input name='grid' placeholder='e.g. FM18lw'>";

  s += "<div class='row'><div><label>QTH latitude</label>"
       "<input name='lat' value='" + String(qthLat, 4) + "'></div>";
  s += "<div><label>QTH longitude</label>"
       "<input name='lon' value='" + String(qthLon, 4) + "'></div></div>";

  s += "<label>Min pass elevation (deg)</label>";
  s += "<input name='minel' value='" + String(minPassEl, 0) + "'>";
  s += "<label>Reference-orbit days</label>";
  s += "<input name='refdays' value='" + String(refDays) + "'>";

  s += "<label>Default view</label><select name='view'>";
  const char* vn[] = {"Polar - auto N/S", "Polar - North",
                      "Polar - South", "QTH-centred"};
  for (int i = 0; i < 4; i++) {
    s += "<option value='" + String(i) + "'";
    if ((int)viewMode == i) s += " selected";
    s += ">" + String(vn[i]) + "</option>";
  }
  s += "</select>";

  s += "<button type='submit'>Save</button> ";
  s += "</form>";
  s += "<form method='POST' action='/refresh'>"
       "<button type='submit'>Fetch AMSAT GP elements now</button></form>";
  s += "<form method='POST' action='/wifi'>"
       "<button type='submit'>Reconfigure WiFi (captive portal)</button></form>";
  s += "<p><small>Status: " + statusMsg + "<br>"
       "Satellites loaded: " + String(satCount) + "</small></p>";
  s += "<p><small>Live OSCARLOCATOR display. Credit: Paul Stoetzer, N8HM "
       "&mdash; oscarlocator.n8hm.radio</small></p>";
  s += "</main></body></html>";
  return s;
}

bool webRunning = false;
bool wantWifiPortal = false;

void handleRoot()    { server.send(200, "text/html", htmlConfigPage()); }
void handleSave() {
  if (server.hasArg("sname")) {
    String n = server.arg("sname"); n.toCharArray(selName, sizeof(selName));
  }
  if (server.hasArg("norad")) selNorad = server.arg("norad").toInt();
  if (server.hasArg("grid")) {
    double la, lo;
    if (gridToLatLon(server.arg("grid"), la, lo)) { qthLat = la; qthLon = lo; }
  }
  if (server.hasArg("lat") && server.arg("lat").length())
    qthLat = server.arg("lat").toFloat();
  if (server.hasArg("lon") && server.arg("lon").length())
    qthLon = server.arg("lon").toFloat();
  if (server.hasArg("minel")) minPassEl = server.arg("minel").toFloat();
  if (server.hasArg("refdays")) refDays = server.arg("refdays").toInt();
  if (server.hasArg("view")) viewMode = (ViewMode)server.arg("view").toInt();
  saveConfig();
  selectSatByNorad(selNorad);
  loadSelectedIntoSat();
  statusMsg = "Saved. Recomputing...";
  server.sendHeader("Location", "/");
  server.send(303);
  // recompute on next loop tick
  lastTickMs = 0;
}
void handleRefresh() {
  statusMsg = "Fetching AMSAT GP...";
  bool ok = fetchGP();
  selectSatByNorad(selNorad);
  loadSelectedIntoSat();
  statusMsg = ok ? "GP refreshed." : "GP fetch failed (using cache).";
  server.sendHeader("Location", "/");
  server.send(303);
  lastTickMs = 0;
}
void handleWifi() {
  wantWifiPortal = true;
  server.send(200, "text/html",
              "<html><body><h3>Starting WiFi portal...</h3>"
              "<p>Join the OSCARLOCATOR-Setup network.</p></body></html>");
}
void handleNotFound() {
  // captive-portal redirect
  server.sendHeader("Location", String("http://") +
                    WiFi.softAPIP().toString());
  server.send(302, "text/plain", "");
}

void startWeb() {
  if (webRunning) return;
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/refresh", HTTP_POST, handleRefresh);
  server.on("/wifi", HTTP_POST, handleWifi);
  server.onNotFound(handleNotFound);
  server.begin();
  webRunning = true;
}

// ===========================================================================
//  TOUCH HANDLING
// ===========================================================================
bool hit(const Btn& b, int x, int y) {
  return x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h;
}

void cycleView() {
  viewMode = (ViewMode)(((int)viewMode + 1) % 4);
  saveConfig();
  buildGroundArc();
  havePrevDot = false;            // projection changed; old dot box is invalid
  fullRedrawNeeded = true;        // repaint the whole base map cleanly
  redraw();
}

// Simple on-device satellite picker overlay (paged list).
int satPickPage = 0;
bool inSatPicker = false;

void drawSatPicker() {
  ensureCanvas();
  canvas.fillSprite(C_WHITE);
  drawHeader("Select satellite");
  int perPage = 10;
  int start = satPickPage * perPage;
  int x = 24, y = 80;
  canvas.setFont(&fonts::FreeSans12pt7b);
  for (int i = 0; i < perPage; i++) {
    int idx = start + i;
    if (idx >= satCount) break;
    int by = 80 + i * 40;
    canvas.drawRoundRect(x, by, SCREEN_W - 2 * x - 220, 34, 6, C_BLACK);
    char buf[40];
    snprintf(buf, sizeof(buf), "%s  (%ld)", satList[idx].name, satList[idx].norad);
    canvas.setTextDatum(middle_left);
    canvas.drawString(buf, x + 12, by + 17);
    canvas.setTextDatum(top_left);
  }
  // Prev/Next/Close
  Btn prev = {SCREEN_W - 210, 80,  90, 44, "Prev"};
  Btn next = {SCREEN_W - 110, 80,  90, 44, "Next"};
  Btn close= {SCREEN_W - 210, 140, 190,44, "Close"};
  drawButton(prev, false); drawButton(next, false); drawButton(close, false);
  fullRedrawNeeded = true;        // overlay: force a clean full refresh
  flushFull();
}

void handleSatPickerTouch(int x, int y) {
  int perPage = 10;
  Btn prev = {SCREEN_W - 210, 80,  90, 44, "Prev"};
  Btn next = {SCREEN_W - 110, 80,  90, 44, "Next"};
  Btn close= {SCREEN_W - 210, 140, 190,44, "Close"};
  if (hit(prev, x, y)) { if (satPickPage > 0) satPickPage--; drawSatPicker(); return; }
  if (hit(next, x, y)) {
    if ((satPickPage + 1) * perPage < satCount) satPickPage++;
    drawSatPicker(); return;
  }
  if (hit(close, x, y)) { inSatPicker = false; fullRedrawNeeded = true; redraw(); return; }
  int start = satPickPage * perPage;
  for (int i = 0; i < perPage; i++) {
    int idx = start + i;
    if (idx >= satCount) break;
    int by = 80 + i * 40;
    if (x >= 24 && x <= SCREEN_W - 24 - 220 && y >= by && y <= by + 34) {
      selNorad = satList[idx].norad;
      String(satList[idx].name).toCharArray(selName, sizeof(selName));
      selectSatByNorad(selNorad);
      loadSelectedIntoSat();
      saveConfig();
      inSatPicker = false;
      lastTickMs = 0;           // force recompute
      havePrevDot = false;
      fullRedrawNeeded = true;
      statusMsg = "Selected " + String(selName);
      redraw();
      return;
    }
  }
}

void handleTouch(int x, int y) {
  if (inSatPicker) { handleSatPickerTouch(x, y); return; }

  if (hit(tabMap, x, y))  { screen = SCR_MAP;       fullRedrawNeeded = true; havePrevDot = false; redraw(); return; }
  if (hit(tabPass, x, y)) { screen = SCR_PASSES;    fullRedrawNeeded = true; redraw(); return; }
  if (hit(tabRef, x, y))  { screen = SCR_REFORBITS; fullRedrawNeeded = true; redraw(); return; }
  if (hit(btnRefr, x, y)) { lastTickMs = 0; fullRedrawNeeded = true; statusMsg="Refreshing..."; return; }

  if (screen == SCR_MAP) {
    if (hit(btnView, x, y)) { cycleView(); return; }
  } else {
    if (hit(btnSat, x, y)) { inSatPicker = true; satPickPage = 0; drawSatPicker(); return; }
  }
}

// ===========================================================================
//  COMPUTE TICK
// ===========================================================================
// Light update every tick: live position + ground arc only.
void recomputeLive() {
  loadSelectedIntoSat();
  updateLivePosition();
  buildGroundArc();
}

// Heavy update: pass list + reference orbits. Run on change or hourly.
void recomputeTables() {
  predictPasses();
  buildReferenceOrbits();
}

void recomputeAll() {
  recomputeLive();
  recomputeTables();
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
void redrawBootMsg(const char* msg) {
  ensureCanvas();
  canvas.fillSprite(C_WHITE);
  canvas.setTextColor(C_BLACK, C_WHITE);
  canvas.setFont(&fonts::FreeSansBold18pt7b);
  canvas.setTextDatum(top_left);
  canvas.drawString("OSCARLOCATOR for PaperS3", 40, 60);
  canvas.setFont(&fonts::FreeSans12pt7b);
  canvas.drawString("Paul Stoetzer, N8HM  -  oscarlocator.n8hm.radio", 40, 110);
  canvas.drawFastHLine(40, 150, SCREEN_W - 80, C_BLACK);
  canvas.drawString(msg, 40, 190);
  fullRedrawNeeded = true;
  flushFull();
}

void connectWiFiAndTime() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  statusMsg = "Connecting WiFi...";
  redrawBootMsg("Connecting WiFi (portal: OSCARLOCATOR-Setup)...");
  bool ok = wm.autoConnect(CONFIG_AP_SSID);
  wifiOK = ok && (WiFi.status() == WL_CONNECTED);
  if (wifiOK) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm t;
    redrawBootMsg("Syncing time (NTP)...");
    for (int i = 0; i < 20 && !getLocalTime(&t, 500); i++) delay(250);
    time_t now = time(nullptr);
    timeOK = now > 1700000000;
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);                 // landscape 960x540
  M5.Display.setEpdMode(epd_quality);        // start clean; flushes set mode per refresh
  M5.Display.fillScreen(C_WHITE);

  if (!LittleFS.begin(true)) {
    // format on first run if needed
    LittleFS.begin(true);
  }

  loadConfig();
  redrawBootMsg("Starting...");

  connectWiFiAndTime();

  // Load elements: try cache first, fetch if stale or missing.
  bool haveCache = loadCachedGP();
  bool needFetch = !haveCache;
  if (haveCache && timeOK) {
    time_t now = time(nullptr);
    if (lastTLEEpoch == 0 || (now - lastTLEEpoch) > 86400) needFetch = true;
  }
  if (needFetch && wifiOK) {
    redrawBootMsg("Fetching AMSAT GP elements...");
    if (fetchGP()) statusMsg = "GP loaded.";
    else { statusMsg = "GP fetch failed; using cache."; loadCachedGP(); }
  } else {
    statusMsg = haveCache ? "Using cached GP." : "No GP data yet.";
  }

  selectSatByNorad(selNorad);
  recomputeAll();

  startWeb();
  redraw();
  lastTickMs = millis();
}

void loop() {
  M5.update();
  if (webRunning) server.handleClient();

  // On-demand WiFi portal request from the web page
  if (wantWifiPortal) {
    wantWifiPortal = false;
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    redrawBootMsg("WiFi portal active: join OSCARLOCATOR-Setup");
    wm.startConfigPortal(CONFIG_AP_SSID);
    wifiOK = (WiFi.status() == WL_CONNECTED);
    redraw();
  }

  // Touch
  auto td = M5.Touch.getDetail();
  static bool wasDown = false;
  if (td.isPressed() && !wasDown) {
    wasDown = true;
    handleTouch(td.x, td.y);
  } else if (!td.isPressed()) {
    wasDown = false;
  }

  // Periodic recompute / redraw
  unsigned long now = millis();
  unsigned long interval = (liveValid && liveEl > 0) ? TICK_VISIBLE_MS
                                                     : TICK_HIDDEN_MS;
  if (lastTickMs == 0 || (now - lastTickMs) >= interval) {
    bool forced = (lastTickMs == 0);
    lastTickMs = now;
    if (!timeOK) {
      time_t t = time(nullptr);
      timeOK = t > 1700000000;
    }
    recomputeLive();
    // Recompute the heavy tables on forced refresh (sat/QTH change, button)
    // or about once an hour; otherwise just refresh the live geometry.
    static unsigned long lastTablesMs = 0;
    if (forced || lastTablesMs == 0 || (now - lastTablesMs) >= 3600000UL) {
      recomputeTables();
      lastTablesMs = now;
      fullRedrawNeeded = true;     // table contents changed -> clean full paint
    }
    if (forced) fullRedrawNeeded = true;
    if (!inSatPicker) redraw();
  }

  delay(10);
}
