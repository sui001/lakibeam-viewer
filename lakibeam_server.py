"""
Richbeam LakiBeam 1 LIDAR - live room scanner.

Reads the sensor's UDP point stream and serves a browser view of the room.
Cross platform: Windows, macOS, Linux. The only dependency is Flask.

    python lakibeam_server.py
    then open http://localhost:5002

The sensor ships with the laser and rotor OFF, and those settings persist to
its EEPROM, so a unit that was left disabled stays disabled through a power
cycle. This server enables both on startup over the HTTP config API.

Packet format, 1206 bytes:
    12 x sub_packet (100 bytes) + uint32 timestamp + uint16 factory
    sub_packet: uint16 header (0xEEFF) + uint16 azimuth (0.01 deg)
                + 16 x point (uint16 dist_mm, uint8 rssi, uint16 dist, uint8 rssi)
    dist == 0 means no return. azimuth == 0xFFFF means the sub-packet is invalid.
"""

import argparse
import queue
import socket
import struct
import threading
import urllib.parse
import urllib.request

from flask import Flask, Response

app = Flask(__name__)

UDP_BLOCKS = 12
BLOCK_POINTS = 16
SUB_PKT_SIZE = 100

scan_queue = queue.Queue(maxsize=10)
CFG = {"lidar_ip": "192.168.198.2", "udp_port": 2368}


# ── Sensor configuration over HTTP ────────────────────────────────────────────

def _post(method):
    """Send one command to the sensor's config CGI.

    The Referer header is required. Without it the request hangs until it times
    out rather than returning an error, which is a confusing failure to debug.
    """
    base = "http://%s" % CFG["lidar_ip"]
    req = urllib.request.Request(
        base + "/cgi-bin/config.php",
        data=urllib.parse.urlencode({"method": method}).encode(),
        headers={
            "Content-Type": "application/x-www-form-urlencoded",
            "Referer": base + "/config.html",
        },
    )
    with urllib.request.urlopen(req, timeout=8) as r:
        return r.read()


def configure_sensor():
    """Enable the laser and set the rotor to its fastest scan rate.

    Both are needed. Enabling the laser alone leaves rpm at 0 and no points are
    produced, which looks exactly like a dead sensor.
    """
    for label, method in (("scan rate", "-freq 5"), ("laser", "-en 1")):
        try:
            _post(method)
            print("  %-10s enabled (%s)" % (label, method))
        except Exception as exc:
            print("  %-10s FAILED: %s" % (label, exc))
            print("             The sensor's web server becomes slow and")
            print("             unreliable while it is scanning. If it is")
            print("             already running this failure is harmless.")


# ── Packet decoding ───────────────────────────────────────────────────────────

def decode_packet(data):
    """Return [(angle_deg, distance_mm), ...] for the valid returns in a packet."""
    if len(data) < 1206:
        return []

    sub = []
    for i in range(UDP_BLOCKS):
        off = i * SUB_PKT_SIZE
        azimuth = struct.unpack_from("<H", data, off + 2)[0]
        sub.append((azimuth, data[off + 4:off + SUB_PKT_SIZE]))

    out = []
    for i in range(UDP_BLOCKS - 1):
        start = sub[i][0]
        if start == 0xFFFF:
            continue
        start %= 36000
        end = sub[i + 1][0] % 36000
        # Azimuth is reported once per block; interpolate across the 16 points.
        span = (end - start) if end >= start else (36000 - start + end)
        step = span / BLOCK_POINTS
        block = sub[i][1]
        for j in range(BLOCK_POINTS):
            dist = struct.unpack_from("<H", block, j * 6)[0]
            if dist > 0:
                out.append((round(((start + j * step) % 36000) * 0.01, 2), dist))
    return out


def udp_listener():
    print("Configuring sensor at %s ..." % CFG["lidar_ip"])
    configure_sensor()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CFG["udp_port"]))
    sock.settimeout(1.0)
    print("Listening for scan data on UDP %d" % CFG["udp_port"])

    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        points = decode_packet(data)
        if points:
            try:
                scan_queue.put_nowait(points)
            except queue.Full:
                pass          # viewer is behind; drop rather than lag


# ── Web ───────────────────────────────────────────────────────────────────────

def event_stream():
    """Server-sent events: 'angle,dist;angle,dist;...' per frame."""
    while True:
        try:
            points = scan_queue.get(timeout=2.0)
            yield "data: %s\n\n" % ";".join("%s,%d" % (a, d) for a, d in points)
        except queue.Empty:
            yield "data: \n\n"


@app.route("/scan")
def scan():
    return Response(event_stream(), mimetype="text/event-stream")


@app.route("/")
def index():
    return HTML


HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LakiBeam - Room Scan</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #101010; color: #fff; font-family: 'Courier New', monospace;
       display: flex; flex-direction: column; align-items: center;
       gap: 10px; padding: 14px; }
h1 { font-size: 0.9rem; letter-spacing: 0.2em; text-transform: uppercase; }
h1 span { color: #ff0; }
#status { font-size: 0.85rem; color: #4dff88; min-height: 1.2rem; font-weight: bold; }
canvas { border: 2px solid #666; background: #000; display: block; }
#ctl { display: flex; flex-wrap: wrap; gap: 14px; align-items: center;
       font-size: 0.85rem; justify-content: center; max-width: 820px; }
.c { display: flex; align-items: center; gap: 6px; }
.c b { color: #ff0; min-width: 42px; display: inline-block; }
input[type=range] { accent-color: #4dff88; width: 100px; }
button { background: #333; color: #fff; border: 2px solid #777; padding: 5px 14px;
         cursor: pointer; font-family: inherit; font-size: 0.85rem; font-weight: bold; }
button:hover { background: #4a4a4a; }
#leg { display: flex; gap: 18px; font-size: 0.8rem; flex-wrap: wrap; justify-content: center; }
.sw { display: inline-block; width: 12px; height: 12px; vertical-align: middle; margin-right: 4px; }
</style>
</head>
<body>
<h1>Room Scan <span>fixed sensor</span></h1>
<div id="status">connecting...</div>
<canvas id="c"></canvas>

<div id="ctl">
  <div class="c">zoom <input type="range" id="sz" min="4" max="60" value="18" step="2"><b id="vz">18</b></div>
  <div class="c">rotate <input type="range" id="sr" min="0" max="359" value="0"><b id="vr">0&deg;</b></div>
  <div class="c">solid after <input type="range" id="st" min="1" max="40" value="4"><b id="vt">4</b></div>
  <div class="c">max range <input type="range" id="sm" min="1000" max="20000" value="14000" step="500"><b id="vm">14m</b></div>
  <label class="c"><input type="checkbox" id="cf" checked>&nbsp;auto fit</label>
  <label class="c"><input type="checkbox" id="cl" checked>&nbsp;live sweep</label>
  <button onclick="clearMap()">Clear</button>
</div>

<div id="leg">
  <span><span class="sw" style="background:#4dff88"></span>room (accumulated)</span>
  <span><span class="sw" style="background:#ff9500"></span>live sweep</span>
  <span><span class="sw" style="background:#4da6ff"></span>sensor</span>
  <span style="color:#ff0">grid = 1m</span>
</div>

<script>
const SZ = Math.min(window.innerWidth - 40, 760);
const cv = document.getElementById('c');
cv.width = cv.height = SZ;
const ctx = cv.getContext('2d');

// Accumulated room: "gx,gy" -> hit count. Never evicted. The sensor is fixed
// and a room is bounded, so this converges rather than growing without limit.
// Evicting cells is what makes an accumulating map churn and look like drift.
const room = new Map();
let live = [], frames = 0, pkts = 0;

let ZOOM = 18, ROT = 0, THRESH = 4, MAXD = 14000, FIT = true, SHOWLIVE = true;
const GRID = 50;   // mm per cell

const $ = id => document.getElementById(id);
$('sz').oninput = e => { ZOOM = +e.target.value; $('vz').textContent = ZOOM; draw(); };
$('sr').oninput = e => { ROT = +e.target.value; $('vr').textContent = ROT + '\\u00b0'; draw(); };
$('st').oninput = e => { THRESH = +e.target.value; $('vt').textContent = THRESH; draw(); };
$('sm').oninput = e => { MAXD = +e.target.value; $('vm').textContent = (MAXD/1000).toFixed(0)+'m'; };
$('cf').onchange = e => { FIT = e.target.checked; draw(); };
$('cl').onchange = e => { SHOWLIVE = e.target.checked; draw(); };
function clearMap() { room.clear(); frames = 0; draw(); }

function toXY(angDeg, dist) {
  const r = angDeg * Math.PI / 180;
  return [Math.sin(r) * dist, -Math.cos(r) * dist];
}

let scale = 1, ox = 0, oy = 0;
function computeFit() {
  if (!FIT || room.size === 0) { scale = ZOOM / 1000; ox = 0; oy = 0; return; }
  let mnx = 1e9, mny = 1e9, mxx = -1e9, mxy = -1e9;
  room.forEach((n, k) => {
    if (n < THRESH) return;
    const p = k.split(','), x = +p[0]*GRID, y = +p[1]*GRID;
    if (x < mnx) mnx = x; if (x > mxx) mxx = x;
    if (y < mny) mny = y; if (y > mxy) mxy = y;
  });
  if (mnx > mxx) { scale = ZOOM / 1000; ox = 0; oy = 0; return; }
  mnx = Math.min(mnx, 0); mxx = Math.max(mxx, 0);
  mny = Math.min(mny, 0); mxy = Math.max(mxy, 0);
  const w = (mxx - mnx) || 1000, h = (mxy - mny) || 1000;
  scale = Math.min(SZ * 0.88 / w, SZ * 0.88 / h);
  ox = (mnx + mxx) / 2; oy = (mny + mxy) / 2;
}

function px(x, y) {
  const r = ROT * Math.PI / 180, dx = x - ox, dy = y - oy;
  return [SZ/2 + (dx*Math.cos(r) - dy*Math.sin(r)) * scale,
          SZ/2 + (dx*Math.sin(r) + dy*Math.cos(r)) * scale];
}

function grid() {
  const step = 1000 * scale;
  if (step > 6) {
    ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1;
    const sx = ((SZ/2 - ox*scale) % step + step) % step;
    const sy = ((SZ/2 - oy*scale) % step + step) % step;
    for (let x = sx; x < SZ; x += step) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,SZ); ctx.stroke(); }
    for (let y = sy; y < SZ; y += step) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(SZ,y); ctx.stroke(); }
  }
  const [cx0, cy0] = px(0, 0);
  ctx.strokeStyle = '#3a3a3a'; ctx.setLineDash([4,4]);
  for (let m = 2; m <= 20; m += 2) {
    const rr = m * 1000 * scale;
    if (rr > SZ) break;
    ctx.beginPath(); ctx.arc(cx0, cy0, rr, 0, Math.PI*2); ctx.stroke();
  }
  ctx.setLineDash([]);
}

function draw() {
  computeFit();
  ctx.fillStyle = '#000'; ctx.fillRect(0,0,SZ,SZ);
  grid();

  const cs = Math.max(2, GRID * scale);
  room.forEach((n, k) => {
    const p = k.split(','), x = +p[0]*GRID, y = +p[1]*GRID;
    const [a,b] = px(x,y);
    if (a < -cs || a > SZ+cs || b < -cs || b > SZ+cs) return;
    if (n >= THRESH) {
      const t = Math.min(1, n / (THRESH*4));
      ctx.fillStyle = 'rgb(' + Math.round(60+40*t) + ',255,' + Math.round(120+40*t) + ')';
    } else {
      ctx.fillStyle = 'rgba(77,255,136,' + (0.15 + 0.5*(n/THRESH)) + ')';
    }
    ctx.fillRect(a-cs/2, b-cs/2, cs, cs);
  });

  if (SHOWLIVE) {
    ctx.fillStyle = '#ff9500';
    for (const [ang,d] of live) {
      if (d <= 0 || d > MAXD) continue;
      const [x,y] = toXY(ang,d);
      const [a,b] = px(x,y);
      ctx.fillRect(a-1.5, b-1.5, 3, 3);
    }
  }

  const [sx0, sy0] = px(0,0);
  ctx.beginPath(); ctx.arc(sx0, sy0, 6, 0, Math.PI*2);
  ctx.fillStyle = '#4da6ff'; ctx.fill();
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();

  ctx.fillStyle = '#ff0'; ctx.font = 'bold 12px Courier New';
  ctx.fillText('cells ' + room.size + '   frames ' + frames, 10, 18);
  const bar = 1000 * scale;
  if (bar > 20 && bar < SZ - 40) {
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(14, SZ-16); ctx.lineTo(14+bar, SZ-16); ctx.stroke();
    ctx.fillText('1m', 14 + bar/2 - 9, SZ-22);
  }
}
draw();

const es = new EventSource('/scan');
const st = document.getElementById('status');
es.onmessage = e => {
  pkts++;
  const body = e.data.trim();
  if (!body) {
    st.textContent = 'waiting for scan  |  cells ' + room.size + '  |  pkt ' + pkts;
    return;
  }
  live = []; frames++;
  for (const s of body.split(';')) {
    const c = s.indexOf(',');
    if (c < 0) continue;
    const ang = parseFloat(s.slice(0,c)), d = parseInt(s.slice(c+1));
    if (!(d > 0) || d > MAXD) continue;
    live.push([ang,d]);
    const [x,y] = toXY(ang,d);
    const k = Math.round(x/GRID) + ',' + Math.round(y/GRID);
    room.set(k, (room.get(k) || 0) + 1);
  }
  draw();
  st.textContent = live.length + ' pts  |  ' + room.size + ' cells  |  frame ' + frames;
};
es.onerror = () => { st.textContent = 'disconnected'; st.style.color = '#ff5555'; };
</script>
</body>
</html>"""


def main():
    ap = argparse.ArgumentParser(description="LakiBeam 1 room scanner")
    ap.add_argument("--lidar-ip", default="192.168.198.2",
                    help="sensor address (default 192.168.198.2)")
    ap.add_argument("--udp-port", type=int, default=2368,
                    help="port the sensor sends to (default 2368)")
    ap.add_argument("--web-port", type=int, default=5002,
                    help="port to serve the viewer on (default 5002)")
    ap.add_argument("--no-configure", action="store_true",
                    help="skip laser and rotor setup; just listen")
    args = ap.parse_args()

    CFG["lidar_ip"] = args.lidar_ip
    CFG["udp_port"] = args.udp_port

    if args.no_configure:
        global configure_sensor
        configure_sensor = lambda: print("  skipped (--no-configure)")

    threading.Thread(target=udp_listener, daemon=True).start()
    print("\nOpen  http://localhost:%d\n" % args.web_port)
    app.run(host="0.0.0.0", port=args.web_port, threaded=True)


if __name__ == "__main__":
    main()
