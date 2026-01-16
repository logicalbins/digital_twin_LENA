#!/usr/bin/env python3
import subprocess
import json
import os
import re
import requests
from datetime import datetime
import traceback
import csv
from pathlib import Path

# ====== YOUR CONFIG ======
THIS_DIR = "/Users/ducpham/Documents/studies/5G_thesis/codes"
FAILURE_LOG_OUT = f"{THIS_DIR}/failures.log"
SERVER_URL = "http://qos.nextg.dice.aalto.fi:5000/data"  # (unused below; keep if later POST)
IMSI = "001011000000004"
CLIENT = "Client_1"

# Single CSV that stores EVERYTHING
CSV_FILE = f"{THIS_DIR}/1-4-frame.csv"

# Router RPC config
ROUTER_BASE = os.getenv("ROUTER_BASE", "http://192.168.1.1").rstrip("/")
RPC_URL = f"{ROUTER_BASE}/v1/rpc"
TIMEOUT_S = 8

# ---- Login creds (set env vars to override) ----
ROUTER_USERNAME = os.getenv("ROUTER_USERNAME", "superadmin")
ROUTER_PASSWORD = os.getenv("ROUTER_PASSWORD", "superadmin")
LOGIN_METHOD    = os.getenv("ROUTER_LOGIN_METHOD", "login")
# ------------------------------------------------

# Start with an empty token; we’ll obtain it via login()
COOKIES = {
    "token": "",
    "role": os.getenv("ROUTER_ROLE", "superadmin"),
    "username": ROUTER_USERNAME,
    "styleType": os.getenv("ROUTER_STYLETYPE", "pc"),
    "cfgStyleType": os.getenv("ROUTER_CFGSTYLETYPE", "auto"),
}

COMMON_HEADERS = {
    "Content-Type": "application/json; charset=UTF-8",
    "Accept": "application/json, text/plain, */*",
    "Origin": ROUTER_BASE,
    "Referer": ROUTER_BASE + "/",
    "Connection": "keep-alive",
}

# Try several Authorization header styles
AUTH_VARIANTS = [
    lambda t: {"Authorization": f"token={t}"} if t else {},
    lambda t: {"Authorization": t} if t else {},
    lambda t: {},  # cookie-only
]
# =========================

def log_error():
    now_str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    with open(FAILURE_LOG_OUT, 'a+') as f:
        f.write(f"{now_str}\n{traceback.format_exc()}\n")

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return str(e)

def extract_throughput(iperf_output_json, direction='uplink'):
    try:
        j = json.loads(iperf_output_json)
        if direction == 'uplink':
            throughput = j['end']['sum_sent']['bits_per_second'] / 1_000_000
        else:
            throughput = j['end']['sum_received']['bits_per_second'] / 1_000_000
        return round(throughput, 2)
    except Exception:
        log_error()
        print("Error parsing iperf3 output:")
        print(iperf_output_json)
        return None

def extract_avg_rtt(ping_output):
    m = re.search(r'round-trip min/avg/max/stddev = [\d.]+/([\d.]+)/[\d.]+/[\d.]+ ms', ping_output)
    return float(m.group(1)) if m else None

# -------- Router RPC helpers --------
def rpc_call(session: requests.Session, method: str, auth_headers: dict, params=None, _id=1):
    payload = {"jsonrpc": "2.0", "id": _id, "method": method}
    if params is not None:
        payload["params"] = params
    r = session.post(RPC_URL, cookies=COOKIES,
                     headers={**COMMON_HEADERS, **auth_headers},
                     data=json.dumps(payload), timeout=TIMEOUT_S)
    r.raise_for_status()
    # If server set/rotated the token cookie, keep it
    token_cookie = r.cookies.get("token")
    if token_cookie is not None and token_cookie != "":
        COOKIES["token"] = token_cookie
    return r.json()

def login_and_get_token(session: requests.Session) -> bool:
    """
    Perform JSON-RPC login and capture 'sid' from result.
    We reuse that sid as our 'token' cookie and/or Authorization header.
    """
    COOKIES["token"] = ""  # clear stale
    payload = {
        "jsonrpc": "2.0",
        "id": 1,
        "method": LOGIN_METHOD,  # typically "login"
        "params": {"username": ROUTER_USERNAME, "password": ROUTER_PASSWORD},
    }
    headers = dict(COMMON_HEADERS)  # no Authorization for login
    try:
        r = session.post(RPC_URL, cookies=COOKIES, headers=headers,
                         data=json.dumps(payload), timeout=TIMEOUT_S)
        r.raise_for_status()
        res = r.json()
    except Exception:
        log_error()
        return False

    # Extract sid from common shapes: {"result":{"sid":"..."}}
    sid = None
    if isinstance(res, dict) and "result" in res:
        sid = res["result"].get("sid")

    if sid:
        COOKIES["token"] = sid
        return True

    # Some firmwares also set sid/token as a cookie; try that too
    token_cookie = r.cookies.get("token")
    if token_cookie:
        COOKIES["token"] = token_cookie
        return True

    return False
# ----------------------------------------------

def walk(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield k, v
            yield from walk(v)
    elif isinstance(obj, list):
        for it in obj:
            yield from walk(it)

def extract_signal_metrics(obj):
    picks = {"RSRP": "", "RSRQ": "", "SINR": ""}
    cand = {"rsrp": [], "rsrq": [], "sinr": []}
    for k, v in walk(obj):
        if isinstance(k, str):
            kl = k.lower()
            if "rsrp" in kl: cand["rsrp"].append(str(v))
            if "rsrq" in kl: cand["rsrq"].append(str(v))
            if "sinr" in kl: cand["sinr"].append(str(v))
    def choose(key):
        return cand[key][0] if cand[key] else ""
    picks["RSRP"] = choose("rsrp")
    picks["RSRQ"] = choose("rsrq")
    picks["SINR"] = choose("sinr")
    return picks

def fetch_signal_after_login(session: requests.Session) -> dict:
    """
    Ensure we're logged in (get sid), then try a few auth header styles
    to fetch signal metrics.
    """
    if not COOKIES.get("token"):
        if not login_and_get_token(session):
            print("[signal] login failed: check ROUTER_USERNAME/ROUTER_PASSWORD/ROUTER_BASE.")
            return {"RSRP": "", "RSRQ": "", "SINR": ""}

    token = COOKIES.get("token", "")
    last_err = None

    for make_auth in AUTH_VARIANTS:
        auth_hdr = make_auth(token)
        try:
            # Optional: verify session is alive if router supports it
            try:
                res_alive = rpc_call(session, "alive", auth_hdr)
                if isinstance(res_alive, dict) and "error" in res_alive:
                    # not fatal; continue with metric calls
                    pass
            except Exception:
                # alive may not exist; ignore
                pass

            res = rpc_call(session, "mobileinfo", auth_hdr)
            if "error" in res:
                # some firmwares keep metrics under different method
                res2 = rpc_call(session, "link_status", auth_hdr)
                res = {"mobileinfo": res, "link_status": res2}
            metrics = extract_signal_metrics(res)
            if any(metrics.values()):
                return metrics
        except Exception as e:
            last_err = e
            continue

    print(f"[signal] fetch failed after login. Last error: {last_err}")
    return {"RSRP": "", "RSRQ": "", "SINR": ""}

def main():
    # --- throughput (UL/DL) ---
    ul_output = run_command("iperf3 -c 172.16.210.1 -p 5201 --json")
    throughput_ul = extract_throughput(ul_output, direction='uplink')

    dl_output = run_command("iperf3 -c 172.16.210.1 -p 5201 --json --reverse")
    throughput_dl = extract_throughput(dl_output, direction='downlink')

    if throughput_ul is None or throughput_dl is None:
        return

    # --- latency ---
    ping_output = run_command("ping -c 5 172.16.210.1")
    avg_rtt = extract_avg_rtt(ping_output)

    # --- Signal metrics (RSRP/RSRQ/SINR) using new login/token flow ---
    session = requests.Session()
    signal = fetch_signal_after_login(session)  # dict

    # --- write single CSV row ---
    timestamp = int(datetime.now().timestamp())
    # write header if new file
    write_header = not os.path.exists(CSV_FILE)
    with open(CSV_FILE, 'a', newline='') as f:
        w = csv.writer(f)
        if write_header:
            w.writerow([
                "timestamp",
                "throughput UL (Mbit/s)", "throughput DL (Mbit/s)",
                "latency (ms)",
                "RSRP", "RSRQ", "SINR"
            ])
        w.writerow([
            timestamp,
            throughput_ul, throughput_dl,
            avg_rtt,
            signal.get("RSRP", ""), signal.get("RSRQ", ""), signal.get("SINR", "")
        ])

    # For reference (and possible server POST later)
    data = {
        "client": CLIENT,
        "IMSI": IMSI,
        "timestamp": timestamp,
        "Throughput Sender(Mbit/s)": throughput_ul,
        "Throughput Receiver(Mbit/s)": throughput_dl,
        "Latency (ms)": avg_rtt,
        "RSRP": signal.get("RSRP", ""),
        "RSRQ": signal.get("RSRQ", ""),
        "SINR": signal.get("SINR", "")
    }
    print(data)

if __name__ == "__main__":
    main()
