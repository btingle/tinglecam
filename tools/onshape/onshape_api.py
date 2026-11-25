"""Minimal Onshape REST client using API keys from ~/.onshape_access and ~/.onshape_secret."""
import base64
import json
import sys
import urllib.error
import urllib.parse
import urllib.request
import socket
from pathlib import Path

BASE = "https://cad.onshape.com/api/v10"
_KEYS = (Path("~/.onshape_access").expanduser().read_text().strip(),
         Path("~/.onshape_secret").expanduser().read_text().strip())
# WSL's resolver is broken on this machine; resolve the API host over DNS-over-HTTPS.
def _doh(host):
    req = urllib.request.Request(f"https://1.1.1.1/dns-query?name={host}&type=A",
                                 headers={"Accept": "application/dns-json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return [a["data"] for a in json.load(r)["Answer"] if a["type"] == 1][0]


_HOST_IP = _doh("cad.onshape.com")
_getaddrinfo = socket.getaddrinfo
socket.getaddrinfo = lambda host, *a, **k: _getaddrinfo(_HOST_IP if host == "cad.onshape.com" else host, *a, **k)

_AUTH = "Basic " + base64.b64encode(":".join(_KEYS).encode()).decode()


def call(method, path, body=None, **params):
    url = BASE + path + ("?" + urllib.parse.urlencode(params) if params else "")
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={
        "Authorization": _AUTH, "Accept": "application/json", "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req) as r:
            text = r.read().decode()
    except urllib.error.HTTPError as e:
        raise SystemExit(f"{method} {path} -> {e.code}: {e.read().decode()[:3000]}")
    return json.loads(text) if text else None


def download(path):
    """GET raw bytes (exports, external data)."""
    req = urllib.request.Request(BASE + path, headers={"Authorization": _AUTH, "Accept": "application/octet-stream"})
    try:
        with urllib.request.urlopen(req) as r:
            return r.read()
    except urllib.error.HTTPError as e:
        raise SystemExit(f"GET {path} -> {e.code}: {e.read().decode()[:3000]}")


if __name__ == "__main__":
    method, path = sys.argv[1], sys.argv[2]
    body = json.loads(sys.argv[3]) if len(sys.argv) > 3 else None
    print(json.dumps(call(method, path, body), indent=1))
