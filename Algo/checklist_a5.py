import sys
import requests


def main():
    steps = "['left,90,reverse,0', 'center,0,forward,40', 'left,90,forward,0', 'center,0,forward,10', 'left,90,forward,0', 'SNAP1', 'FIN']"

    payload = f"CONTROL - {steps}"

    url = "http://127.0.0.1:5001/path"
    try:
        resp = requests.post(url, data=payload)
        resp.raise_for_status()
    except Exception as e:
        print(f"request failed: {e}")
        sys.exit(1)

    print("status", resp.status_code)
    try:
        print("response:", resp.json())
    except ValueError:
        print("non-json response:", resp.text)


if __name__ == "__main__":
    main()
    