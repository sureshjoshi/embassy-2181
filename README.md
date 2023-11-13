# Reproduction for Issue #2181

The sample code uses a serial speed of 921600 and CTS/RTS flow control. A buffered uart on a separate task consumes bytes until a panicking.

After installing a softdevice and the code in this repo onto an NRF52840, the following script will cause a panic.

```bash
cd scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Update your serial port in panic.py
python panic.py
```
