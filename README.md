## Preparing to work


### Python experiments

Create new virtual environment

Install Zephyr SDK and toolchain. See original Zephyr documentation
https://docs.zephyrproject.org/latest/develop/getting_started/index.html or
https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.5.0/nrf/installation.html.

(TBD)
```bash
pushd $HOME/.local/opt
curl -L -O https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_macos-aarch64.tar.xz
```

```bash
mkdir elrs-control
cd elrs-control
python3 -m venv ./.venv
. ./.venv/bin/activate
pip install west
#git clone https://github.com/baden/esp32-c3-super-mini
#west init -l esp32-c3-super-mini
west init -m https://github.com/baden/esp32-c3-super-mini --mr main fxa-custom-zephyr esp32-c3-super-mini
cd esp32-c3-super-mini
#git submodule update --init --recursive
west update
pip install -r ../zephyr/scripts/requirements.txt
```

