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

## Опис підключення

https://evo.net.ua/waveshare-rp2040-zero-development-board-20187/?gad_source=1&gclid=Cj0KCQiA5-uuBhDzARIsAAa21T97qgjEY4JwgWkLLuh9oGOced7Py77p4wcL7k6ulejtFxWxFpEWVdYaAgHmEALw_wcB

https://evo.net.ua/content/uploads/images/rp2040-zero-details-7.jpg

Debug UART0, 115200
GP0 <- TX, GP1 <- RX

CLRS RX UART1, 420kbod
GP8 <- TX, GP9 <- RX

DAC1, DAC2 (I2C0)
GP4 <- SDA, GP5 <- SCL

DAC3, DAC4 (I2C1)
GP6 <- SDA, GP7 <- SCL




## Всілякі посилання

CRSF з проекту AM32-ESC

https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/blob/master/Src/crsf.c

Процесор та драйвер ключів в одному корпусі:

https://www.digikey.com/en/products/detail/STSPIN32F0/497-16978-ND/6236758?curr=usd&utm_campaign=buynow&utm_medium=aggregator&utm_source=octopart

### Запишу сюди посилання по AI:

RISC-V Milk-V Duo

https://www.aliexpress.com/item/1005005979520624.html?gad_source=1&dp=f5fcb5172f6273b1e696f7646bdfa329&af=984299&cv=47843&afref=https%3A%2F%2Fwww.google.com%2F&mall_affr=pr3&utm_source=admitad&utm_medium=cpa&utm_campaign=984299&utm_content=47843&dp=f5fcb5172f6273b1e696f7646bdfa329&af=984299&cv=47843&afref=https%3A%2F%2Fwww.google.com%2F&mall_affr=pr3&utm_source=admitad&utm_medium=cpa&utm_campaign=984299&utm_content=47843&aff_fcid=172d7fe336b84905baab88c47f1afbbe-1708992624705-05872-_ePNSNV&aff_fsk=_ePNSNV&aff_platform=portals-tool&sk=_ePNSNV&aff_trace_key=172d7fe336b84905baab88c47f1afbbe-1708992624705-05872-_ePNSNV&terminal_id=77d224a98e3c40d3ae15351f213f4d14&afSmartRedirect=y

https://github.com/milkv-duo/cvitek-tdl-sdk-cv180x/tree/main

І в подальшому RISC-V Milk-V Duo 256 (інший процесор та вдвічі більше (не факт) потужність NPU)
