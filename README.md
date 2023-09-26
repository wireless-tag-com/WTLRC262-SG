## Brief

The demo uses the esp8684 chip to drive the sx1262, and can use two identical devices to achieve point-to-point communication.

## Environmental description

| sdk     | sdk version  | short commit |
| ------- | ------------ | ------------ |
| esp-idf | release/v5.1 | gf0437b945f  |

If you need to build an environment, see [Environment construction](https://docs.espressif.com/projects/esp-idf/zh_CN/release-v5.1/esp32c2/get-started/linux-macos-setup.html).

## Hardware description

- esp8684
- sx1262

## Instructions for use

Build a software environment, clone the demo, compile and burn it to the hardware according to the official instructions of esp-idf. If you need to view the serial port print, the protocol is 8N1 and the baud rate is 115200.

## Supplementary statement

The driver part of an sx1262 references the semtech official routine [Driver routine](https://github.com/Lora-net/SWSD003).

## Link
WTLRC262-SG：https://www.wireless-tag.com/product-item-58.html

ESP8684：https://www.espressif.com.cn/en/products/socs/esp32-c2

SX1262：https://www.semtech.com/products/wireless-rf/lora-connect/sx1262