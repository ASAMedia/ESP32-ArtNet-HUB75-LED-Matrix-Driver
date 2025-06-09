---

# ESP32 Art-Net LED Matrix Controller

![Final Product](./images/final.jpg)

> A high-performance ESP32-based controller for HUB75 P5 64×32 RGB LED matrix panels, receiving Art-Net DMX over Wi-Fi and displaying pixel data at up to 20 FPS.

---

## Table of Contents

1. [Features](#features)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Installation](#installation)
5. [Configuration](#configuration)
6. [Usage](#usage)
7. [QLC+ Example](#qlc-example)
8. [Resources & Links](#resources--links)
9. [Limitations](#Limitations)
10. [License](#license)

---

## Features

* **Fast rendering**: lookup-table pixel packing, and dual-core tasks for 10–20 FPS.
* **Art-Net DMX**: Receives up to 16 universes (5–20) over Wi-Fi via Art-Net protocol.
* **Configurable**: Adjust I²S clock, and panel geometry in `setup()`.
* **Open-source**: Based on the [ESP32-HUB75-MatrixPanel-I2S-DMA](https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA) library.

---

## Hardware Requirements

* **ESP32** development module (e.g. DOIT ESP32 DEVKIT V1)
* **SCOREDUINO-ESP32** Modules for P3, P4, and P5 Indoor RGB LED Matrix Display
* **P5 64×32 RGB LED Matrix** panel
* **Jumper wires** or **ribbon cable** to connect HUB75 interface
* **5 V power supply**, capable of at least 4 A for a 64×32 panel

---

## Software Requirements

* [Arduino IDE](https://www.arduino.cc/en/software)
* **Libraries**:

  * [ESP32-HUB75-MatrixPanel-I2S-DMA](https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA)
  * [AsyncUDP](https://github.com/me-no-dev/AsyncUDP)
  * Built-in: `WiFi.h`, `Arduino.h`

---

## Installation

1. **Clone the repo**:

   ```bash
   git clone https://github.com/yourusername/esp32-artnet-matrix.git
   cd esp32-artnet-matrix
   ```
2. **Install libraries** via Library Manager.
3. **Open** the `esp32_artnet_matrix.ino` sketch in Arduino IDE.

---

## Configuration

1. Edit Wi-Fi credentials and **hostname** in `setup()`.
2. Adjust pin mappings and panel dimensions under the `// Panel pin definitions` section.
3. (Optional) Change:

   * `cfg.i2sspeed` to `HZ_20M` for higher throughput
   * `setBrightness8(128)` for higher brightness

---

## Usage

1. Power up the ESP32 and panel.
2. Connect to your Wi-Fi network (SSID/password as configured).
3. Send Art-Net DMX data (universes 5–20) to the ESP32’s IP on UDP port **6454**.
4. The panel will update automatically as packets arrive.

<img src="./video/demo.gif" alt="Demo" width="600" />

---

## QLC+ Example

An example QLC+ project file is included in the `examples/` folder.
Use it to stream test patterns or live pixels over Art-Net:

* `examples/qlcplus-matrix-demo.qlc`

---

## Resources & Links

* **ESP32**: [https://www.espressif.com/en/products/socs/esp32](https://www.espressif.com/en/products/socs/esp32)
* **SCOREDUINO-ESP32** Modules: [https://github.com/Scoreduino/SCOREDUINO-ESP32](https://buildcircuits.com/products/scoreduino-esp32-modules-for-p3-p4-and-p5-indoor-rgb-led-matrix-display?_pos=1&_sid=81cdcb21f&_ss=r&variant=41666553512034)]
* **P5 64×32 LED Matrix**: [https://www.adafruit.com/product/2277](https://buildcircuits.com/products/p5-64x32-pixels-rgb-indoor-led-matrix-display?_pos=12&_sid=81cdcb21f&_ss=r)
* **ESP32-HUB75-MatrixPanel-I2S-DMA** Library: [https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA](https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA)

---

## Limitations

* **Wireless throughput**: Over Wi‑Fi and Art‑Net, UDP packet reception can introduce latency and variability in timing, limiting sustained refresh rates to around **10–20 FPS** on busy networks.
* **Packet loss & retries**: In congested or distant Wi‑Fi environments, dropped Art‑Net packets may cause frame stalls or visual artifacts.
* **Network configuration**: Routers or access points configured with client isolation, high latency, or heavy multicast filtering can further reduce effective frame rate.

## License

This project is licensed under the Apache License. See [LICENSE](LICENSE) for details.

---
