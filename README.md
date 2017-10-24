# esp8266-xpressnet-z21
Adaptation of the Z21 emulation from Philipp Gahtow to work on ESP8266 using ported libraries [esp8266-xpressnet-lib](https://github.com/kind3r/esp8266-xpressnet-lib) and [esp8266-z21-lib](https://github.com/kind3r/esp8266-z21-lib).

**Under development.**

## Default pin configuration:

### XpressNet (RS485)
| ESP8266 | Variable       | Description |
|---------|----------------|-------------|
| GPIO2   | `XNetRS485_RX` | RX          |
| GPIO4   | `XNetRS485_TX` | TX          |
| GPIO5   | `RS485_TXRX_PIN` |TX Control (optional) |

### S88n
| ESP8266 | Variable       | Description |
|---------|----------------|-------------|
| GPIO15  | `S88DataPin`   | Data in     |
| GPIO13  | `S88ClkPin`    | Clock       |
| GPIO12  | `S88PSPin`     | PS/LOAD     |
| GPIO14  | `S88ResetPin`  | Reset       |

