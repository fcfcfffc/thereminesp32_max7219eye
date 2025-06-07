🎛️ ESP32-S3 Frequency-Synchronized Expression Display
This project uses the ESP32-S3 to detect frequency input, synchronize via ESP-NOW, control PWM output, and visually display "expressive eyes" using LED matrices. It is designed to monitor external frequency input, determine its stability, and visually react using different eye expressions based on real-time frequency changes.

🚀 Features
📡 Frequency Detection: Utilizes ESP32's PCNT (pulse counter) hardware module to measure incoming frequency signals.

🔘 Reference Frequency Setting: Supports manual reference setting via a button or automatic detection when the input signal stabilizes.

🎚️ PWM Output: Outputs 8-bit precision PWM signals via ledcWrite.

🔄 Data Smoothing & Stability Detection: Implements exponential moving average filtering and determines stability using frequency change thresholds.

📶 ESP-NOW Transmission: Sends calculated PWM values wirelessly to a paired ESP-NOW receiver.

👁️ LED Matrix Expressions: Displays expressive "eyes" using two 8x8 LED matrices, including blinking, directional glances, and neutral states.

🔧 Hardware Connections
Function	GPIO	Description
Frequency Input	1	External signal input
PWM Output	2	PWM signal output
Set Button	19	Reference frequency set
LED Matrix - CS	4	Chip Select
LED Matrix - CLK	5	Clock
LED Matrix - DIN	6	Data Input

📦 Dependencies
Ensure the following libraries are installed:

esp_now.h

LedControl.h

SPI.h

WiFi.h

driver/pcnt.h (ESP-IDF driver included with ESP32 platform)

🧠 How It Works
Frequency Sampling:

Frequency is sampled every 200ms using PCNT.

An exponential moving average (alpha = 0.5) smooths the readings.

Stability Detection:

If frequency deviation remains below a threshold (3Hz) for 10 consecutive samples, it is considered stable.

The system automatically or manually sets this as the reference frequency.

PWM Mapping and Sync:

Frequency differences from the reference are mapped to a PWM value (0–255).

PWM value is sent to a paired device over ESP-NOW.

LED Matrix Display:

Displays various expressions based on signal state:

Unstable: blinking or closed eyes.

Stable with offset: eyes glancing left or right.

Just calibrated: wide-open eyes.

🧪 How to Use
Flash the code to your ESP32-S3 board.

Connect a signal source to GPIO 1.

Use GPIO 19 button to manually set the reference frequency, or allow auto-calibration when stable.

Monitor LED matrix eyes for visual feedback.

Ensure a receiving device is paired via ESP-NOW to receive PWM values.
