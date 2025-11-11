# BLE Motion Tracker  
### Step Count • Jump Count • Jump Length • Real-time Bluetooth Data

A lightweight motion-tracking system built using **MPU6050** + **ESP32** for detecting **steps**, **jumps**, and **jump length**, and streaming the data over **Bluetooth Low Energy (BLE)**.  
Designed for fitness experiments, sports analytics, and embedded-systems learning.

---

## 🚀 Features

- ✅ **Step Detection** using acceleration magnitude + FSM  
- ✅ **Jump Detection** (takeoff + landing detection)  
- ✅ **Jump Length Estimation** using time-of-flight + velocity approximation  
- ✅ **Real-time Bluetooth Streaming** to mobile / PC  
- ✅ **Noise reduction** with a moving average filter  
- ✅ **Modular Arduino code** for easy customisation  
- ✅ **Low power ESP32** implementation

---

## 📡 Hardware Used

| Component | Purpose |
|----------|---------|
| **ESP32** | BLE communication + main controller |
| **MPU6050 (Accelerometer + Gyro)** | Motion sensing |
| Battery Pack | Portable use |
| Optional: Enclosure | Wearable attachment |

---

## 🛠️ Project Structure


---

## 📐 How Step Detection Works

We use a **3-state Finite State Machine (FSM):**

1. **IDLE** – waiting for motion  
2. **RISING** – acceleration > upper threshold  
3. **FALLING** – acceleration dips below lower threshold → **step counted**

This avoids double-counting and reduces noise compared to naive peak detection.

---

## 🦘 Jump Detection Logic

Jump is detected using:

- **Takeoff:** sudden drop in vertical acceleration (approaching free fall)  
- **Air Time:** timestamp difference between takeoff & landing  
- **Landing:** sharp high-acceleration spike  
- **Jump Length:**  
  \[
  L = v \times t_{air}
  \]
  where \(v\) is estimated initial horizontal velocity using filtered acceleration.

This is a simplified model suitable for wearable sensors.

---

## 📲 Bluetooth Format (Example)

Data is streamed as comma-separated values:

