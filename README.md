# Temuulel_OBC

**Temuulel_OBC** is the onboard computer firmware project for the Temuulel satellite mission. This repository contains the core source code for the OBC, written in C, developed and tested using a VS-based development environment.

## 🧠 Overview

The onboard computer (OBC) is responsible for handling communication, telemetry, command processing, and mission-critical operations. This repository is focused on:

- Bare-metal embedded programming for PIC18
- GPIO control, UART/SPI communication
- Handel sub system data streams in order 
- Minimal dependencies (no dynamic memory / malloc)
  

## 📁 Project Structure
- IDK just started

  
## ⚙️ Build & Development

- Compiler: VS-based C compiler (without malloc support)
- Target: PIC18 microcontroller
- Toolchain: XC8 
- Debugging via UART & GPIO-based LED feedback

## ✅ Features

- [x] Shutdown counter with persistent storage (EEPROM or similar)
- [x] RTCC timmer
- [x] Flash memory opperations
- [ ] CRC integrity checks (future implementation)
- [ ] Telemetry packet handling

## 📦 How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/ganjiguur16/Temuulel_OBC.git
   cd Temuulel_OBC
