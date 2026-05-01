Smart Encoder Motor Driver
![Encoder Motor Driver - Front View](https://github.com/egeozgul/Servo-Motor-Feedback-Controller/raw/main/pcb_.png)
The Smart Encoder Motor Driver is a compact, closed-loop motor controller designed for DC motors equipped with magnetic or optical encoders. It integrates a PID feedback controller and an H-bridge on a single board, enabling precise, autonomous motor control with minimal host-side overhead. The board is powered by the STM32C0 microcontroller and communicates over I2C or UART, making it easy to integrate into robotics, automation, and multi-axis motion systems.
---
PCB Assembly Video
![Watch the assembly video](https://img.youtube.com/vi/LhhGqf6qH90/maxresdefault.jpg)
---
Key Features
Closed-Loop PID Control — High-frequency feedback loop for precise, stable motor operation
Trapezoidal Trajectory Planner — Smooth motion profiles with configurable acceleration and deceleration limits
ASCII Command Interface — Set target positions, tune PID gains, query torque feedback, and more over a simple serial protocol
Wide Voltage Range — 5 V to 45 V supply
Current Capacity — Up to 4.1 A continuous
Dual Communication — I2C and UART supported simultaneously
Scalable Architecture — Qwiic connectors allow up to 112 drivers to be daisy-chained on a single I2C bus (the maximum supported by the 7-bit I2C address space)
---
Firmware
The main firmware source file is located in the `SourceCode/Src` folder.
---
Trapezoidal Trajectory Planner
The onboard trapezoidal trajectory generator computes smooth motion profiles in real time, controlling both acceleration and deceleration phases to eliminate sudden jerks, reduce mechanical wear, and prevent overshoot.
Configurable parameters:
Maximum speed
Acceleration and deceleration limits
Target position
The controller continuously evaluates the current state against the trajectory and adjusts motor drive accordingly — no trajectory pre-computation is required on the host side.
Live Motion Plots
A custom web-based UART plotter captures real-time telemetry from the driver. The plots below show a sample motion sequence with three simultaneously streamed variables:
Position
Velocity
Motor current draw
![Motion profile plot](https://private-user-images.githubusercontent.com/10923392/403673709-ec2e79d4-fb35-4699-954e-250158fb1edc.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Nzc1OTQ2ODIsIm5iZiI6MTc3NzU5NDM4MiwicGF0aCI6Ii8xMDkyMzM5Mi80MDM2NzM3MDktZWMyZTc5ZDQtZmIzNS00Njk5LTk1NGUtMjUwMTU4ZmIxZWRjLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNjA1MDElMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjYwNTAxVDAwMTMwMlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTY1NWNiYWUwMzU1MWJkYjlmZWM1YjJkMzljYzY2NTZkODBhNGQwODg3MDMxMmM4MzFhNjBlMmE3MmM2MTQ4MTkmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JnJlc3BvbnNlLWNvbnRlbnQtdHlwZT1pbWFnZSUyRnBuZyJ9.fMu3Dml4f01wiQo1fdzR42pAyiqzO0xeKwGzRGUTaBs)
---
Custom Live Plotter
A purpose-built browser-based plotter is available for real-time visualization of motor telemetry over UART.
🔗 Live UART Plotter
Supports an unlimited number of simultaneously plotted variables with fully customizable styling. No installation required — runs entirely in the browser.
---
Hardware
Final PCB
![PCB front view](https://private-user-images.githubusercontent.com/10923392/447361835-449203f8-8fd5-4fc8-97ef-0804dd21e011.jpg?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Nzc1OTQ2ODIsIm5iZiI6MTc3NzU5NDM4MiwicGF0aCI6Ii8xMDkyMzM5Mi80NDczNjE4MzUtNDQ5MjAzZjgtOGZkNS00ZmM4LTk3ZWYtMDgwNGRkMjFlMDExLmpwZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNjA1MDElMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjYwNTAxVDAwMTMwMlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTc1YjNhNjQ3YTRiYTcwNWQ5MjI4ZDA0YzJmNGQ2ZTFjMWUwMzI0MDRlNDM2MjI5OWY4YTczM2RjNjA1ZjEzMDkmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JnJlc3BvbnNlLWNvbnRlbnQtdHlwZT1pbWFnZSUyRmpwZWcifQ.Uj-n246ZcasvQ2SJTe5A0D8wEk81IPVy69HD1rZMheU)
![PCB assembled](https://private-user-images.githubusercontent.com/10923392/476422184-fbc92872-6157-4653-9876-7cdbfeab308c.jpg?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Nzc1OTQ2ODIsIm5iZiI6MTc3NzU5NDM4MiwicGF0aCI6Ii8xMDkyMzM5Mi80NzY0MjIxODQtZmJjOTI4NzItNjE1Ny00NjUzLTk4NzYtN2NkYmZlYWIzMDhjLmpwZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNjA1MDElMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjYwNTAxVDAwMTMwMlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTU2NWIxZWIzMjBiNDE5MTc5ZjU1MjQzNGM4YWJmMGE1OWZiNTA4MzgwNDZmZjE0ZjhhNWUwZGVkMGZmYmE5YjQmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JnJlc3BvbnNlLWNvbnRlbnQtdHlwZT1pbWFnZSUyRmpwZWcifQ.HhCzAtrO5cGMg6rN0AqhgfsMZkW2KL5mIBt92hPetw0)
Design files — including schematics, Gerber files, and BOM — are available in the repository.
---
Connectivity and Expansion
The driver uses Qwiic connectors for I2C chaining, allowing multiple units to share a single two-wire bus. Up to 112 drivers can be addressed on one I2C bus using the standard 7-bit address space (128 total addresses minus the 16 reserved by the I2C specification).
This makes the driver well suited for multi-axis systems such as robotic arms, CNC machines, and automated platforms where independent closed-loop control of many motors is required from a single host controller.
---
License
This project is open-source under the MIT License. You are free to use, modify, and distribute it for any purpose.
