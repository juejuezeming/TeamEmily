# Pinout Übersicht

## MOTOR & ENCODER

### MOTOR R:
- Encoder A: `D3`  
- Encoder B: `D2`  
- `R_PWM`: `D11`  
- `R_DIR`: `D10`  

### MOTOR L:
- Encoder A: `D18`  
- Encoder B: `D19`  
- `L_PWM`: `D8`  
- `L_DIR`: `D9`  

---

## ULTRASCHALLSENSOREN

- Sensor 1:  
  - Trigger: `D23`  
  - Echo: `D22`  

- Sensor 2:  
  - Trigger: `D25`  
  - Echo: `D24`  

---

## STEUERBOARD

- Linker Knopf: `D1`  
- Rechter Knopf: `D0`  
  - *(LOW when pressed)*  

- Blaue LEDs: `D4`, `D5`, `D6`, `D13`  

---

## TOF-Sensoren

- I²C: `SDA`, `SCL` (gemeinsam)  
- `XSHUT1`: `D16`  
- `XSHUT2`: `D17`  

---

## SERVO

- Signal: `D7`
