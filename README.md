# Automatic Street Light System using LDR

## 🎯 Objective  
To design a street light system that turns ON automatically in darkness and OFF in daylight using an Arduino and LDR.

---

## 🛠 Components Required  
- Arduino UNO  
- LDR (Light Dependent Resistor)  
- 10k Resistor  
- LED (Street Light)  
- Jumper wires  
- Breadboard  

---

## 🔌 Circuit Diagram  

+5V -----> LDR -----> A0 (Arduino)
|
10k Resistor ---> GND

Arduino Pin 13 ---> LED ---> Resistor ---> GND

---

## ⚙️ Working Principle  
- LDR changes resistance with light intensity.  
- In **darkness**, Arduino detects low light → LED turns ON.  
- In **daylight**, Arduino detects high light → LED turns OFF.  

---

## 🌍 Applications  
- Smart city street lighting.  
- Energy-saving home lighting.  
- Automatic outdoor lamps.  

---
