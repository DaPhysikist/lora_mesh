import datetime
from fastapi import FastAPI, WebSocket
from fastapi.responses import Response
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles   
import uvicorn
import asyncio
import os
import json

import warnings
import serial
import serial.tools.list_ports            

app = FastAPI()     #initialize the web app
app.mount("/static", StaticFiles(directory="static"), name="static")    

ser = None

def connect_to_feather():
    global ser

    arduino_ports = []
    for p in serial.tools.list_ports.comports():
        if 'USB' in p.description or 'ACM' in p.device or 'usbmodem' in p.description or 'tty' in p.device:
            arduino_ports.append(p.device)
    if not arduino_ports:
        raise IOError("No Arduino found")
    if len(arduino_ports) > 1:
        warnings.warn('Multiple Arduinos found - using the first')
    ser = serial.Serial(arduino_ports[0], 9600)

@app.get("/", response_class=HTMLResponse)       
def get_html() -> HTMLResponse:                 
  with open('lora_config.html') as html:             
    return HTMLResponse(content=html.read())               
  
@app.post("/config_params")       #configures LoRa parameters
def config_params(parameters: dict):
   print(parameters)
   bValue = parameters["bandwidth"]
   sfValue = parameters["spreading_factor"]
   crValue = parameters["coding_rate"]
   tpValue = parameters["tx_power"]
   message = parameters["message"]
   param_message = f"{tpValue},{bValue},{sfValue},{crValue},{message}\n"
   byte_message = param_message.encode('utf-8')
   ser.write(byte_message)

@app.post("/reconnect_node")
def reconnect_node():
    connect_to_feather()

@app.post("/data_rate_test")
def data_rate_test(parameters: dict):
    duration = parameters["duration"]
    packetSize = parameters["packet_size"]
    numPackets = parameters["num_packets"]
    print(parameters)

@app.websocket("/message")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        with open(log_file_name, "a") as log_file:
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    print(line)
                    log_file.write(line + '\n')
                    log_file.flush()
                    await websocket.send_text(line)                          
                await asyncio.sleep(0.1)            
    except Exception as e:
        print(f"Error: {e}")
        await websocket.close()
  
if __name__ == "__main__":
    connect_to_feather()
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file_name = f"../serial_output_{current_time}.log"
    uvicorn.run(app, host="0.0.0.0", port=6543)     #starts the app
