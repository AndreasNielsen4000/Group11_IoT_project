# Project of IoT: Task Management Checker


# TrackLogix  

TrackLogix is a privacy-focused solution for time and geolocation tracking, designed for professionals to log jobsite arrivals and departures with ease.  

## Features  
- **Privacy by Design:** Tracking is only activated when the user presses a button.  
- **Jobsite Tracking:** Logs arrival and departure times with timestamps and geolocation.  
- **Web Interface:** View and manage logged data through a web app with map and list views.  
- **Power Efficiency:** Operates in sleep mode and wakes only on button press to conserve energy.  
- **Automatic LED Adjustment:** Uses a photoresistor to adjust LED brightness based on ambient light.  
- **Web-App Integration:** Allows logging and data access via a phone app, even away from the vehicle.  

## Use Case  
The device is ideal for craftsmen and other professionals who need to:  
1. Press the button upon arriving at a jobsite to log the start time.  
2. Press the button again when leaving to log the end time.  
3. Access the logged data later for reporting or analysis.  

## System Components  
1. **Hardware**:  
   - Pro-Mini (with optional Arduino Uno or ESP8266)  
   - GNSS module for location tracking  
   - LoRaWAN for data transmission  
   - Photoresistor for LED brightness adjustment  
   - Push-button for manual activation  

2. **Backend**:  
   - Azure for secure logging and data storage  

3. **Frontend**:  
   - Web app to display data as a sortable list or map  

## Power Management  
- The system remains in low-power sleep mode until the button is pressed.  
- Transmits data and returns to sleep immediately to optimize battery life.  

## Setup Instructions  
1. **Hardware Assembly**:  
   - Connect the Pro-Mini to the GNSS module and LoRaWAN transmitter.  
   - Attach the photoresistor and LED for brightness control.  
   - Install the push-button for activation.  

2. **Software Installation**:  
   - Upload the Arduino code to the Pro-Mini.  
   - Configure LoRaWAN and Azure settings in the code.  

3. **Frontend Deployment**:  
   - Deploy the web app on your preferred server or use Azure hosting.  
   - Configure API endpoints to connect with the backend.  

## Usage  
1. Install the device in your vehicle.  
2. Press the button to log timestamps and locations.  
3. View logged data on the web app for tracking and reporting.  

## Future Enhancements  
- Integration with additional sensors for expanded functionality.  
- Enhanced data visualization tools.  
- Mobile app for better accessibility.  

## License  
This project is licensed under the MIT License. See `LICENSE` for details.  

## Contributing  
Contributions are welcome! Feel free to fork this repository and submit a pull request.  

## Contact  
For any questions or feedback, please contact us at support@tracklogix.com.  
