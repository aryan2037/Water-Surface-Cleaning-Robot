        import serial
import json
import time

# Configure the serial port (adjust 'COM3' or '/dev/ttyUSB0' to your Arduino port)
SERIAL_PORT = "COM7"  # Replace with your port, e.g., "/dev/ttyUSB0" for Linux
BAUD_RATE = 115200      # Match the baud rate used in the Arduino code
OUTPUT_FILE = "data.json"

def save_data_to_json(data):
    """Appends data to a JSON file."""
    try:
        # Load existing data if the file exists
        try:
            with open(OUTPUT_FILE, "r") as file:
                existing_data = json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            existing_data = []

        # Append the new datar
        existing_data.append(data)

        # Save the updated data back to the file
        with open(OUTPUT_FILE, "w") as file:
            json.dump(existing_data, file, indent=4)

    except Exception as e:
        print(f"Error saving data: {e}")

def main():
    # Initialize serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Listening on {SERIAL_PORT}...")

        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()

            if not line:
                continue  # Skip empty lines

            try:
                # Parse the JSON data
                data = json.loads(line)

                # Verify expected data structure
                expected_keys = ['temp', 'lat', 'lon', 'alt', 'heading']
                if all(key in data for key in expected_keys):
                    print("Received:", data)
                    # Save the data to the JSON file
                    save_data_to_json(data)
                else:
                    print("Received data missing required fields:", data)

            except json.JSONDecodeError:
                print(f"Invalid JSON: {line}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
