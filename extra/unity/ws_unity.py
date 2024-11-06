import asyncio
import websockets
import json

async def echo(websocket, path):
    async for message in websocket:
        try:
            data = json.loads(message)
            rounded_data = {key: round(value, 3) for key, value in data.items()}
            rounded_message = json.dumps(rounded_data)
            print(f"Received: {rounded_message}")
            # await asyncio.sleep(0.0)
        except json.JSONDecodeError as e:
            print(f"Invalid JSON: {e}")

# Create and run the WebSocket server within an asyncio event loop
start_server = websockets.serve(echo, "10.18.88.73", 8765)  # Change the IP and port as needed

# Create an asyncio event loop
loop = asyncio.get_event_loop()

# Start the WebSocket server within the event loop
loop.run_until_complete(start_server)

# Keep the event loop running indefinitely
loop.run_forever()
