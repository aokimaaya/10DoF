import asyncio
import websockets
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Initialize cube vertices
cube_vertices = np.array([[-0.5, -0.5, -0.5],
                          [0.5, -0.5, -0.5],
                          [0.5, 0.5, -0.5],
                          [-0.5, 0.5, -0.5],
                          [-0.5, -0.5, 0.5],
                          [0.5, -0.5, 0.5],
                          [0.5, 0.5, 0.5],
                          [-0.5, 0.5, 0.5]])

# Define cube edges (vertex indices for edges)
cube_edges = [(0, 1), (1, 2), (2, 3), (3, 0),
              (4, 5), (5, 6), (6, 7), (7, 4),
              (0, 4), (1, 5), (2, 6), (3, 7)]

# Function to update and visualize the cube
def update_cube(x, y, z, rx, ry, rz):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.cla()

    # Rotate cube
    rotation_matrix = np.array([[np.cos(rz), -np.sin(rz), 0],
                                 [np.sin(rz), np.cos(rz), 0],
                                 [0, 0, 1]])

    cube_vertices_rotated = cube_vertices.dot(rotation_matrix)

    # Translate cube
    cube_vertices_transformed = cube_vertices_rotated + [x, y, z]

    # Draw cube edges
    for edge in cube_edges:
        ax.plot(*zip(*cube_vertices_transformed[edge]), c='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)

    plt.draw()
    plt.pause(0.01)

# WebSocket server handler
async def cube_server(websocket, path):
    global x, y, z, rx, ry, rz

    while True:
        try:
            # Receive JSON data from the WebSocket
            data = await websocket.recv()
            data = json.loads(data)

            # Update cube position and rotation based on received data
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            z = data.get('z', 0.0)
            rx = data.get('rx', 0.0)
            ry = data.get('ry', 0.0)
            rz = data.get('rz', 0.0)

            # Update and visualize the cube
            update_cube(x, y, z, rx, ry, rz)

            await asyncio.sleep(0.01)  # Yield control to the event loop

        except Exception as e:
            print(f"WebSocket Error: {str(e)}")

# Start the WebSocket server within an asyncio event loop
async def main():
    start_server = await websockets.serve(cube_server, "10.18.88.73", 8765)  # Change IP and port as needed
    async with start_server:
        await asyncio.Future()  # Keep the event loop running

# Run the asyncio event loop
if __name__ == "__main__":
    asyncio.run(main())
