# sGOOter: The Autonomous E-Scooter Project

## About the Project

In the bustling corridors of industrial parks and the vibrant pathways of school campuses, the sGOOter project envisions a future of smart, sustainable transportation. sGOOter is an autonomous electric scooter (e-Scooter) designed with a dual focus on advanced intelligence and eco-friendly mobility. Traditional transportation methods, often inefficient and pollutive, are ripe for transformation. sGOOter aims to revolutionize these methods by leveraging the synergy of autonomous technology and clean electric propulsion.

The sGOOter isn't just a vehicle; it's a cutting-edge solution equipped with features that ensure a safe, intelligent, and harmonious interaction with its environment:

- **Autonomous Driving**: Navigate with autonomy through complex environments.
- **Intelligent Path Planning**: Efficient route determination for optimal travel.
- **Auto-Parking**: Hassle-free automatic docking at journey’s end.
- **Voice Command Interface**: Hands-free operation through voice interactions.
- **Safety Monitoring**: Constant vigilance with onboard safety sensors.
- **Environmental Consciousness**: A green approach to local transportation.

Join us in pioneering the next wave of eco-conscious, intelligent mobility with sGOOter.

## Development Environment

This project is developed using:

- **Webots**: Our go-to simulation platform that replicates real-world conditions within a virtual testing environment.
- **Python**: The scripting language of choice for writing our control algorithms, praised for its flexibility and community support.

## Repository Structure

### Controllers

Components managing various functionalities of the autonomous system:
- Autonomous vehicle part:
    - `autonomous_vehicle`: Manages the self-driving capabilities.
    - `crossroads_traffic_lights`: Controls the traffic lights at intersections.
    - `generic_traffic_light`: Regulates standard traffic lights.
- sGOOter part:
    - `little_bicycle_P_V2`: Currently controls circular movement; will evolve with project development.
    - `keyboard_control`: User keyboard to control the motion.
    - `scooter_follow_line`: Follow the line automaticly while enabling the key interference.

### Worlds

The virtual environments where our concepts come to life:
- `city.wbt`: The Webots world file simulating urban conditions for sGOOter operation.
- `obj`: Please download the e-scooter model file follwing this link: https://drive.google.com/drive/folders/1xkpanp5PWJnbIVZQzKmCscyqJh0M_cvK?usp=sharing . Unzip it and construct the `obj` file parallel to `city.wbt`.

### Plugins

Modules enhancing the functionality of the vehicle (currently non-essential for the primary operation of sGOOter).

### Protos

Proto files define the sGOOter’s 3D model and behavior as an entity. They serve as backups of the sGOOter robot node in `city.wbt` for now. But development starts with robot node first, which upon finalization are compiled into the proto.

## Installation and Requirements

To install and run sGOOter on your local machine, follow these steps:

1. Clone this repository to your local system.
2. Navigate to the repository folder and install the project dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Download the `obj` model file and Unzip it parallel to `city.wbt`.
4. Open Webots and load the `city.wbt` world file from the `worlds` directory.



