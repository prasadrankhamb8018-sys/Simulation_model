import numpy as np
import matplotlib.pyplot as plt


def safe_float(prompt, default):
    while True:
        s = input(prompt)
        if s.strip() == "":
            print(f"Using default value: {default}")
            return default
        try:
            return float(s)
        except:
            print("Invalid input. Please enter a number.")


def get_thrust_profile():
    print("\nEnter thrust profile:")
    print("Format: <time> <thrust>")
    print("Example: 0 50")
    print("Type 'done' to finish.\n")

    thrust_profile = {}
    auto_time = 0

    while True:
        s = input("Time Thrust: ")

        if s.lower().strip() == "done":
            break

        parts = s.split()

        
        if len(parts) == 1:
            try:
                F = float(parts[0])
                auto_time += 1
                thrust_profile[auto_time] = F
                print(f" Added: time={auto_time}, thrust={F}")
            except:
                print("Invalid value. Try again.")
            continue

        
        if len(parts) == 2:
            try:
                t = float(parts[0])
                F = float(parts[1])
                thrust_profile[t] = F
                print(f" Added: time={t}, thrust={F}")
            except:
                print("Invalid numbers. Try again.")
            continue

        print("Invalid format. Enter: <time> <thrust> or just <thrust>.")

    
    if not thrust_profile:
        print("No thrust entered. Using default motor thrust curve.")
        thrust_profile = {0: 50, 1: 150, 2: 120, 3: 80, 4: 0}

    return dict(sorted(thrust_profile.items()))


G0 = 9.81
R_AIR = 287.05
L = 0.0065
T_0 = 288.15
P_0 = 101325



class Vehicle:
    def __init__(self, mass, diameter, Cd_rocket, Cd_parachute, parachute_area, thrust_profile):
        self.mass = mass
        self.area = np.pi * (diameter / 2)**2
        self.Cd_rocket = Cd_rocket
        self.Cd_parachute = Cd_parachute
        self.parachute_area = parachute_area
        self.thrust_profile = thrust_profile
        self.thrust_time_max = max(thrust_profile.keys())



def get_temperature(altitude):
    return max(T_0 - L * altitude, 216.65)


def get_density(altitude, T):
    pressure = P_0 * (T / T_0)**(G0 / (R_AIR * L))
    return pressure / (R_AIR * T)



def calculate_forces(time, altitude, velocity, vehicle):
    T = get_temperature(altitude)
    rho = get_density(altitude, T)

    
    thrust = np.interp(
        time,
        list(vehicle.thrust_profile.keys()),
        list(vehicle.thrust_profile.values())
    ) if time <= vehicle.thrust_time_max else 0

    F_g = vehicle.mass * G0

    
    if velocity < 0 and altitude < 900:
        Cd = vehicle.Cd_parachute
        A = vehicle.parachute_area
    else:
        Cd = vehicle.Cd_rocket
        A = vehicle.area

    drag = 0.5 * rho * velocity**2 * Cd * A
    F_net = thrust - F_g - drag * np.sign(velocity)

    return F_net



def simulate(vehicle, dt, max_time):
    time = 0
    altitude = 0
    velocity = 0

    T_list, H_list, V_list, A_list = [], [], [], []

    while time < max_time:
        F_net = calculate_forces(time, altitude, velocity, vehicle)
        acceleration = F_net / vehicle.mass

        velocity += acceleration * dt
        altitude += velocity * dt
        time += dt

        if altitude < 0:  
            altitude = 0
            break

        T_list.append(time)
        H_list.append(altitude)
        V_list.append(velocity)
        A_list.append(acceleration)

    return np.array(T_list), np.array(H_list), np.array(V_list), np.array(A_list)



print("\n === SYSTEM FOR SIMULATION ===")

mass = safe_float("Enter mass (kg) [default=0.5]: ", 0.5)
diameter = safe_float("Enter diameter (m) [default=0.065]: ", 0.065)
Cd_rocket = safe_float("Enter rocket Cd [default=0.5]: ", 0.5)
Cd_parachute = safe_float("Enter parachute Cd [default=1.5]: ", 1.5)
parachute_area = safe_float("Enter parachute area (m^2) [default=0.2]: ", 0.2)
max_time = safe_float("Simulation time (s) [default=120]: ", 120)
dt = safe_float("Time step dt [default=0.01]: ", 0.01)

thrust_profile = get_thrust_profile()

vehicle = Vehicle(mass, diameter, Cd_rocket, Cd_parachute, parachute_area, thrust_profile)


times, altitudes, velocities, accelerations = simulate(vehicle, dt, max_time)


plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(times, altitudes)
plt.title("Altitude vs Time")
plt.ylabel("Altitude (m)")
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(times, velocities)
plt.title("Velocity vs Time")
plt.ylabel("Velocity (m/s)")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(times, accelerations)
plt.title("Acceleration vs Time")
plt.ylabel("Acceleration (m/s²)")
plt.xlabel("Time (s)")
plt.grid(True)

plt.tight_layout()
plt.show()

print("\nSimulation Complete!")
print(f"Max altitude: {max(altitudes):.2f} m")
