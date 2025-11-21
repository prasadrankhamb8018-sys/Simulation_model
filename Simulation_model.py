import numpy as np
import matplotlib.pyplot as plt
import csv
import os


def load_from_csv(filename):
    if not os.path.exists(filename):
        print("\nCSV file not found. Switching to manual input...\n")
        return None, None

    params = {}
    thrust_profile = {}
    thrust_section = False

    print(f"\nLoading configuration from: {filename}")

    
    with open(filename, "r", encoding="utf-8-sig") as f:
        reader = csv.reader(f)

        for row in reader:
            if not row:
                continue  

            row = [x.strip() for x in row]

            if row[0].startswith("#") or row[0] == "":
                continue  

            
            if row[0].lower() == "time":
                thrust_section = True
                continue

            if thrust_section:
                try:
                    t = float(row[0])
                    F = float(row[1])
                    thrust_profile[t] = F
                except:
                    print(" Invalid thrust row:", row)
                continue

            
            try:
                params[row[0]] = float(row[1])
            except:
                print(" Invalid parameter row:", row)

    # required parameter list
    required = [
        "mass", "diameter", "Cd_rocket", "Cd_parachute",
        "parachute_area", "max_time", "dt"
    ]

    for key in required:
        if key not in params:
            raise ValueError(f" CSV missing required parameter: {key}")

    if not thrust_profile:
        print(" Warning: No thrust data found in CSV. Using default thrust curve.")
        thrust_profile = {0: 50, 1: 150, 2: 120, 3: 80, 4: 0}

    return params, thrust_profile



def safe_float(prompt, default):
    s = input(prompt)
    return default if s.strip() == "" else float(s)


def get_thrust_profile_manual():
    print("\nEnter thrust profile manually.")
    print("Format: <time> <thrust>, or only <thrust>")
    print("Type 'done' to stop.\n")

    thrust_profile = {}
    auto_t = 0

    while True:
        s = input("Time Thrust: ")
        if s.lower() == "done":
            break

        parts = s.split()

        if len(parts) == 1:
            try:
                F = float(parts[0])
                auto_t += 1
                thrust_profile[auto_t] = F
            except:
                print("Invalid number.")
            continue

        if len(parts) == 2:
            try:
                t = float(parts[0])
                F = float(parts[1])
                thrust_profile[t] = F
            except:
                print("Invalid entry.")
            continue

        print("Invalid format.")

    if not thrust_profile:
        print("Using default thrust curve.")
        thrust_profile = {0: 50, 1: 150, 2: 120, 3: 80, 4: 0}

    return thrust_profile



G0 = 9.81
R_AIR = 287.05
L = 0.0065
T_0 = 288.15
P_0 = 101325



class Vehicle:
    def __init__(self, mass, diameter, Cd_rocket, Cd_parachute, parachute_area, thrust_profile):
        self.mass = mass
        self.area = np.pi * (diameter / 2) ** 2
        self.Cd_rocket = Cd_rocket
        self.Cd_parachute = Cd_parachute
        self.parachute_area = parachute_area
        self.thrust_profile = thrust_profile
        self.thrust_time_max = max(thrust_profile.keys())



def get_temperature(alt):
    return max(T_0 - L * alt, 216.65)


def get_density(alt, T):
    pressure = P_0 * (T / T_0) ** (G0 / (R_AIR * L))
    return pressure / (R_AIR * T)



def calculate_forces(time, altitude, velocity, vehicle):
    T = get_temperature(altitude)
    rho = get_density(altitude, T)

    thrust = np.interp(
        time,
        list(vehicle.thrust_profile.keys()),
        list(vehicle.thrust_profile.values())
    ) if time <= vehicle.thrust_time_max else 0

    Fg = vehicle.mass * G0

    if velocity < 0 and altitude < 900:
        Cd = vehicle.Cd_parachute
        area = vehicle.parachute_area
    else:
        Cd = vehicle.Cd_rocket
        area = vehicle.area

    drag = 0.5 * rho * velocity * abs(velocity) * Cd * area
    F_net = thrust - Fg - drag

    return F_net



def simulate(vehicle, dt, max_time):
    t = 0
    h = 0
    v = 0

    T_list, H_list, V_list, A_list = [], [], [], []

    while t < max_time:
        F = calculate_forces(t, h, v, vehicle)
        a = F / vehicle.mass

        v += a * dt
        h += v * dt
        t += dt

        if h < 0:
            h = 0
            break

        T_list.append(t)
        H_list.append(h)
        V_list.append(v)
        A_list.append(a)

    return np.array(T_list), np.array(H_list), np.array(V_list), np.array(A_list)



print("\n=== Load CSV or Manual Input ===")
filename = input("Enter CSV filename (or press Enter to skip): ").strip()

params, thrust_profile = load_from_csv(filename) if filename else (None, None)

if params is None:
    print("\n--- Manual Input Mode ---")
    params = {
        "mass": safe_float("Mass (kg) [0.5]: ", 0.5),
        "diameter": safe_float("Diameter (m) [0.065]: ", 0.065),
        "Cd_rocket": safe_float("Rocket Cd [0.5]: ", 0.5),
        "Cd_parachute": safe_float("Parachute Cd [1.5]: ", 1.5),
        "parachute_area": safe_float("Parachute area [0.2]: ", 0.2),
        "max_time": safe_float("Simulation time (s) [120]: ", 120),
        "dt": safe_float("Time step dt [0.01]: ", 0.01),
    }
    thrust_profile = get_thrust_profile_manual()

vehicle = Vehicle(
    params["mass"], params["diameter"], params["Cd_rocket"],
    params["Cd_parachute"], params["parachute_area"], thrust_profile
)

T, H, V, A = simulate(vehicle, params["dt"], params["max_time"])


plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(T, H)
plt.title("Altitude vs Time")
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(T, V)
plt.title("Velocity vs Time")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(T, A)
plt.title("Acceleration vs Time")
plt.xlabel("Time (s)")
plt.grid(True)

plt.tight_layout()
plt.show()

print("\nSimulation Complete!")
print(f"Max altitude reached: {max(H):.2f} m")
