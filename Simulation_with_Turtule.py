import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import turtle
import time
import sys


def load_from_csv(filename):
    if not os.path.exists(filename):
        print("\nCSV file not found. Switching to manual input...\n")
        return None, None

    params = {}
    thrust_profile = {}
    thrust_section = False

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
                    t = float(row[0]); F = float(row[1]); thrust_profile[t] = F
                except Exception:
                    print("⚠ invalid thrust row:", row)
                continue
            try:
                params[row[0]] = float(row[1])
            except Exception:
                print("⚠ invalid param row:", row)

    required = ["mass", "diameter", "Cd_rocket", "Cd_parachute",
                "parachute_area", "max_time", "dt"]
    for k in required:
        if k not in params:
            raise ValueError(f"CSV missing parameter: {k}")

    if not thrust_profile:
        print("⚠ No thrust rows found in CSV; using default thrust curve.")
        thrust_profile = {0:50, 1:150, 2:120, 3:80, 4:0}

    return params, thrust_profile


def safe_float(prompt, default):
    s = input(prompt); return default if s.strip()=="" else float(s)

def get_thrust_profile_manual():
    print("Enter thrust profile (time thrust) lines, 'done' to finish:")
    tp = {}; auto=0
    while True:
        s = input("Time Thrust: ")
        if s.lower().strip()=="done": break
        parts = s.split()
        if len(parts)==1:
            try:
                F = float(parts[0]); auto += 1; tp[auto]=F
            except: print("invalid")
        elif len(parts)==2:
            try:
                t=float(parts[0]); F=float(parts[1]); tp[t]=F
            except: print("invalid")
        else:
            print("format error")
    if not tp: tp={0:50,1:150,2:120,3:80,4:0}
    return tp


G0 = 9.81
R_AIR = 287.05
L = 0.0065
T_0 = 288.15
P_0 = 101325


class Vehicle:
    def __init__(self, mass, diameter, Cd_rocket, Cd_parachute, parachute_area, thrust_profile):
        self.mass = mass
        self.area = np.pi*(diameter/2)**2
        self.Cd_rocket = Cd_rocket
        self.Cd_parachute = Cd_parachute
        self.parachute_area = parachute_area
        self.thrust_profile = dict(sorted(thrust_profile.items()))
        self.thrust_time_max = max(self.thrust_profile.keys())


def get_temperature(alt): return max(T_0 - L*alt, 216.65)
def get_density(alt, T):
    pressure = P_0 * (T / T_0) ** (G0 / (R_AIR * L))
    return pressure / (R_AIR * T)

def calculate_forces(time, altitude, velocity, vehicle):
    T = get_temperature(altitude)
    rho = get_density(altitude, T)
    thrust = np.interp(time, list(vehicle.thrust_profile.keys()), list(vehicle.thrust_profile.values())) if time <= vehicle.thrust_time_max else 0.0
    weight = vehicle.mass * G0
    if velocity < 0 and altitude < 900:
        Cd = vehicle.Cd_parachute; A = vehicle.parachute_area
    else:
        Cd = vehicle.Cd_rocket; A = vehicle.area
    drag = 0.5 * rho * velocity * abs(velocity) * Cd * A
    return thrust - weight - drag


def simulate(vehicle, dt, max_time):
    t = 0.0; h = 0.0; v = 0.0
    T_hist, H_hist, V_hist, A_hist = [], [], [], []
    steps = int(max_time / dt) + 1
    for _ in range(steps):
        F = calculate_forces(t, h, v, vehicle)
        a = F / vehicle.mass
        v += a * dt
        h += v * dt
        t += dt
        if h < 0:
            h = 0.0
            T_hist.append(t); H_hist.append(h); V_hist.append(v); A_hist.append(a)
            break
        T_hist.append(t); H_hist.append(h); V_hist.append(v); A_hist.append(a)
    return np.array(T_hist), np.array(H_hist), np.array(V_hist), np.array(A_hist)


def upsample_array(y, target_len):
    if len(y) == 0: return y
    x_old = np.linspace(0, 1, len(y))
    x_new = np.linspace(0, 1, target_len)
    return np.interp(x_new, x_old, y)


def turtle_animation(altitudes, fps=60, use_fullscreen=True):
    if len(altitudes) == 0 or max(altitudes) <= 0.001:
        print("No useful altitude; skipping Turtle animation.")
        return

    
    turtle.TurtleScreen._RUNNING = True

    
    upsample_factor = 6
    frames = upsample_array(altitudes, max(len(altitudes)*upsample_factor, 300)).tolist()

    
    try:
        screen = turtle.Screen()
        screen.title("Rocket Launch (final fixed)")
        screen.bgcolor("black")
        if use_fullscreen:
            try:
                screen.setup(width=1.0, height=1.0)  
            except Exception:
                
                screen.setup(width=1000, height=700)
        else:
            screen.setup(width=1000, height=700)
    except turtle.Terminator:
        print("Turtle could not start.")
        return
    except Exception as e:
        print("Turtle screen error:", e)
        return

   
    try:
        screen_w = screen.window_width()
        screen_h = screen.window_height()
    except Exception:
        screen_w, screen_h = 1000, 700

    
    bottom_y = - (screen_h // 2) + 80   
    top_y    =   (screen_h // 2) - 140  

    max_alt = max(altitudes)
    if max_alt <= 0: max_alt = 1.0

    
    base_scale = 200.0
    scale_factor = max(0.5, min(3.0, base_scale / max_alt))
    rocket_w = max(0.6, 0.9 * scale_factor)
    rocket_l = max(1.0, 1.6 * scale_factor)
    parachute_dot = int(max(8, min(40, 30 * (base_scale / max_alt))))

    
    try:
        rocket = turtle.Turtle()
        rocket.hideturtle()
        rocket.shape("triangle")
        rocket.shapesize(stretch_wid=rocket_w, stretch_len=rocket_l)
        rocket.color("white")
        rocket.penup()
        rocket.setheading(90)
        rocket.showturtle()

        chute = turtle.Turtle()
        chute.hideturtle()
        chute.penup()
        chute.color("yellow")
    except turtle.Terminator:
        print("Turtle init aborted.")
        try:
            turtle.bye()
        except Exception:
            pass
        return

   
    print(f"[visual debug] screen_h={screen_h}, screen_w={screen_w}, max_alt={max_alt:.3f}, scale_factor={scale_factor:.3f}")

    
    def scale_y(alt):
        
        a = max(0.0, min(max_alt, float(alt)))
        return bottom_y + (a / max_alt) * (top_y - bottom_y)

    apex_idx = int(np.argmax(altitudes) * upsample_factor)
    delay = 1.0 / fps

    try:
        for i, alt in enumerate(frames):
            if not getattr(turtle.TurtleScreen, "_RUNNING", True):
                print("Turtle window closed early, stopping animation.")
                return

            y = scale_y(alt)
            rocket.goto(0, y)

            if i < apex_idx:
                
                rocket.color("white")
                chute.clear()
            else:
                
                rocket.color("yellow")
                chute.clear()
                chute.goto(0, y + 34 * (scale_factor/1.5))
                chute.dot(parachute_dot, "yellow")
                chute.goto(0, y + 18 * (scale_factor/1.5))
                chute.pendown()
                try:
                    chute.width(max(1, int(scale_factor/2)))
                except Exception:
                    pass
                chute.goto(0, y)
                chute.penup()

            time.sleep(delay)

    except turtle.Terminator:
        print("Turtle window closed (Terminator). Continuing.")
        try:
            turtle.bye()
        except Exception:
            pass
        return
    except Exception as e:
        print("Error in turtle animation:", e)
        try:
            turtle.bye()
        except Exception:
            pass
        return

   
    try:
        turtle.bye()
    except Exception:
        pass


def save_simulation_csv(filename, T, H, V, A):
    try:
        with open(filename, "w", newline='') as f:
            w = csv.writer(f)
            w.writerow(["time_s", "altitude_m", "velocity_m_s", "acceleration_m_s2"])
            for i in range(len(T)):
                w.writerow([f"{T[i]:.6f}", f"{H[i]:.6f}", f"{V[i]:.6f}", f"{A[i]:.6f}"])
        print("Saved simulation data to:", filename)
    except Exception as e:
        print("Could not save CSV:", e)


def main():
    print("=== Load CSV or Manual Input ===")
    filename = input("Enter CSV filename (or press Enter to skip): ").strip()
    params = None; thrust_profile = None

    if filename:
        try:
            params, thrust_profile = load_from_csv(filename)
        except Exception as e:
            print("CSV load error:", e)
            return

    if params is None:
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

    vehicle = Vehicle(params["mass"], params["diameter"], params["Cd_rocket"],
                      params["Cd_parachute"], params["parachute_area"], thrust_profile)

    T, H, V, A = simulate(vehicle, params["dt"], params["max_time"])

    print(f"Simulation steps: {len(T)}  Max altitude: {max(H):.2f} m")

    save = input("Save simulation data to CSV? (y/N): ").strip().lower()
    if save == "y":
        outname = input("Enter output CSV filename [simulation_out.csv]: ").strip() or "simulation_out.csv"
        save_simulation_csv(outname, T, H, V, A)

    
    turtle_animation(H, fps=60, use_fullscreen=True)

    
    if len(T) > 0:
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1); plt.plot(T, H); plt.title("Altitude vs Time"); plt.ylabel("Altitude (m)"); plt.grid(True)
        plt.subplot(3, 1, 2); plt.plot(T, V); plt.title("Velocity vs Time"); plt.ylabel("Velocity (m/s)"); plt.grid(True)
        plt.subplot(3, 1, 3); plt.plot(T, A); plt.title("Acceleration vs Time"); plt.xlabel("Time (s)"); plt.ylabel("Acceleration (m/s²)"); plt.grid(True)
        plt.tight_layout(); plt.show()

    print("Simulation Complete.")

if __name__ == "__main__":
    main()
