
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

BARO_FILE = "../data/barometer.csv"
EST_FILE = "../data/estimated_state.csv"

bar = pd.read_csv(BARO_FILE)
est = pd.read_csv(EST_FILE)

bar = bar.sort_values("sys_time_us")
est = est.sort_values("sys_time_us")

# Konverzija u min
t0 = min(bar["sys_time_us"].iloc[0], est["sys_time_us"].iloc[0])

bar["time_s"] = (bar["sys_time_us"] - t0) / 1e6
est["time_s"] = (est["sys_time_us"] - t0) / 1e6

# #--------------------------------------------------
# Usporedba barometarskih mjerenja i estimacije visine
# # --------------------------------------------------

plt.figure(figsize=(14, 6))

plt.plot(
            bar["time_s"],
            bar["baro_alt_m"],
            label="Mjerenje Barometra",
            linewidth=1.5,
            marker=".",
            markersize=3
        )

plt.plot(
            est["time_s"],
            est["est_altitude"],
            label="Estimacija visine",
            linewidth=1
        )

plt.xlabel("Vrijeme [s]")
plt.ylabel("Visina [m]")
plt.title("Mjerenja barometra i Estimacija")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig("bar_alt_comp.png", dpi=150)

# -----------------------------------------------------------
# Interpolacija estimirane visine u vrem. trenucima mjerenja
# -----------------------------------------------------------

est_alt_at_bar = np.interp(bar["sys_time_us"], est["sys_time_us"], est["est_altitude"])

# Razlika između mjerenja i estimacije
alt_error = bar["baro_alt_m"].to_numpy() - est_alt_at_bar

# --------------------------------------------------
# Graf odstupanja mjerenja i estimacije
# --------------------------------------------------

plt.figure(figsize=(14, 5))

plt.plot(
        bar["time_s"],
        alt_error,
        label="Barometer - Estimacija Visine",
        linewidth=1
        )

plt.axhline(0, color="black", linestyle="--", linewidth=0.8)

plt.xlabel("Vrijeme [s]")
plt.ylabel("Razlika [m]")
plt.title("Odstupanje Estimacije Visine")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig("alt_error.png", dpi=150)

# --------------------------------------------------
# Graf estimirane brzine
# --------------------------------------------------

plt.figure(figsize=(14, 5))

plt.plot(
        est["time_s"],
        est["est_velocity"],
        label="Estimacija Brzine",
        linewidth=0.8
        )

plt.xlabel("Vrijeme [s]")
plt.ylabel("Brzina [m/s]")
plt.title("Estimacija Vertikalne Brzine")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig("est_v.png", dpi=150)

# --------------------------------------------------
# Statistika odstupanja
# --------------------------------------------------

print("Statistika Odstupanja:")
print(f"Prosječno odstupanje:   {np.mean(alt_error):.4f} m")
print(f"aps. vr. pr. ods.:  {np.mean(np.abs(alt_error)):.4f} m")
print(f"RMSE:             {np.sqrt(np.mean(alt_error**2)):.4f} m")
print(f"Najvece odstupanje: {np.max(np.abs(alt_error)):.4f} m")

plt.show()
