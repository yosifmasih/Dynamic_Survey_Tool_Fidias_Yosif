import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from state_logic import compute_rolling_average, get_drilling_state, compute_rolling_variance
from master_state_logic import get_master_drilling_state
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
from datetime import datetime

# Load data
input_file = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WB Run 7\Flybar 1 WB Run 7 (Modified)_ShiftedData.xlsx"
df = pd.read_excel(input_file)
df["RT_Time"] = pd.to_datetime(df["RT_Time"], errors="coerce")
df = df.dropna(subset=["RT_Time"])
df.ffill(inplace=True)

use_time_filter = True
start_time = pd.to_datetime("2024-12-03 03:42:56")
end_time = pd.to_datetime("2024-12-04 18:25:28")

# Create timestamp string
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

if use_time_filter:
    df = df[(df["RT_Time"] >= start_time) & (df["RT_Time"] <= end_time)]

WINDOW_SIZE_VAR = 600  # Change when using feather data

# Compute rolling variance for vibration logic
df = compute_rolling_variance(df, WINDOW_SIZE_VAR)
# Compute total vibration magnitude and its rolling average and variance
df["Vib_Total"] = np.sqrt(df["MR_VIBA.MR"]**2 + df["MR_VIBL.MR"]**2)
df["Vib_Total_var"] = df["Vib_Total"].rolling(window=WINDOW_SIZE_VAR, min_periods=1).var()

# apply master state with transition logic
prev_master_state = "Pumps Off"
master_states = []
for _, row in df.iterrows():
    new_state = get_master_drilling_state(row, prev_master_state)
    master_states.append(new_state)
    prev_master_state = new_state
df["Master_State"] = master_states

# apply vibration state with transition logic
prev_vib_state = "Pumps Off"
vibration_states = []
for _, row in df.iterrows():
    new_state = get_drilling_state(row, prev_vib_state)
    vibration_states.append(new_state)
    prev_vib_state = new_state
df["Vibration_State"] = vibration_states

# Match comparison
df["Match"] = df["Master_State"] == df["Vibration_State"]

# Save the full DataFrame to Excel
df.to_excel(f"state_comparison_full_{timestamp}.xlsx", index=False)
print(f"Saved full comparison results to state_comparison_full.xlsx")

# Compute accuracy by state
results = []
for state in df["Master_State"].unique():
    if state == "Unknown":
        continue
    subset = df[df["Master_State"] == state]
    correct = subset["Match"].sum()
    total = len(subset)
    results.append({
        "State": state,
        "Total Points": total,
        "Correct": correct,
        "Incorrect": total - correct,
        "Accuracy (%)": round((correct / total) * 100, 2) if total else 0
    })

# Overall accuracy
total = df[df["Master_State"] != "Unknown"].shape[0]
correct = df[df["Master_State"] != "Unknown"]["Match"].sum()
results.append({
    "State": "Overall",
    "Total Points": total,
    "Correct": correct,
    "Incorrect": total - correct,
    "Accuracy (%)": round((correct / total) * 100, 2)
})

# Convert to DataFrame and print/save
results_df = pd.DataFrame(results)
results_df.to_excel(f"state_match_scoring_{timestamp}.xlsx", index=False)
print(results_df)

# Filter out Unknowns (if any remain)
filtered_df = df[(df["Master_State"] != "Unknown") & (df["Vibration_State"] != "Unknown")]

# Define label order
labels = ["Pumps Off", "Pumps On", "Sliding Drilling", "Rotating", "Rotating Drilling"]

# Compute and display confusion matrix
cm = confusion_matrix(filtered_df["Master_State"], filtered_df["Vibration_State"], labels=labels)
fig, ax = plt.subplots(figsize=(8, 6))
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=labels)
disp.plot(include_values=True, cmap="Blues", ax=ax, xticks_rotation=45)
plt.title("Confusion Matrix: Vibration vs Master State")
plt.tight_layout()
plt.show()
