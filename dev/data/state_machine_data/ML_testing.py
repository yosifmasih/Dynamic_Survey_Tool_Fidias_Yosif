import pandas as pd
import numpy as np
from xgboost import XGBClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
from master_state_logic import get_master_drilling_state
from datetime import datetime

# Load file
WB1_7 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WB Run 7\Flybar 1 WB Run 7 (Modified)_ShiftedData.xlsx"
WC1_6 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WC Run 6\Flybar1 WC Run 6_ShiftedData.xlsx"
WC1_7 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WC Run 7\Flybar1 WC Run 7_ShiftedData.xlsx"
WC1_8 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WC Run 8\Flybar1 WC Run 8_ShiftedData.xlsx"
WC2_5 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar2 WC Run 5\Flybar2 WC Run 5_ShiftedData.xlsx"

df = pd.read_excel(WB1_7)
df["RT_Time"] = pd.to_datetime(df["RT_Time"], errors="coerce")
df = df.dropna(subset=["RT_Time"])
df.ffill(inplace=True)

# time filter file
use_time_filter = True
start_time = pd.to_datetime("2024-12-03 03:42:56")
end_time = pd.to_datetime("2024-12-04 18:25:28")
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

if use_time_filter:
    df = df[(df["RT_Time"] >= start_time) & (df["RT_Time"] <= end_time)]

# compute total vibration
df["Vib_Total"] = np.sqrt(df["MR_VIBA.MR"]**2 + df["MR_VIBL.MR"]**2)

# rolling statistics (var and avg)
WINDOW_SIZE_VAR = 600

df["MR_VIBA.MR_var"] = df["MR_VIBA.MR"].rolling(window=WINDOW_SIZE_VAR, min_periods=1).var()
df["MR_VIBL.MR_var"] = df["MR_VIBL.MR"].rolling(window=WINDOW_SIZE_VAR, min_periods=1).var()
df["Vib_Total_var"] = df["Vib_Total"].rolling(window=WINDOW_SIZE_VAR, min_periods=1).var()

# apply master state with transition logic
prev_state = "Pumps Off"
master_states = []
for _, row in df.iterrows():
    new_state = get_master_drilling_state(row, prev_state)
    master_states.append(new_state)
    prev_state = new_state
df["Master_State"] = master_states

# master states to numbers
state_to_num = {
    "Pumps Off": 0,
    "Pumps On": 1,
    "Sliding Drilling": 2,
    "Rotating": 3,
    "Rotating Drilling": 4
}
# filter valid for ML training
df = df[df["Master_State"].isin(state_to_num.keys())]

df["Master_Label"] = df["Master_State"].map(state_to_num)

# label which columns in data set will be used
features = [
    "MR_VIBA.MR", "MR_VIBL.MR", "Vib_Total",
    "MR_VIBA.MR_var", "MR_VIBL.MR_var", "Vib_Total_var",
    "MR_FLOW.M", "MR_ROTATION"
]
# X is table that holds column listed in 'features' also fills blank spaces with 0
X = df[features].fillna(0)
y = df["Master_Label"]      # what we are tyring to teach the model

# train/test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# train model using XGBoost (decision tree) using 50 trees with 4 yes/no questions
model = XGBClassifier(n_estimators=50, max_depth=4, use_label_encoder=False, eval_metric='mlogloss')
model.fit(X_train, y_train)

# here we actually let the model predict on 20% of the data
y_pred = model.predict(X_test)
df["ML_Predicted_Label"] = model.predict(X)
df["ML_Predicted_State"] = df["ML_Predicted_Label"].map({v: k for k, v in state_to_num.items()})
df["Match"] = df["Master_State"] == df["ML_Predicted_State"]

# making sure that the ML model only does legal transitions
legal_transitions = {
    "Pumps Off": ["Pumps On"],
    "Pumps On": ["Pumps Off", "Sliding Drilling", "Rotating"],
    "Sliding Drilling": ["Pumps On", "Rotating"],
    "Rotating": ["Pumps On", "Rotating Drilling"],
    "Rotating Drilling": ["Rotating"]
}

filtered_states = []
prev = "Pumps Off"
broken_transition_counter = 0
for i, predicted in enumerate(df["ML_Predicted_State"]):
    flow = df.iloc[i]["MR_FLOW.M"]
    rotation = df.iloc[i]["MR_ROTATION"]
    allowed = legal_transitions.get(prev, [])

    # edge case found in field data: allow fallback to Pumps On if both flow and rotation drop
    if flow == 0 and rotation == 0 and prev in ["Rotating", "Rotating Drilling"]:
        filtered_states.append("Pumps On")
        prev = "Pumps On"
        broken_transition_counter += 1

    elif flow == 1 and rotation == 1 and prev == "Sliding Drilling":
        filtered_states.append("Rotating")
        prev = "Rotating"
        broken_transition_counter += 1

    elif predicted in legal_transitions[prev]:
        filtered_states.append(predicted)
        prev = predicted
    else:
        filtered_states.append(prev)
        # broken_transition_counter += 1

df["ML_Filtered_State"] = filtered_states
df["Filtered_Match"] = df["Master_State"] == df["ML_Filtered_State"]

# performance report on how well model did with basic matches and filtered matches (printed and saved)
report_path = f"ML_classification_report_{timestamp}.txt"
with open(report_path, "w") as f:
    f.write("Classification Report (Raw ML Prediction vs Master):\n")
    f.write(classification_report(y_test, y_pred, target_names=state_to_num.keys()))

    f.write("\n\nClassification Report (Filtered ML vs Master):\n")
    f.write(classification_report(df["Master_State"], df["ML_Filtered_State"], target_names=state_to_num.keys()))

    f.write("\n\nClassification Report (Filtered ML vs Master, Test Set Only):\n")
    df_test = df.loc[y_test.index]
    f.write(classification_report(df_test["Master_State"], df_test["ML_Filtered_State"], target_names=state_to_num.keys()))

    f.write(f"\n\nNumber of broken transitions avoided: {broken_transition_counter}\n")

print("\nClassification Report (Raw ML Prediction vs Master):\n")
print(classification_report(y_test, y_pred, target_names=state_to_num.keys()))
print("\nClassification Report (Filtered ML vs Master):\n")
print(classification_report(df["Master_State"], df["ML_Filtered_State"], target_names=state_to_num.keys()))
print("\nClassification Report (Filtered ML vs Master, Test Set Only):\n")
print(classification_report(df_test["Master_State"], df_test["ML_Filtered_State"], target_names=state_to_num.keys()))

# confusion matrix
cm = confusion_matrix(y_test, y_pred)
plt.figure(figsize=(6, 5))
sns.heatmap(cm, annot=True, fmt="d", cmap="Blues", xticklabels=state_to_num.keys(), yticklabels=state_to_num.keys())
plt.xlabel("Predicted")
plt.ylabel("Actual")
plt.title("Confusion Matrix - ML Model vs Master State")
plt.tight_layout()

# save confusion matrix and full prediction results
matrix_path = f"confusion_matrix_{timestamp}.png"
plt.savefig(matrix_path)
plt.close()

excel_path = f"ML_predictions_{timestamp}.xlsx"
df.to_excel(excel_path, index=False)

print(f"Confusion matrix saved to: {matrix_path}")
print(f"Excel predictions saved to: {excel_path}")
print(f"Classification report saved to: {report_path}")
print(f"Number of broken transitions avoided: {broken_transition_counter}")
