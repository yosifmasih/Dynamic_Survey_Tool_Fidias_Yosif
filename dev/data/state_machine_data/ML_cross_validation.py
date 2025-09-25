import os
import pandas as pd
import numpy as np
from xgboost import XGBClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from master_state_logic import get_master_drilling_state
from datetime import datetime
import matplotlib.pyplot as plt
import seaborn as sns
import joblib

# constants like windows and legal transitions
WINDOW_SIZE_VAR = 75
state_to_num = {
    "Pumps Off": 0,
    "Pumps On": 1,
    "Sliding Drilling": 2,
    "Rotating": 3,
    "Rotating Drilling": 4
}
num_to_state = {v: k for k, v in state_to_num.items()}
legal_transitions = {
    "Pumps Off": ["Pumps On"],
    "Pumps On": ["Pumps Off", "Sliding Drilling", "Rotating"],
    "Sliding Drilling": ["Pumps On", "Rotating"],
    "Rotating": ["Pumps On", "Rotating Drilling"],
    "Rotating Drilling": ["Rotating"]
}

# better way to store files and their starting/ending times
WB1_7 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WB Run 7\state_vibration_analysis_WB1_7.xlsx"
WC1_6 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WC Run 6\state_vibration_analysis_WC1_6.xlsx"
WC1_7 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WC Run 7\state_vibration_analysis_WC1_7.xlsx"
WC1_8 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar1 WC Run 8\state_vibration_analysis_WC1_8.xlsx"
WC2_5 = r"C:\Users\fidia\OneDrive - Texas A&M University\Spring 2025\ECEN 403\Modified Field Data\Flybar2 WC Run 5\state_vibration_analysis_WC2_5.xlsx"

file_info = [
    (WB1_7, "2024-12-03 03:42:56", "2024-12-04 18:25:27"),
    (WC1_6, "2024-12-10 03:21:52", "2024-12-10 08:59:52"),
    (WC1_7, "2024-12-10 16:14:40", "2024-12-11 11:04:19"),
    (WC1_8, "2024-12-11 18:53:05", "2024-12-12 18:43:12"),
    (WC2_5, "2024-11-24 03:53:05", "2024-11-28 10:13:17")
]
# output directories to hold all tests
os.makedirs("saved_models", exist_ok=True)
os.makedirs("confusion_matrices", exist_ok=True)
os.makedirs("reports", exist_ok=True)
os.makedirs("excel_logs", exist_ok=True)


# helper function that will load and preprocess a single file
def load_and_preprocess(file_path, start, end):
    df = pd.read_excel(file_path)
    df.dropna(subset=["RT_Time"], inplace=True)
    df["RT_Time"] = pd.to_datetime(df["RT_Time"])
    df["Master_Label"] = df["Master_State"].map(state_to_num)
    return df


# start of the CV loop
for i in range(len(file_info)):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    test_file, start, end = file_info[i]
    train_files = file_info[:i] + file_info[i + 1:]

    # call helper function to load the training data
    train_df = pd.concat([load_and_preprocess(f, s, e) for f, s, e in train_files], ignore_index=True)
    test_df = load_and_preprocess(test_file, start, end)

    # again vibration features that model will train on
    features = [
        "MR_VIBA.MR", "MR_VIBL.MR", "Vib_Total",
        "MR_VIBA.MR_var", "MR_VIBL.MR_var", "Vib_Total_var",
        "MR_FLOW.M", "MR_ROTATION"
    ]
    X_train = train_df[features].fillna(0)
    y_train = train_df["Master_Label"]
    X_test = test_df[features].fillna(0)
    y_test = test_df["Master_Label"]

    # model actually training using decision tree with 50 trees with 4 yes/no questions
    model = XGBClassifier(n_estimators=50, max_depth=4, use_label_encoder=False, eval_metric='mlogloss')
    model.fit(X_train, y_train)

    # now we can test using the trained model
    test_df["ML_Predicted_Label"] = model.predict(X_test)
    test_df["ML_Predicted_State"] = test_df["ML_Predicted_Label"].map(num_to_state)
    test_df["Match"] = test_df["ML_Predicted_State"] == test_df["Master_State"]

    # here we apply the legal transitions set in place by the FSM keeping logic intact
    filtered = []
    prev = "Pumps Off"
    broken_count = 0
    for _, row in test_df.iterrows():
        pred = row["ML_Predicted_State"]
        flow = row["MR_FLOW.M"]
        rot = row["MR_ROTATION"]
        allowed = legal_transitions.get(prev, [])
        # critical override changes
        if flow == 0 and rot == 0 and prev in ["Rotating", "Rotating Drilling"]:
            filtered.append("Pumps On")
            prev = "Pumps On"
            broken_count += 1
        elif prev == "Pumps On" and flow == 0 and rot == 0:
            filtered.append("Pumps Off")
            prev = "Pumps Off"
        elif flow == 1 and rot == 1 and prev == "Sliding Drilling":
            filtered.append("Rotating")
            prev = "Rotating"
            broken_count += 1
        # allows transition if its legal
        elif pred in allowed:
            filtered.append(pred)
            prev = pred
        # helper functions to move from pumps on to a drilling state
        elif prev == "Pumps On" and flow == 1 and rot == 1 and pred == "Rotating Drilling":
            filtered.append("Rotating")
            prev = "Rotating"
            broken_count += 1
        elif prev == "Pumps On" and flow == 1 and rot == 0 and pred == "Rotating Drilling":
            filtered.append("Sliding Drilling")
            prev = "Sliding Drilling"
            broken_count += 1
        # another helper check to move from rotating drilling to sliding drilling
        elif prev == "Rotating Drilling" and flow == 1 and rot == 0 and (pred == "Sliding Drilling" or pred == "Pumps On"):
            filtered.append("Rotating")
            prev = "Rotating"
            broken_count += 1
        # another critical check to make sure we are transitioning to pumps on if rotation goes low and we are in rotating
        elif prev == "Rotating" and flow == 1 and rot == 0:
            filtered.append("Pumps On")
            prev = "Pumps On"
        else:
            filtered.append(prev)

    test_df["ML_Filtered_State"] = filtered
    test_df["Filtered_Match"] = test_df["ML_Filtered_State"] == test_df["Master_State"]

    # performance report on how well model did with basic matches and filtered matches (printed and saved)
    report_path = f"reports/ML_fold{i + 1}_report_{timestamp}.txt"
    with open(report_path, "w") as f:
        f.write("Classification Report (Raw ML Prediction vs Master):\n")
        f.write(classification_report(test_df["Master_State"], test_df["ML_Predicted_State"], target_names=state_to_num.keys()))
        f.write("\n\nClassification Report (Filtered ML vs Master):\n")
        f.write(classification_report(test_df["Master_State"], test_df["ML_Filtered_State"], target_names=state_to_num.keys()))
        f.write(f"\n\nBroken Transitions Avoided: {broken_count}\n")
    # confusion matrix to better contextualize data
    cm = confusion_matrix(test_df["Master_State"], test_df["ML_Predicted_State"])
    plt.figure(figsize=(6, 5))
    sns.heatmap(cm, annot=True, fmt="d", cmap="Blues", xticklabels=state_to_num.keys(), yticklabels=state_to_num.keys())
    plt.xlabel("Predicted")
    plt.ylabel("Actual")
    plt.title(f"Confusion Matrix - Fold {i + 1}")
    plt.tight_layout()
    plt.savefig(f"confusion_matrices/conf_matrix_fold_{i + 1}_{timestamp}.png")
    plt.close()

    test_df.to_excel(f"excel_logs/fold_{i + 1}_predictions_{timestamp}.xlsx", index=False)
    joblib.dump(model, f"saved_models/model_fold_{i + 1}_{timestamp}.joblib")
    print(f"Fold {i + 1} complete. Test file: {test_file}. Broken transitions: {broken_count}")
