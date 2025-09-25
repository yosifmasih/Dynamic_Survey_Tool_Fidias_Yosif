def compute_rolling_average(df, window):
    for col in ["MR_VIBA.MR", "MR_VIBL.MR"]:
        df[f"{col}_avg"] = df[col].rolling(window=window, min_periods=1).mean()
    return df

def compute_rolling_variance(df, window):
    for col in ["MR_VIBA.MR", "MR_VIBL.MR"]:
        df[f"{col}_var"] = df[col].rolling(window=window, min_periods=1).var()
    return df

def get_drilling_state(row, prev_state):
    flow = row["MR_FLOW.M"]
    rotation = row["MR_ROTATION"]
    vibA = row["MR_VIBA.MR"]
    vibL = row["MR_VIBL.MR"]
    vib_total = row["Vib_Total"]
    vibA_var = row["MR_VIBA.MR_var"]
    vibL_var = row["MR_VIBL.MR_var"]
    vib_total_var = row["Vib_Total_var"]

    # pumps off to pumps on
    if prev_state == "Pumps Off" and flow == 1 and rotation == 0:
        return "Pumps On"

    # pumps on to pumps off
    if prev_state == "Pumps On" and flow == 0 and rotation == 0:
        return "Pumps Off"

    # Pumps on/sldiing drilling to rotating
    if prev_state in ["Pumps On", "Sliding Drilling"] and flow == 1 and rotation == 1:
        return "Rotating"

    # rotating to pumps on
    if prev_state == "Rotating" and rotation == 0:
        return "Pumps On"

    # classify pumps on or sliding drilling
    if prev_state in ["Pumps On", "Sliding Drilling"] and flow == 1 and rotation == 0:
        scores = {"Pumps On": 0, "Sliding Drilling": 0}
        # Points for Sliding Drilling
        if vib_total > 1.0: scores["Sliding Drilling"] += 1
        if vib_total_var > 0.01: scores["Sliding Drilling"] += 1
        if vibL > 1.0: scores["Sliding Drilling"] += 1
        if vibL_var > 0.01: scores["Sliding Drilling"] += 1
        # Points for Pumps On (inverse conditions)
        if vib_total <= 1.0: scores["Pumps On"] += 1
        if vib_total_var <= 0.01: scores["Pumps On"] += 1
        if vibL <= 1.0: scores["Pumps On"] += 1
        if vibL_var <= 0.01: scores["Pumps On"] += 1

        return max(scores, key=scores.get)

    # classify rotating vs rotating drilling
    if flow == 1 and rotation == 1:
        scores = {"Rotating": 0, "Rotating Drilling": 0}
        # Points for Rotating Drilling
        # if prev_state == "Rotating": scores["Rotating"] += 1.1
        if vib_total > 1.4: scores["Rotating"] += 1
        if vib_total_var > 0.025: scores["Rotating"] += 1
        if vibA > 1.0: scores["Rotating"] += 1
        # Points for Rotating Drilling (inverse)
        # if prev_state == "Rotating Drilling": scores["Rotating Drilling"] += 1.1
        if vib_total <= 1.4: scores["Rotating Drilling"] += 1
        if vib_total_var <= 0.025: scores["Rotating Drilling"] += 1
        if vibA <= 1.0: scores["Rotating Drilling"] += 1

        return max(scores, key=scores.get)

    if flow == 0 and rotation == 1:

        return "Rotating"

    if flow == 1 and rotation == 0:

        return "Rotating"

    return prev_state
