def get_master_drilling_state(row, prev_state):
    flow = row["MR_FLOW.M"]         # extract signal inputs from current row/data point
    rotation = row["MR_ROTATION"]
    rop = row["RT_ROP"]

    # define all valid transitions from each state, along with necessary conditions
    # Pumps Off
    transitions = {
        "Pumps Off": [
            # can only go to pumps on if flow is on, but rotation is off
            ("Pumps On", flow == 1 and rotation == 0)
        ],
        "Pumps On": [
            # return to Pumps Off if flow turns off
            ("Pumps Off", flow == 0 and rotation == 0),
            # transition to Sliding Drilling if drilling starts (ROP > 0) but no rotation
            ("Sliding Drilling", flow == 1 and rotation == 0 and rop > 0),
            # transition to Rotating if rotation starts, but no drilling yet
            ("Rotating", flow == 1 and rotation == 1 and rop == 0)
        ],
        "Sliding Drilling": [
            # return to Pumps On if drilling stops (ROP == 0)
            ("Pumps On", flow == 1 and rotation == 0 and rop == 0),
            # transition to Rotating if rotation is high and drilling stops (ROP == 0)
            ("Rotating", flow == 1 and rotation == 1 and rop == 0),
            ("Rotating", rotation == 1 and rop > 0) # needed if rop goes up while rotation is 0.
        ],
        "Rotating": [
            # return to Pumps On if rotation stops
            ("Pumps On", flow == 1 and rotation == 0),
            ("Pumps On", flow == 0 and rotation == 0),  # needed if flow goes down before rot
            # transition to Rotating Drilling if drilling starts while rotating (ROP > 0)
            ("Rotating Drilling", flow == 1 and rotation == 1 and rop > 0)
        ],
        "Rotating Drilling": [
            # drop back to Rotating if ROP stops but rotation remains
            ("Rotating", flow == 1 and rotation == 1 and rop == 0),
            ("Rotating", flow == 1 and rotation == 0 and rop == 0)
        ]
    }

    # loop through all valid transitions from the previous state
    for next_state, condition in transitions.get(prev_state, []):
        if condition:
            # if a condition is met, transition to the new state
            return next_state

    # if no valid condition is met, remain in the current state
    return prev_state
