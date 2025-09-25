import xgboost as xgb

bst = xgb.Booster()
bst.load_model(r"C:\Users\fidia\OneDrive - Texas A&M University\Fall 2025\ECEN 404\Software\State machine\saved_models\3rd test\model_fold_1_2025-06-25_20-15-31.json")
bst.dump_model("model_fold_1_2025-06-25_20-15-31.raw.txt", with_stats= False)