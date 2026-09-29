"""
Train the junction speed-modulation models.

  Model A (x2): LightGBM quantile regressors (q50, q90) predicting the RESIDUAL
                over the physics estimate:   t_clear_peer = t_peer_clear_kin + residual
  Model B     : LightGBM classifier for conflict probability r_hat

Usage:  python train_model.py --data amr_junction_dataset.csv --out models/
"""
import argparse, json, os
import joblib
import numpy as np
import pandas as pd
import lightgbm as lgb
from sklearn.metrics import (average_precision_score, brier_score_loss,
                             recall_score, precision_score)

NON_FEATURES = {"episode_id", "layout_id", "split"}


def get_features(df):
    return [c for c in df.columns if not c.startswith("y_") and c not in NON_FEATURES]


def train_quantile(X_tr, y_tr, X_va, y_va, q):
    m = lgb.LGBMRegressor(
        objective="quantile", alpha=q,
        n_estimators=1500, learning_rate=0.03,
        num_leaves=31, min_child_samples=50,       # limits overfitting
        subsample=0.8, subsample_freq=1, colsample_bytree=0.8,
        reg_lambda=1.0, verbose=-1)
    m.fit(X_tr, y_tr, eval_set=[(X_va, y_va)],
          callbacks=[lgb.early_stopping(50, verbose=False)])
    return m


def main(a):
    os.makedirs(a.out, exist_ok=True)
    df = pd.read_csv(a.data)
    feats = get_features(df)
    tr, va, te = (df[df.split == s] for s in ("train", "val", "test"))
    print(f"features={len(feats)}  train={len(tr)}  val={len(va)}  test={len(te)}")

    # ---------------- Model A: time-to-clear (residual, q50 + q90) ----------
    m50 = train_quantile(tr[feats], tr.y_residual, va[feats], va.y_residual, 0.5)
    m90 = train_quantile(tr[feats], tr.y_residual, va[feats], va.y_residual, 0.9)

    kin = te.t_peer_clear_kin.values
    p50 = kin + m50.predict(te[feats])
    p90 = np.maximum(kin + m90.predict(te[feats]), p50)      # enforce q90 >= q50
    y = te.y_t_clear_peer.values

    metrics = {
        "ttc_mae_physics_only_s": float(np.abs(y - kin).mean()),
        "ttc_mae_q50_s": float(np.abs(y - p50).mean()),
        "ttc_q90_coverage": float((y <= p90).mean()),       # target ~0.90
        "ttc_q90_mean_margin_s": float((p90 - p50).mean()),
    }

    # ---------------- Model B: conflict probability -------------------------
    clf = lgb.LGBMClassifier(
        n_estimators=1000, learning_rate=0.03, num_leaves=31,
        min_child_samples=50, subsample=0.8, subsample_freq=1,
        colsample_bytree=0.8, reg_lambda=1.0, verbose=-1)
    clf.fit(tr[feats], tr.y_conflict, eval_set=[(va[feats], va.y_conflict)],
            callbacks=[lgb.early_stopping(50, verbose=False)])

    # pick the threshold on VALIDATION so recall on missed conflicts >= 0.95
    pv = clf.predict_proba(va[feats])[:, 1]
    thr = 0.5
    for t in np.linspace(0.5, 0.01, 50):
        if recall_score(va.y_conflict, pv >= t) >= 0.95:
            thr = float(t); break
    pt = clf.predict_proba(te[feats])[:, 1]
    metrics.update({
        "conflict_pr_auc": float(average_precision_score(te.y_conflict, pt)),
        "conflict_brier": float(brier_score_loss(te.y_conflict, pt)),
        "conflict_threshold": thr,
        "conflict_recall_at_thr": float(recall_score(te.y_conflict, pt >= thr)),
        "conflict_precision_at_thr": float(precision_score(te.y_conflict, pt >= thr)),
        "conflict_base_rate": float(te.y_conflict.mean()),
    })

    # ---------------- save --------------------------------------------------
    joblib.dump(m50, f"{a.out}/ttc_q50.pkl")
    joblib.dump(m90, f"{a.out}/ttc_q90.pkl")
    joblib.dump(clf, f"{a.out}/conflict_clf.pkl")
    json.dump({"features": feats, "conflict_threshold": thr},
              open(f"{a.out}/meta.json", "w"), indent=2)
    json.dump(metrics, open(f"{a.out}/metrics.json", "w"), indent=2)

    print("\n=== TEST METRICS (unseen layout) ===")
    for k, v in metrics.items():
        print(f"{k:30s} {v:.3f}")

    imp = pd.Series(m50.feature_importances_, feats).sort_values(ascending=False)
    print("\nTop features (q50 model):\n", imp.head(8).to_string())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="amr_junction_dataset.csv")
    ap.add_argument("--out", default="models")
    main(ap.parse_args())
