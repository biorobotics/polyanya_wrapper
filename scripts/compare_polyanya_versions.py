import numpy as np

results_polyanya = np.load('results_original_epsilon.npy')
results_polyanya2 = np.load('results_polyanya2.npy')

not_nan_idx1 = np.where(np.logical_not(np.isnan(results_polyanya)))[0]
not_nan_idx2 = np.where(np.logical_not(np.isnan(results_polyanya2)))[0]
print(np.all(not_nan_idx1 == not_nan_idx2))
assert(np.all(results_polyanya[not_nan_idx1, :4] == results_polyanya2[not_nan_idx2, :4]))
not_inf_idx1 = np.isfinite(results_polyanya[not_nan_idx1, 4])
not_inf_idx2 = np.isfinite(results_polyanya2[not_nan_idx2, 4])
assert(np.all(not_inf_idx1 == not_inf_idx2))
print(np.amax(np.abs(results_polyanya[not_nan_idx1, 4][not_inf_idx1] - results_polyanya2[not_nan_idx2, 4][not_inf_idx1])))
