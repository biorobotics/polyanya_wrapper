import numpy as np

results_new_epsilon = np.load('results_new_epsilon.npy')
results_original_epsilon = np.load('results_original_epsilon.npy')
not_nan_idx = np.logical_not(np.isnan(results_original_epsilon[:, 4]))
inf_idx = np.isinf(results_original_epsilon[:, 4])
assert(np.all(np.isinf(results_new_epsilon[inf_idx, 4])))
finite_idx = np.isfinite(results_new_epsilon[:, 4])
mask = np.logical_and(finite_idx, not_nan_idx)
print('Max absolute value of difference in path length after changing COLLINEARITY_EPSILON: %f' %(np.amax(np.abs(results_new_epsilon[mask, 4] - results_original_epsilon[mask, 4]))))
argmax = np.where(mask)[0][np.argmax(np.abs(results_new_epsilon[mask, 4] - results_original_epsilon[mask, 4]))]
print('Test index of max difference: %d' %(argmax%100)) # Test index
print('Obstacle map index of max difference: %d' %(argmax//100)) # Obstacle map index
print('Path length with new COLLINEARITY_EPSILON is %f, and with old COLLINEARITY_EPSILON is %f' %(results_new_epsilon[argmax, 4], results_original_epsilon[argmax, 4]))
print('Ratio of path length with new COLLINEARITY_EPSILON to path length with old COLLINEARITY_EPSILON is %f' %(results_new_epsilon[argmax, 4]/results_original_epsilon[argmax, 4]))
