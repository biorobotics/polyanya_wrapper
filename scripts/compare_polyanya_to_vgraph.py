import numpy as np

results_vgraph = np.load('results_vgraph.npy')
results_polyanya = np.load('results_original_epsilon.npy')
not_nan_idx = np.logical_not(np.isnan(results_polyanya[:, 4]))

results_vgraph = results_vgraph[not_nan_idx]
results_polyanya = results_polyanya[not_nan_idx]

inf_idx = np.where(np.isinf(results_polyanya[:, 4]))[0]
start_goal = results_vgraph[:, :4]
assert(np.all(start_goal == results_polyanya[:, :4]))
print(np.where(np.isfinite(results_polyanya[np.isinf(results_vgraph[:, 4]), 4]))[0])
print(inf_idx[np.where(np.isfinite(results_vgraph[inf_idx, 4]))[0]])
# assert(np.all(np.isinf(results_vgraph[inf_idx, 4])))
finite_idx = np.where(np.logical_and(np.isfinite(results_vgraph[:, 4]), np.isfinite(results_polyanya[:, 4])))[0]
print('Max absolute value of difference in path length from polyanya and vgraph: %f' %(np.amax(np.abs(results_vgraph[finite_idx, 4] - results_polyanya[finite_idx, 4]))))
argmax = finite_idx[np.argmax(np.abs(results_vgraph[finite_idx, 4] - results_polyanya[finite_idx, 4]))]
print('Test index of max difference: %d' %(argmax%100)) # Test index
print('Obstacle map index of max difference: %d' %(argmax//100)) # Obstacle map index
print('Path length from vgraph is %f, and from polyanya is %f' %(results_vgraph[argmax, 4], results_polyanya[argmax, 4]))
print('Ratio of vgraph path length to polyanya path length is %f' %(results_vgraph[argmax, 4]/results_polyanya[argmax, 4]))
