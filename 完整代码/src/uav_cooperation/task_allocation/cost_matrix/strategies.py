"""Migrated <=threshold hybrid strategy; threshold includes depot connections."""
import numpy as np


def euclidean_matrix(positions):
    positions = np.asarray(positions,dtype=float)
    return np.linalg.norm(positions[:,None,:]-positions[None,:,:],axis=2)


def candidate_astar_edges(euclidean,strategy,threshold_m):
    if strategy not in ('euclidean','hybrid','astar') or threshold_m <= 0:
        raise ValueError('Unknown distance strategy or nonpositive threshold')
    return [(a,b) for a in range(len(euclidean)) for b in range(a+1,len(euclidean))
            if strategy == 'astar' or (strategy == 'hybrid' and euclidean[a,b] <= threshold_m)]
