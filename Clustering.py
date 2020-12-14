import numpy as np
# import matplotlib.pyplot as plt

def cluster(file):
    """ Returns an Nx2 numpy array of averaged unique measurements given a 
    .txt file with x,y coordinate pairs on each line."""
    
    f = open(file, 'r')
    c = np.array(f.read().split())
    f.close()
    
    c = c.astype(np.float)
    c = c.reshape((-1, 2))
    # =========================================================================
    # plt.plot(c[:,0], c[:,1], 'b+')
    # plt.title('Raw Data')
    # plt.grid()
    # plt.xlim((-50,50))
    # plt.ylim((-50,50))
    # plt.show()
    # =========================================================================
    
    print('Sorting...')
    idx = np.argsort(c[:,0])
    c = c[idx]
    
    print('Clustering...')
    
    epsilon = 0.5
    
    # Find euclidean distance between consecutive rows
    differences = np.diff(c, axis = 0, prepend = 0)
    euclidean_distance = np.linalg.norm(differences, axis=1)
    
    # Gather first index of unique measurments
    unique_idx = np.arange(len(c))[euclidean_distance > epsilon]
    print(f"There are {len(unique_idx)} objects to be collected.")
    
    # Average out values
    clustered = np.zeros(2)
    for i, idx in enumerate(unique_idx):
        idx1 = idx
        if i == len(unique_idx) - 1:
            average = np.average(c[idx1::], axis = 0)
        else:
            idx2 = unique_idx[i+1]
            average = np.average(c[idx1:idx2], axis=0)
        clustered = np.vstack((clustered, average))
        corrected = clustered[1:]  
    
    # =========================================================================
    # plt.plot(corrected[:,0], corrected[:,1], 'r+')
    # plt.title('Clustered')
    # plt.grid()
    # plt.xlim((-50,50))
    # plt.ylim((-50,50))
    # plt.show()
    # =========================================================================

    return corrected

file = 'OBJS_easy_corr.txt'
print(cluster(file))