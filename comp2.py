import sys
import numpy as np
from open3d import *
from statistics import *

def calculateDiffsWithErrors(pcTarget, pcSource, sourceTree, thre, errors):
    mind = 1000
    maxd = 0
    sumd = 0.0
    sumd2 = 0.0
    nCorrect = 0
    nTotal = len(np.asarray(pcTarget.points))
    errors = [0.0] * nTotal
    for j in range(nTotal):
        [k, idx, _] = sourceTree.search_knn_vector_3d(pcTarget.points[j], 1)
        p0 = np.asarray(pcTarget.points)[j, :]
        p1 = np.asarray(pcSource.points)[idx[0], :]

        d = np.linalg.norm(p0 - p1)
        sumd += d
        sumd2 += d*d
        errors[j] = d

        if mind > d:
            mind = d
        
        if maxd < d:
            maxd = d

        if thre > d:
            nCorrect = nCorrect +  1

    stas = statistics()
    stas.min = mind
    stas.max = maxd
    stas.avg = sumd / nTotal
    stas.stddev = np.sqrt(sumd2 / nTotal - ((sumd/nTotal)*(sumd/nTotal)))
    stas.numCorrect = nCorrect
    stas.numTotal = nTotal
    stas.threshold = thre

    return stas

if __name__ == "__main__":
    
    thre = float(sys.argv[1])
    GtPath = sys.argv[2]
    print(f"GT Mesh : {GtPath}")

    sPaths = []
    for i in range(3, len(sys.argv)):
        sPaths.append(sys.argv[i])

    pcdg = read_point_cloud(GtPath)
    GtTree = KDTreeFlann(pcdg)
    errorss = [[]] * len(sPaths)
    for i in range(len(sPaths)):
        print(f"Source Mesh : {sPaths[i]}")
        pc = read_point_cloud(sPaths[i])
        print("precision")
        print(calculateDiffsWithErrors(pc, pcdg, GtTree, thre, errorss[i]))

    
