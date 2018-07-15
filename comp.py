import sys
import numpy as np
from open3d import *
from statistics import *
from MeshCompFunc import *

def calculateDiffs(pcTarget, pcSource, sourceTree, thre):
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

    return stas, errors

def CalcColorMap(v, vmin, vmax):
    
    rgb_val = [255.0, 255.0, 255.0]
    
    if (v < vmin):
         v = vmin
    if (v > vmax):
         v = vmax
    dv = vmax - vmin
    
    if (v < (vmin + 0.25 * dv)):
        rgb_val[0] = 0
        rgb_val[1] = (4.0 * (v - vmin) / dv) * 255.0
    
    elif (v < (vmin + 0.5 * dv)):
        rgb_val[0] = 0
        rgb_val[2] = (1.0 + 4.0 * (vmin + 0.25 * dv - v) / dv) * 255.0

    elif (v < (vmin + 0.75 * dv)):
        rgb_val[0] = (4.0 * (v - vmin - 0.5 * dv) / dv) * 255.0
        rgb_val[2] = 0

    else:
        rgb_val[1] = (1.0 + 4.0 * (vmin + 0.75 * dv - v) / dv) * 255.0
        rgb_val[2] = 0
    
    return map(int,rgb_val)


def rewriteColorForErrorMap(pc, errors, max):
    nTotal = len(np.asarray(pc.points))
    if nTotal != len(errors):
        raise ValueError("errors and points are not equal")
        return

    for i in range(nTotal):
        rgb = CalcColorMap(errors[i], 0, max)
        np.asarray(pc.colors)[i, :] = [rgb[0], rgb[1], rgb[2]]

    return



if __name__ == "__main__":
    
    thre = float(sys.argv[1])
    GtPath = sys.argv[2]
    print(f"GT Mesh : {GtPath}")

    sPaths = []
    for i in range(3, len(sys.argv)):
        sPaths.append(sys.argv[i])

    pcdg = read_point_cloud(GtPath)
    GtTree = KDTreeFlann(pcdg)
    errors = []
    stass = []
    pcs = []

    for i in range(len(sPaths)):
        print(f"Source Mesh : {sPaths[i]}")
        tempPC = read_point_cloud(sPaths[i])
        pcs.append(tempPC)
        print("precision")
        stasTemp, errorTemp = calculateDiffs(pcs[i], pcdg, GtTree, thre)
        stass.append(stasTemp)
        errors.append(errorTemp)
        print(stass[i])


        sTree = KDTreeFlann(pcs[i])        
        print("completness")
        stasTemp, _ = calculateDiffs(pcdg, pcs[i], sTree, thre)
        print(stasTemp)

    maxError = 0.0

    for i in len(stass):
        if stass[i].max > maxError:
            maxError = stass[i].max

    for i in len(errors):
        rewriteColorForErrorMap(pcs[i], errors[i], maxError)
        write_point_cloud("output" + str(i) + ".ply", pcs[i])