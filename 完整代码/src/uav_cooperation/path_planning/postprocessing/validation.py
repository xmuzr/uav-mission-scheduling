import numpy as np


def validate_path(path,start,end,checker):
    points = np.asarray(path.points)
    return bool(np.allclose(points[0],start) and np.allclose(points[-1],end)
                and np.isclose(np.linalg.norm(np.diff(points,axis=0),axis=1).sum(),path.length_m)
                and all(checker.line_is_free(a,b) for a,b in zip(points,points[1:])))
