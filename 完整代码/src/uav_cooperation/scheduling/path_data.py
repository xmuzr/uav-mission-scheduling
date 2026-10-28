from dataclasses import dataclass


@dataclass(frozen=True)
class PathData:
    labels: tuple
    matrix_m: object
    store: object

    def segment_points(self,start,end):
        return self.store.points(start,end)
