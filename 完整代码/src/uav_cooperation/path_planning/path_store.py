"""Per-run path storage; no cross-run cached results are loaded."""
from dataclasses import dataclass,field


@dataclass
class PathStore:
    paths: dict = field(default_factory=dict)

    @staticmethod
    def key(a,b):
        return min(a,b),max(a,b)

    def points(self,a,b):
        path = self.paths[self.key(a,b)].points
        return path if a < b else tuple(reversed(path))
