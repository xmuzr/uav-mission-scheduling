"""Original visibility shortcut simplification extracted from AStarPlanner."""


def simplify_grid_path(grid,path):
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path)-1:
        candidate = len(path)-1
        start_point = grid.index_to_point(path[anchor])
        while candidate > anchor+1:
            if grid.line_is_free(start_point,grid.index_to_point(path[candidate])):
                break
            candidate -= 1
        simplified.append(path[candidate])
        anchor = candidate
    return simplified
