import csv
import json
from math import isfinite
from pathlib import Path


def json_value(value):
    if isinstance(value,float) and not isfinite(value):
        return None
    if isinstance(value,dict):
        return {str(key):json_value(item) for key,item in value.items()}
    if isinstance(value,(tuple,list)):
        return [json_value(item) for item in value]
    return value


def write_json(path,value):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(json_value(value),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def export_plan(folder,data,plan):
    folder = Path(folder)
    write_json(folder/'mission.json',data)
    write_json(folder/'plan.json',plan)
    with (folder/'waypoints.csv').open('w',encoding='utf-8-sig',newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('drone_id','point_index','x_m','y_m','z_m'))
        for route in plan['routes']:
            for index,point in enumerate(route['points_m']):
                writer.writerow((route['drone_id'],index,*point))
    with (folder/'assignments.csv').open('w',encoding='utf-8-sig',newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('drone_id','task_order','task_id'))
        for route in plan['routes']:
            for index,task in enumerate(route['task_ids'],1):
                writer.writerow((route['drone_id'],index,task))
    if plan['edge_details']:
        with (folder/'edge_timing.csv').open('w',encoding='utf-8-sig',newline='') as file:
            writer = csv.DictWriter(file,fieldnames=list(plan['edge_details'][0]))
            writer.writeheader()
            writer.writerows(plan['edge_details'])
