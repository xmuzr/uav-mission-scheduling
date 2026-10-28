"""Generic 3D routes and time-slider playback, using existing trajectory data."""
import numpy as np
from uav_cooperation.models.request import MissionRequest
from uav_cooperation.models.geometry import Cuboid
from uav_cooperation.environment.mesh import TriangleMesh
from uav_cooperation.scheduling.trajectory import TimedTrajectory,TimedSegment


def show_plan(data,plan,snapshot=None):
    from uav_cooperation.application.pipeline import input_fingerprint
    if plan.get('input_fingerprint') != input_fingerprint(data):
        raise ValueError('Plan does not match inputs; use its exported mission.json or replan')
    import matplotlib
    if snapshot:
        matplotlib.use('Agg')
    else:
        import sys
        import importlib.util
        if sys.platform == 'win32' and importlib.util.find_spec('_tkinter') is None:
            matplotlib.use('WebAgg')
            matplotlib.rcParams.update({'webagg.address':'127.0.0.1','webagg.port':8988,
                                        'webagg.port_retries':50,'webagg.open_in_browser':True})
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider,Button
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    request = MissionRequest.from_dict(data)
    figure = plt.figure(figsize=(12,7))
    axis = figure.add_subplot(111,projection='3d')
    figure.subplots_adjust(bottom=0.2)
    for obstacle in request.scene.obstacles:
        if isinstance(obstacle,Cuboid):
            center,half = np.asarray(obstacle.center),np.asarray(obstacle.size)/2
            vertices = np.asarray([center+half*np.asarray((x,y,z)) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
            faces = [vertices[list(indices)] for indices in ((0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5))]
            axis.add_collection3d(Poly3DCollection(faces,alpha=0.15,facecolor='gray'))
        elif isinstance(obstacle,TriangleMesh):
            axis.add_collection3d(Poly3DCollection(obstacle.triangles,alpha=0.16,facecolor='gray',edgecolor='gray',linewidths=0.25))
        else:
            angle = np.linspace(0,2*np.pi,16)
            z = np.asarray((obstacle.center[2]-obstacle.height/2,obstacle.center[2]+obstacle.height/2))
            axis.plot_surface(obstacle.center[0]+obstacle.radius*np.cos(angle)[None,:]+np.zeros((2,1)),
                              obstacle.center[1]+obstacle.radius*np.sin(angle)[None,:]+np.zeros((2,1)),
                              z[:,None]+np.zeros((1,16)),alpha=0.15,color='gray')
    tasks = np.asarray([t.position_m for t in request.tasks])
    axis.scatter(*tasks.T,color='black',marker='x',label='Tasks')
    for task in request.tasks:
        axis.text(*task.position_m,task.id,fontsize=7)
    markers = {}
    for index,route in enumerate(plan['routes']):
        points = np.asarray(route['points_m'])
        color = plt.get_cmap('tab20')(index%20)
        axis.plot(*points.T,color=color,label=route['drone_id'])
        marker, = axis.plot(*np.asarray(route['start_m'])[:,None],marker='o',color=color,markersize=7)
        markers[index+1] = marker
    low,high = request.scene.config.map_min,request.scene.config.map_max
    axis.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2],high[2]),xlabel='X / m',ylabel='Y / m',zlabel='Z / m')
    axis.set_box_aspect(np.asarray(high)-low)
    allocation = plan.get('allocation',{}).get('strategy')
    allocation_label = f', {allocation}' if allocation else ''
    axis.set_title(f"{request.name}: {plan['distance_strategy']}{allocation_label}, {plan['status']}")
    axis.legend(loc='upper left')
    trajectories = tuple(TimedTrajectory(vehicle_id=t['vehicle_id'],route=tuple(t['route']),route_labels=tuple(t['route_labels']),
                                         start_delay_s=t['start_delay_s'],airborne_duration_s=t['airborne_duration_s'],
                                         segments=tuple(TimedSegment(**s) for s in t['segments'])) for t in plan['trajectories'])
    state = {'playing':False}
    slider = Slider(figure.add_axes([0.2,0.09,0.65,0.035]),'Time / s',0,max(plan['makespan_after_scheduling_s'],1),valinit=0)
    def update(value):
        for trajectory in trajectories:
            if value < trajectory.start_s:
                point = trajectory.segments[0].start
            elif value > trajectory.end_s:
                point = trajectory.segments[-1].end
            else:
                point,_ = trajectory.state_at(value)
            markers[trajectory.vehicle_id].set_data_3d([point[0]],[point[1]],[point[2]])
        figure.canvas.draw_idle()
    slider.on_changed(update)
    button = Button(figure.add_axes([0.03,0.06,0.11,0.06]),'Play / Pause')
    button.on_clicked(lambda event:state.update(playing=not state['playing']))
    timer = figure.canvas.new_timer(interval=100)
    def tick():
        if state['playing']:
            slider.set_val(min(slider.valmax,slider.val+max(slider.valmax/200,0.1)))
            if slider.val >= slider.valmax:
                state['playing'] = False
    timer.add_callback(tick)
    if snapshot:
        figure.savefig(snapshot,dpi=160)
        plt.close(figure)
    else:
        timer.start()
        if matplotlib.get_backend().lower() == 'webagg':
            from matplotlib.backends.backend_webagg import WebAggApplication
            WebAggApplication.initialize()
            print(f'Local playback: http://127.0.0.1:{WebAggApplication.port}/',flush=True)
        plt.show()
