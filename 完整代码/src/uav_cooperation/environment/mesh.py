"""Closed triangle meshes: union-of-components occupancy and swept-sphere segments.

Only NumPy is required. Open meshes, inconsistent winding and degenerate faces
are rejected. Disconnected closed components are combined by union, so overlap
does not incorrectly become free space as in a single global parity test.
"""
from collections import defaultdict
from pathlib import Path
import struct
import numpy as np

EPS = 1e-9


def point_triangle_distance_squared(points, triangles):
    """Pairwise squared distance, including triangle interiors and edges."""
    points=np.asarray(points,dtype=float).reshape(-1,3)
    triangles=np.asarray(triangles,dtype=float)
    a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
    ab,ac=b-a,c-a
    ap=points[:,None,:]-a
    normal=np.cross(ab,ac)
    normal2=np.einsum('ij,ij->i',normal,normal)
    dot_ab=np.einsum('ij,ij->i',ab,ab)
    dot_ac=np.einsum('ij,ij->i',ac,ac)
    dot_cross=np.einsum('ij,ij->i',ab,ac)
    d_ab=np.einsum('ptj,tj->pt',ap,ab)
    d_ac=np.einsum('ptj,tj->pt',ap,ac)
    denominator=dot_ab*dot_ac-dot_cross**2
    u=(dot_ac*d_ab-dot_cross*d_ac)/denominator
    v=(dot_ab*d_ac-dot_cross*d_ab)/denominator
    plane=np.einsum('ptj,tj->pt',ap,normal)**2/normal2
    distance=np.where((u>=-EPS)&(v>=-EPS)&(u+v<=1+EPS),plane,np.inf)
    for start,end in ((a,b),(b,c),(c,a)):
        edge=end-start
        rel=points[:,None,:]-start
        ratio=np.clip(np.einsum('ptj,tj->pt',rel,edge)/np.einsum('ij,ij->i',edge,edge),0,1)
        residual=rel-ratio[:,:,None]*edge
        distance=np.minimum(distance,np.einsum('ptj,ptj->pt',residual,residual))
    return np.maximum(distance,0)


def segment_edges_distance_squared(start,end,edges_a,edges_b):
    """Exact segment-to-segment distance for one segment and many mesh edges."""
    direction=end-start
    edges=edges_b-edges_a
    a=float(direction@direction)
    c=np.einsum('ij,ij->i',edges,edges)
    def to_edges(point):
        relative=point-edges_a
        ratio=np.clip(np.einsum('ij,ij->i',relative,edges)/c,0,1)
        delta=relative-ratio[:,None]*edges
        return np.einsum('ij,ij->i',delta,delta)
    result=np.minimum(to_edges(start),to_edges(end))
    if a<=EPS**2:return result
    for points in (edges_a,edges_b):
        relative=points-start
        ratio=np.clip(relative@direction/a,0,1)
        delta=relative-ratio[:,None]*direction
        result=np.minimum(result,np.einsum('ij,ij->i',delta,delta))
    w=start-edges_a
    b=edges@direction
    d=w@direction
    e=np.einsum('ij,ij->i',w,edges)
    denominator=a*c-b*b
    valid=denominator>1e-12*a*c
    s=np.divide(b*e-c*d,denominator,out=np.zeros_like(b),where=valid)
    t=np.divide(a*e-b*d,denominator,out=np.zeros_like(b),where=valid)
    valid&=(s>=0)&(s<=1)&(t>=0)&(t<=1)
    delta=w+s[:,None]*direction-t[:,None]*edges
    return np.minimum(result,np.where(valid,np.einsum('ij,ij->i',delta,delta),np.inf))


class ClosedComponent:
    def __init__(self,triangles):
        self.triangles=np.asarray(triangles,dtype=float)
        a,b,c=self.triangles[:,0],self.triangles[:,1],self.triangles[:,2]
        volume=float(np.einsum('ij,ij->i',a,np.cross(b,c)).sum()/6)
        if abs(volume)<=1e-10:raise ValueError('Mesh component has zero signed volume')
        if volume<0:self.triangles=self.triangles[:,[0,2,1]]
        self.low=self.triangles.min(axis=(0,1))
        self.high=self.triangles.max(axis=(0,1))
        self.a,self.b,self.c=self.triangles[:,0],self.triangles[:,1],self.triangles[:,2]
        normal=np.cross(self.b-self.a,self.c-self.a)
        self.normal=normal/np.linalg.norm(normal,axis=1)[:,None]
        self.plane=np.einsum('ij,ij->i',self.normal,self.a)
        vertices=np.unique(self.triangles.reshape(-1,3),axis=0)
        self.convex=bool(np.all(vertices@self.normal.T-self.plane<=1e-7))
        self.edges_a=np.concatenate((self.a,self.b,self.c))
        self.edges_b=np.concatenate((self.b,self.c,self.a))

    def _inside(self,points):
        if self.convex:return np.all(points@self.normal.T-self.plane<=EPS,axis=1)
        a=self.a[None,:,:]-points[:,None,:]
        b=self.b[None,:,:]-points[:,None,:]
        c=self.c[None,:,:]-points[:,None,:]
        la,lb,lc=(np.linalg.norm(x,axis=2) for x in (a,b,c))
        numerator=np.einsum('pti,pti->pt',a,np.cross(b,c))
        denominator=la*lb*lc+np.einsum('pti,pti->pt',a,b)*lc+np.einsum('pti,pti->pt',b,c)*la+np.einsum('pti,pti->pt',c,a)*lb
        angles=2*np.arctan2(numerator,denominator)
        return abs(angles.sum(axis=1))>2*np.pi

    def contains_many(self,points,margin=0):
        points=np.asarray(points,dtype=float).reshape(-1,3)
        result=np.zeros(len(points),dtype=bool)
        indices=np.flatnonzero(np.all((points>=self.low-margin-EPS)&(points<=self.high+margin+EPS),axis=1))
        chunk=max(1,min(2048,100000//len(self.triangles)))
        for first in range(0,len(indices),chunk):
            selected=indices[first:first+chunk]
            p=points[selected]
            near=point_triangle_distance_squared(p,self.triangles).min(axis=1)<=(margin+EPS)**2
            remaining=np.flatnonzero(~near)
            if len(remaining):near[remaining]=self._inside(p[remaining])
            result[selected]=near
        return result

    def segment_is_free(self,start,end,margin):
        if np.any(np.maximum(start,end)<self.low-margin) or np.any(np.minimum(start,end)>self.high+margin):return True
        if self.contains_many([start,end],margin).any():return False
        direction=end-start
        if np.linalg.norm(direction)<=EPS:return True
        e1,e2=self.b-self.a,self.c-self.a
        h=np.cross(np.broadcast_to(direction,e2.shape),e2)
        determinant=np.einsum('ij,ij->i',e1,h)
        valid=abs(determinant)>1e-12
        inv=np.divide(1.0,determinant,out=np.zeros_like(determinant),where=valid)
        s=start-self.a
        u=np.einsum('ij,ij->i',s,h)*inv
        q=np.cross(s,e1)
        v=(q@direction)*inv
        t=np.einsum('ij,ij->i',e2,q)*inv
        if np.any(valid&(u>=-EPS)&(v>=-EPS)&(u+v<=1+EPS)&(t>=-EPS)&(t<=1+EPS)):return False
        endpoint=point_triangle_distance_squared([start,end],self.triangles).min()
        edge=segment_edges_distance_squared(start,end,self.edges_a,self.edges_b).min()
        return bool(min(endpoint,edge)>(margin+EPS)**2)


class TriangleMesh:
    """Validated union of closed, consistently wound triangle components."""
    def __init__(self,vertices,faces,label='triangle_mesh'):
        self.vertices=np.asarray(vertices,dtype=float)
        raw_faces=np.asarray(faces)
        if self.vertices.ndim!=2 or self.vertices.shape[1]!=3 or len(self.vertices)<4 or not np.isfinite(self.vertices).all():
            raise ValueError('Mesh vertices_m must be finite Nx3 coordinates')
        if raw_faces.ndim!=2 or raw_faces.shape[1]!=3 or len(raw_faces)<4 or raw_faces.dtype.kind not in 'iu':
            raise ValueError('Mesh faces must be integer Mx3 vertex indices')
        if len(raw_faces)>50000:raise ValueError('Mesh exceeds 50000 triangles; simplify it before importing')
        if raw_faces.min()<0 or raw_faces.max()>=len(self.vertices):raise ValueError('Mesh face index outside vertices_m')
        self.vertices,inverse=np.unique(self.vertices,axis=0,return_inverse=True)
        self.faces=inverse[raw_faces]
        self.triangles=self.vertices[self.faces]
        self.label=str(label)
        normals=np.cross(self.triangles[:,1]-self.triangles[:,0],self.triangles[:,2]-self.triangles[:,0])
        if np.any(np.linalg.norm(normals,axis=1)<=1e-10):raise ValueError('Degenerate triangle in mesh')
        parent=list(range(len(self.faces)))
        def find(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        edges=defaultdict(list)
        for i,face in enumerate(self.faces):
            for a,b in zip(face,np.roll(face,-1)):
                edges[tuple(sorted((int(a),int(b))))].append((i,int(a),int(b)))
        for records in edges.values():
            if len(records)!=2:raise ValueError('Mesh must be watertight: every edge needs exactly two faces')
            left,a,b=records[0];right,c,d=records[1]
            if a!=d or b!=c:raise ValueError('Mesh faces must have consistent winding')
            parent[find(left)]=find(right)
        groups=defaultdict(list)
        for i in range(len(self.faces)):groups[find(i)].append(i)
        self.components=tuple(ClosedComponent(self.triangles[indices]) for indices in groups.values())
        self.low=self.triangles.min(axis=(0,1));self.high=self.triangles.max(axis=(0,1))

    def contains_many(self,points,margin=0):
        points=np.asarray(points,dtype=float).reshape(-1,3)
        result=np.zeros(len(points),dtype=bool)
        for component in self.components:result|=component.contains_many(points,margin)
        return result

    def contains(self,point,margin=0):return bool(self.contains_many([point],margin)[0])

    def segment_is_free(self,start,end,margin=0):
        start,end=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
        return all(c.segment_is_free(start,end,margin) for c in self.components)

    def distance_to_surface(self,point):
        return float(np.sqrt(point_triangle_distance_squared([point],self.triangles).min()))


def read_stl(path):
    """Read binary or ASCII STL as a validated mesh; no external file references."""
    data=Path(path).read_bytes()
    count=struct.unpack_from('<I',data,80)[0] if len(data)>=84 else 0
    if count and len(data)==84+50*count:
        triangles=np.array([struct.unpack_from('<9f',data,84+50*i+12) for i in range(count)]).reshape(-1,3,3)
    else:
        try:
            vertices=[list(map(float,line.split()[1:])) for line in data.decode('ascii').splitlines() if line.strip().startswith('vertex ')]
            triangles=np.asarray(vertices,dtype=float).reshape(-1,3,3)
        except (UnicodeError,ValueError) as error:raise ValueError('Unsupported or malformed STL') from error
    if len(triangles)==0:raise ValueError('STL has no triangles')
    return TriangleMesh(triangles.reshape(-1,3),np.arange(triangles.size//3).reshape(-1,3),Path(path).stem)


def write_stl(path,mesh):
    """Export a mesh in its current coordinate frame as binary STL."""
    header=b'uav-cooperation metric triangle mesh'.ljust(80,b' ')
    with Path(path).open('wb') as stream:
        stream.write(header);stream.write(struct.pack('<I',len(mesh.triangles)))
        for triangle in mesh.triangles:
            normal=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0])
            normal=normal/np.linalg.norm(normal)
            stream.write(struct.pack('<12fH',*normal,*triangle.ravel(),0))
