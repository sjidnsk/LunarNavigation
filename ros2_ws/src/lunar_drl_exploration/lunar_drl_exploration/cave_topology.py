"""Fixed, isomorphism-separated mother graphs and planar cave geometry.

Abstract cycle rank describes the room graph, never the free-cell grid. A bounded
orthogonal lattice avoids crossing corridors. Catalog identity includes layout;
its mother_graph_id is an exact label-independent adjacency canonicalization.
No Torch, native terrain binding or policy is imported here.
"""
from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
import numpy as np

VERSION = 'cave_topology_v2'
CONSTRUCTION_KINDS = ('chain', 'tree', 'single_loop', 'multi_loop')
SPLITS = ('train', 'validation', 'test')
CATALOG_SCHEMA = 'cave_topology_groups_v2'


def _digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def _adjacency(n,edges):
    if type(n) is not int or not 6<=n<=18:raise ValueError('mother graph requires 6–18 nodes')
    adj=[set() for _ in range(n)]
    for a,b in edges:
        if not 0<=a<n or not 0<=b<n or a==b or b in adj[a]:raise ValueError('invalid graph edge')
        adj[a].add(b);adj[b].add(a)
    seen={0};pending=[0]
    while pending:
        for node in adj[pending.pop()]-seen:seen.add(node);pending.append(node)
    if len(seen)!=n:raise ValueError('mother graph disconnected')
    return adj


def validate_graph(n,edges,family):
    adj=_adjacency(n,edges);rank=len(edges)-n+1;degree=max(map(len,adj))
    valid={'chain':rank==0 and degree<=2,'tree':rank==0 and degree>=3,
           'single_loop':rank==1,'multi_loop':2<=rank<=4}
    if family not in valid or not valid[family]:raise ValueError('topology family constraint failed')
    return dict(node_count=n,edge_count=len(edges),abstract_cycle_rank=rank,max_degree=degree)


def _refine(adj,partition):
    while True:
        result=[]
        for group in partition:
            buckets={}
            for v in group:
                signature=tuple(len(adj[v]&set(other)) for other in partition)
                buckets.setdefault(signature,[]).append(v)
            result.extend(tuple(buckets[k]) for k in sorted(buckets))
        if len(result)==len(partition):return tuple(result)
        partition=result


def graph_signature(n,edges):
    """Exact canonical adjacency by exhaustive refinement/individualization.

    WL is only a pruning step. All unresolved labelings are individualized and
    the lexicographically minimal adjacency is used, including regular graphs.
    """
    adj=_adjacency(n,edges)
    partition=tuple(tuple(i for i in range(n) if len(adj[i])==d) for d in sorted(set(map(len,adj))))
    def canonical(part):
        part=_refine(adj,part)
        ambiguous=[(len(g),i) for i,g in enumerate(part) if len(g)>1]
        if not ambiguous:
            order=[g[0] for g in part]
            return ''.join('1' if order[j] in adj[order[i]] else '0' for i in range(n) for j in range(i+1,n))
        _,index=min(ambiguous);group=part[index]
        return min(canonical(part[:index]+((v,),tuple(x for x in group if x!=v))+part[index+1:]) for v in group)
    return str(n)+':'+canonical(partition)


def isomorphic(n_a,edges_a,n_b,edges_b):
    """Exact adjacency search, independent of the stored canonical signature."""
    if n_a!=n_b or len(edges_a)!=len(edges_b):return False
    a=_adjacency(n_a,edges_a);b=_adjacency(n_b,edges_b)
    if sorted(map(len,a))!=sorted(map(len,b)):return False
    mapping={};used=set()
    def visit():
        if len(mapping)==n_a:return True
        v=max((i for i in range(n_a) if i not in mapping),key=lambda i:(len(a[i]&mapping.keys()),len(a[i])))
        for w in range(n_a):
            if w in used or len(a[v])!=len(b[w]):continue
            if any((u in a[v])!=(mapping[u] in b[w]) for u in mapping):continue
            mapping[v]=w;used.add(w)
            if visit():return True
            used.remove(w);del mapping[v]
        return False
    return visit()


def _lattice_candidate(rng,family):
    n=int(rng.integers(6 if family!='multi_loop' else 9,19))
    all_points=[(x,y) for y in range(4) for x in range(5)]
    if family=='chain':
        points=[(x,y) for y in range(4) for x in (range(5) if y%2==0 else range(4,-1,-1))][:n]
        return points,[(i,i+1) for i in range(n-1)]
    chosen=[all_points[int(rng.integers(20))]];edges=[]
    while len(chosen)<n:
        frontier=[(i,p) for i,q in enumerate(chosen) for p in all_points if p not in chosen
                  and abs(p[0]-q[0])+abs(p[1]-q[1])==1]
        i,p=frontier[int(rng.integers(len(frontier)))];edges.append((i,len(chosen)));chosen.append(p)
    if family=='tree':return chosen,edges
    candidates=[(i,j) for i,a in enumerate(chosen) for j,b in enumerate(chosen) if i<j and
                abs(a[0]-b[0])+abs(a[1]-b[1])==1 and (i,j) not in edges and (j,i) not in edges]
    rank=1 if family=='single_loop' else int(rng.integers(2,5))
    if len(candidates)<rank:return None
    picked=rng.choice(len(candidates),rank,replace=False)
    return chosen,edges+[candidates[int(i)] for i in picked]


@lru_cache(maxsize=1)
def _catalog():
    rows=[];rng=np.random.default_rng(np.random.SeedSequence([20261014,702]))
    for split,count in (('train',64),('validation',24),('test',24)):
        for ordinal in range(count):
            for attempt in range(10000):
                # Construction algorithms are private implementation choices;
                # there are no cave subtype tasks, proportions or split quotas.
                construction=CONSTRUCTION_KINDS[int(rng.integers(len(CONSTRUCTION_KINDS)))]
                candidate=_lattice_candidate(rng,construction)
                if candidate is None:continue
                points,edges=candidate;n=len(points)
                try:summary=validate_graph(n,edges,construction)
                except ValueError:continue
                signature=graph_signature(n,edges)
                if any(signature==r['structure_signature'] or isomorphic(n,edges,r['node_count'],r['edges']) for r in rows):continue
                row=dict(generator_version=VERSION,split=split,
                    mother_graph_id=_digest(dict(signature=signature)),structure_signature=signature,
                    node_count=n,edges=[list(e) for e in sorted(tuple(sorted(e)) for e in edges)],
                    lattice_points=[list(p) for p in points],abstract_summary=summary)
                rows.append(row);break
            else:raise ValueError('deterministic graph catalog exhausted')
    return tuple(rows)


def mother_catalog():return deepcopy(list(_catalog()))


def manifest_record():
    return dict(schema=CATALOG_SCHEMA,generator_version=VERSION,mother_graphs=mother_catalog(),
        geometry_schema='orthogonal_lattice_5x4_v2',node_interval=[6,18],
        minimum_extent_m=30.,task='cave_inside',native_qualification='NOT_RUN',
        isomorphism_exclusion='exact_canonical_and_adjacency_search')


def manifest_hash():return _digest(manifest_record())


def select_mother(split,ordinal,*,mother_graph_id=None):
    if split not in SPLITS or type(ordinal) is not int or ordinal<0:
        raise ValueError('invalid topology selection')
    rows=[r for r in _catalog() if r['split']==split]
    if mother_graph_id is not None:
        rows=[r for r in rows if r['mother_graph_id']==mother_graph_id]
        if not rows:raise ValueError('mother graph does not belong to requested split')
    return deepcopy(rows[ordinal%len(rows)])


def _point_segment(p,a,b):
    p,a,b=map(lambda v:np.asarray(v,float),(p,a,b));d=b-a
    if not np.dot(d,d)>0:raise ValueError('zero length passage')
    return float(np.linalg.norm(p-(a+np.clip(np.dot(p-a,d)/np.dot(d,d),0,1)*d)))


def _segments_distance(a,b,c,d):
    a,b,c,d=map(lambda v:np.asarray(v,float),(a,b,c,d))
    def cross(u,v):return float(u[0]*v[1]-u[1]*v[0])
    denom=cross(b-a,d-c)
    if abs(denom)>1e-10:
        t=cross(c-a,d-c)/denom;u=cross(c-a,b-a)/denom
        if 0<=t<=1 and 0<=u<=1:return 0.
    return min(_point_segment(a,c,d),_point_segment(b,c,d),_point_segment(c,a,b),_point_segment(d,a,b))


def validate_geometry(geometry,margin_m=.4):
    """Conservative analytic clearance of all nonincident primitives."""
    rooms=geometry['rooms'];passages=geometry['passages'];edges=geometry['edges']
    if len(edges)!=len(passages):raise ValueError('passage graph mismatch')
    for i,(x,y,r) in enumerate(rooms):
        if r<=0:raise ValueError('invalid room radius')
        for xx,yy,rr in rooms[:i]:
            if math.hypot(x-xx,y-yy)<=r+rr+margin_m:raise ValueError('unrecorded room contact')
    for k,(ax,ay,bx,by,half_width) in enumerate(passages):
        a,b=edges[k]
        if (not np.allclose((ax,ay),rooms[a][:2]) or not np.allclose((bx,by),rooms[b][:2])
                or half_width<=0):raise ValueError('passage endpoints mismatch')
        for i,(x,y,r) in enumerate(rooms):
            if i not in (a,b) and _point_segment((x,y),(ax,ay),(bx,by))<=r+half_width+margin_m:
                raise ValueError('unrecorded passage-room contact')
        for j,(cx,cy,dx,dy,other_width) in enumerate(passages[:k]):
            if set(edges[j])&{a,b}:continue
            if _segments_distance((ax,ay),(bx,by),(cx,cy),(dx,dy))<=half_width+other_width+margin_m:
                raise ValueError('unrecorded passage contact or crossing')
    return True


def build_geometry(mother,seed,extent_m):
    if mother['generator_version']!=VERSION or not math.isfinite(extent_m) or not 30<=extent_m<=1000:
        raise ValueError('v2 cave requires extent in [30,1000]')
    _adjacency(mother['node_count'],mother['edges'])
    rng=np.random.default_rng(np.random.SeedSequence([int(seed),703]))
    # 30 m supports at most 4*6.25+2*2.2=29.4 m. Larger extents stretch corridors;
    # chamber and corridor width remain wheel-sized rather than scale with extent.
    spacing=float(rng.uniform(5.8,max(5.800001,(extent_m-5.)/4)))
    points=np.asarray(mother['lattice_points'],float)*spacing
    points-=(points.min(axis=0)+points.max(axis=0))/2
    angle=int(rng.integers(4))*math.pi/2
    rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    points=points@rotation.T
    rooms=[(*map(float,p),float(rng.uniform(2.1,2.2))) for p in points]
    passages=[(*map(float,points[a]),*map(float,points[b]),float(rng.uniform(1.25,1.4))) for a,b in mother['edges']]
    result=dict(generator_version=VERSION,
        mother_graph_id=mother['mother_graph_id'],structure_signature=mother['structure_signature'],
        dataset_split=mother['split'],geometry_seed=int(seed),edges=deepcopy(mother['edges']),
        rooms=rooms,passages=passages,anchor_node=0,anchor_xy=list(rooms[0][:2]),
        abstract_summary=deepcopy(mother['abstract_summary']),geometry_validation='analytic_nonincident_clearance_passed',
        decorative_unreachable_rooms=[])
    validate_geometry(result)
    return result


def primitive_masks(geometry,shape,origin,resolution):
    x=origin[0]+(np.arange(shape[1])+.5)*resolution
    y=origin[1]+(np.arange(shape[0])+.5)*resolution
    x=x[None,:];y=y[:,None]
    rooms=[(x-cx)**2+(y-cy)**2<=radius**2 for cx,cy,radius in geometry['rooms']]
    passages=[]
    for ax,ay,bx,by,radius in geometry['passages']:
        dx=bx-ax;dy=by-ay
        t=np.clip(((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy),0,1)
        passages.append(np.hypot(x-ax-t*dx,y-ay-t*dy)<=radius)
    return rooms,passages


def qualify_grid(geometry,free_floor,shape,origin,resolution):
    """Contract measured free floor by room/corridor labels and recover edges.

    This does not apply E−V+1 to cells: rooms contract to graph nodes; corridor
    components must each touch exactly their two recorded room endpoints.
    """
    from scipy.ndimage import label,binary_dilation
    free_floor=np.asarray(free_floor,bool)
    if free_floor.shape!=tuple(shape):raise ValueError('qualification shape mismatch')
    rooms,passages=primitive_masks(geometry,shape,origin,resolution)
    ownership=np.full(shape,-1,np.int32)
    for i,mask in enumerate(rooms):
        if not np.any(mask&free_floor):raise ValueError('native room has no free stance')
        if label(mask&free_floor)[1]!=1:raise ValueError('native room disconnected')
        ownership[mask&free_floor]=i
    room_union=ownership>=0;corridors=[]
    for i,mask in enumerate(passages):
        core=mask&free_floor&~room_union
        if label(core)[1]!=1:raise ValueError('native key corridor is disconnected or absent')
        corridors.append(core)
    support=np.logical_or.reduce(rooms+passages)
    if np.any(free_floor&~support):raise ValueError('unlabelled floor shortcut')
    realized=[]
    for i,core in enumerate(corridors):
        contact=binary_dilation(core)
        for other in corridors[:i]:
            if np.any(contact&other):raise ValueError('unrecorded corridor contact shortcut')
        actual=sorted(set(map(int,ownership[contact&room_union])))
        expected=sorted(geometry['edges'][i])
        if actual!=expected:raise ValueError('native corridor endpoints changed')
        realized.append(expected)
    if label(free_floor)[1]!=1:raise ValueError('native cave floor is not one reachable component')
    if realized!=geometry['edges']:raise ValueError('native realized adjacency mismatch')
    return dict(grid_topology_validation='room_corridor_contraction_passed',
        realized_edges=realized,native_floor_free_cells=int(np.count_nonzero(free_floor)),
        native_room_count=len(rooms),native_corridor_count=len(passages),
        native_floor_components=1,abstract_summary=deepcopy(geometry['abstract_summary']))


def qualify_native(scene,terrain):
    from .contracts import Pose
    if terrain.resolution_m!=.2:raise ValueError('cave v2 qualification requires 0.2 m terrain')
    geometry=scene.cave_geometry
    summary=qualify_grid(geometry,(terrain.navigation==1)&(terrain.heights==0),
        terrain.shape,terrain.origin,terrain.resolution_m)
    x,y=terrain.world_to_cell(*geometry['anchor_xy'])
    if not (terrain.navigation[y,x]==1 and terrain.heights[y,x]==0):
        raise ValueError('v2 explicit anchor has no native stance')
    pose=Pose(*terrain.cell_center(x,y),scene._yaw)
    reachable=terrain.reachable(pose)!=0
    floor=(terrain.navigation==1)&(terrain.heights==0)
    if np.any(floor&~reachable):raise ValueError('explicit anchor cannot reach all native rooms/passages')
    if np.any(reachable&(terrain.heights!=0)):
        raise ValueError('native cave floor unexpectedly reaches elevated wall plateau')
    summary.update(native_qualification='passed',resolution_m=.2,
        native_anchor_cell=[x,y],native_anchor_xy=list(terrain.cell_center(x,y)),
        native_reachable_floor_cells=int(np.count_nonzero(reachable&floor)),
        terrain_hash=hashlib.sha256(terrain.heights.tobytes()+terrain.navigation.tobytes()+
            json.dumps((terrain.origin,.2,dict(scene.platform.capability)),sort_keys=True).encode()).hexdigest(),
        capability_hash=_digest(dict(scene.platform.capability)),
        decorative_unreachable_rooms=deepcopy(geometry['decorative_unreachable_rooms']))
    return summary
