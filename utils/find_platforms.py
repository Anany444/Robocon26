import numpy as np

vertices = []
faces = []
with open('/home/robot/robocon_ws/src/description/meshes/robocon_arena.obj', 'r') as f:
    for line in f:
        if line.startswith('v '):
            vertices.append(list(map(float, line.strip().split()[1:4])))
        elif line.startswith('f '):
            face = [int(p.split('/')[0]) - 1 for p in line.strip().split()[1:]]
            faces.append(face)

vertices = np.array(vertices)
platforms = []
for face in faces:
    if len(face) >= 3:
        p0, p1, p2 = vertices[face[:3]]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm > 1e-6:
            normal /= norm
            if normal[2] > 0.99:
                pts = vertices[face]
                center = np.mean(pts, axis=0)
                if center[2] > 0.05:
                    platforms.append(center)

platforms = np.array(platforms)
from sklearn.cluster import DBSCAN
if len(platforms) > 0:
    clustering = DBSCAN(eps=0.4, min_samples=1).fit(platforms[:, :2])
    labels = clustering.labels_
    unique_labels = set(labels)
    centers = []
    for k in unique_labels:
        mask = (labels == k)
        cluster_pts = platforms[mask]
        centers.append(np.mean(cluster_pts, axis=0))
    centers = sorted(centers, key=lambda x: (x[0], x[1]))
    print(f"Total clustered platforms: {len(centers)}")
    for i, c in enumerate(centers):
        print(f"Platform {i}: x={c[0]:.3f}, y={c[1]:.3f}, z={c[2]:.3f}")
