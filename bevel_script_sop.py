import math

MODE = "edge"            # "edge" = rounded bevel on every edge; "vertex" = truncate only selected corners
BEVEL_WIDTH = 0.08        # world units
SEGMENTS = 2              # "edge" mode profile segments; 1 = flat chamfer, higher = rounder. Even = clean quad caps.
VERTEX_INDICES = None     # "vertex" mode only: a set of point indices, e.g. {0, 2, 5}; None = all points
WELD_TOLERANCE = 0.0001   # points closer than this are treated as the same shared point
PROFILE_SHAPE = 0.5       # 0.25 = flat chamfer, 0.5 = circular (default), >0.5 = convex/square, <0.25 = concave
CLAMP_OVERLAP = True      # auto-shrink width so the bevel can't self-intersect on short edges

def _sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def _scale(a, s): return (a[0]*s, a[1]*s, a[2]*s)
def _dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def _cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def _len(a): return math.sqrt(_dot(a, a))

def _norm(a):
	l = _len(a)
	if l < 1e-9:
		return (0.0, 0.0, 0.0)
	return (a[0]/l, a[1]/l, a[2]/l)

def _setPolyUV(poly):
	# Per-polygon local planar projection -- not a seamless global unwrap,
	# but gives every polygon sane, non-degenerate UVs for roughness/normal maps.
	n = len(poly)
	pts = [(poly[i].point.x, poly[i].point.y, poly[i].point.z) for i in range(n)]
	origin = pts[0]
	t = _norm(_sub(pts[1], pts[0])) if n > 1 else (1.0, 0.0, 0.0)
	normal = None
	for k in range(2, n):
		cand = _cross(_sub(pts[1], pts[0]), _sub(pts[k], pts[0]))
		if _len(cand) > 1e-9:
			normal = _norm(cand)
			break
	if normal is None:
		normal = (0.0, 0.0, 1.0)
	b = _norm(_cross(normal, t))
	for i in range(n):
		d = _sub(pts[i], origin)
		poly[i].uv[0] = _dot(d, t)
		poly[i].uv[1] = _dot(d, b)

def _weldIndices(faces, tolerance):
	canon = {}
	nextId = [0]
	inv = 1.0 / tolerance
	def keyOf(p):
		return (round(p[0]*inv), round(p[1]*inv), round(p[2]*inv))
	for f in faces:
		newIdx = []
		for p in f["pos"]:
			k = keyOf(p)
			if k not in canon:
				canon[k] = nextId[0]
				nextId[0] += 1
			newIdx.append(canon[k])
		f["idx"] = newIdx

def _faceInsetCorners(srcPos, normal, width):
	n = len(srcPos)
	origin = srcPos[0]
	T = _norm(_sub(srcPos[1], srcPos[0]))
	B = _norm(_cross(normal, T))

	pts2D = []
	for p in srcPos:
		d = _sub(p, origin)
		pts2D.append((_dot(d, T), _dot(d, B)))

	cx = sum(p[0] for p in pts2D) / n
	cy = sum(p[1] for p in pts2D) / n

	lines = []
	for i in range(n):
		j = (i + 1) % n
		ex, ey = pts2D[j][0]-pts2D[i][0], pts2D[j][1]-pts2D[i][1]
		elen = math.sqrt(ex*ex + ey*ey)
		if elen < 1e-9:
			ex, ey = 1.0, 0.0
		else:
			ex, ey = ex/elen, ey/elen
		nx, ny = -ey, ex
		midx, midy = (pts2D[i][0]+pts2D[j][0])*0.5, (pts2D[i][1]+pts2D[j][1])*0.5
		if (nx*(cx-midx) + ny*(cy-midy)) < 0.0:
			nx, ny = -nx, -ny
		ox, oy = pts2D[i][0] + nx*width, pts2D[i][1] + ny*width
		lines.append(((ox, oy), (ex, ey)))

	def intersect(l1, l2):
		(p1x, p1y), (d1x, d1y) = l1
		(p2x, p2y), (d2x, d2y) = l2
		det = d2x*d1y - d1x*d2y
		if abs(det) < 1e-9:
			return (p1x, p1y)
		t = (-(p2x-p1x)*d2y + d2x*(p2y-p1y)) / det
		return (p1x + t*d1x, p1y + t*d1y)

	inset2D = [intersect(lines[(i-1) % n], lines[i]) for i in range(n)]
	return [_add(origin, _add(_scale(T, u), _scale(B, v))) for (u, v) in inset2D]

def _slerpProfile(origin, startPt, endPt, segments, profileShape=0.5):
	# profileShape blends between a flat chord (0.25, matching Blender's flat
	# chamfer point) and the full circular slerp (0.5, Blender's default round
	# profile), extrapolating past 0.5 for a convex/square-ish push and below
	# 0.25 for a concave scoop -- an approximation of Blender's superellipse
	# profile, not an exact evaluation of that formula.
	vecStart = _sub(startPt, origin)
	vecEnd = _sub(endPt, origin)
	la, lb = _len(vecStart), _len(vecEnd)
	bulge = (profileShape - 0.25) / 0.25
	if la < 1e-9 or lb < 1e-9:
		return [startPt for _ in range(segments+1)]
	ua, ub = _scale(vecStart, 1.0/la), _scale(vecEnd, 1.0/lb)
	cosA = max(-1.0, min(1.0, _dot(ua, ub)))
	angle = math.acos(cosA)
	pts = []
	for k in range(segments+1):
		t = k / float(segments)
		length = la + (lb-la)*t
		flatPt = _add(startPt, _scale(_sub(endPt, startPt), t))
		if angle < 1e-5:
			d = _norm(_add(_scale(ua, 1.0-t), _scale(ub, t)))
		else:
			sinA = math.sin(angle)
			ca = math.sin((1.0-t)*angle) / sinA
			cb = math.sin(t*angle) / sinA
			d = _add(_scale(ua, ca), _scale(ub, cb))
		circPt = _add(origin, _scale(d, length))
		pts.append(_add(flatPt, _scale(_sub(circPt, flatPt), bulge)))
	return pts

def _buildVertexCapLoop(arcs):
	if not arcs:
		return None

	adj = {}
	for i, (fa, fb, chain) in enumerate(arcs):
		adj.setdefault(fa, []).append((fb, chain, i))
		adj.setdefault(fb, []).append((fa, list(reversed(chain)), i))

	# try every possible starting arc before giving up -- a walk that fails
	# from one starting point can still succeed from another.
	for startIdx in range(len(arcs)):
		fa0, fb0, chain0 = arcs[startIdx]
		startFace = fa0
		usedArcIdx = {startIdx}
		loopPoints = list(chain0)
		currentFace = fb0

		maxSteps = len(arcs) + 2
		steps = 0
		failed = False
		while currentFace != startFace and steps < maxSteps:
			steps += 1
			options = [o for o in adj.get(currentFace, []) if o[2] not in usedArcIdx]
			if not options:
				failed = True
				break
			nextFace, chain, idx = options[0]
			usedArcIdx.add(idx)
			loopPoints.extend(chain[1:])
			currentFace = nextFace

		if failed or currentFace != startFace:
			continue

		if len(loopPoints) > 1 and loopPoints[-1] is loopPoints[0]:
			loopPoints = loopPoints[:-1]
		return loopPoints

	return None

def _fanCapFallback(scriptOp, loop):
	# used only when a vertex isn't a clean 3-arc junction (e.g. valence != 3) —
	# fans from one of the loop's own existing bevel vertices, not an invented point.
	n = len(loop)
	if n < 3:
		return

	hub = loop[0]
	k = 1
	while k + 2 <= n - 1:
		quad = scriptOp.appendPoly(4, closed=True, addPoints=False)
		quad[0].point = hub
		quad[1].point = loop[k]
		quad[2].point = loop[k+1]
		quad[3].point = loop[k+2]
		_setPolyUV(quad)
		k += 2

	if k <= n - 2:
		tri = scriptOp.appendPoly(3, closed=True, addPoints=False)
		tri[0].point = hub
		tri[1].point = loop[k]
		tri[2].point = loop[k+1]
		_setPolyUV(tri)

def _triangulateCorner(scriptOp, loop, originalVertex, segments):
	# Proper barycentric triangular-lattice fill for a 3-arc (valence-3) junction.
	# The boundary reuses the existing loop points exactly (no seam); interior
	# points are only created where a lattice position (a,b,c), a+b+c=S, all > 0
	# actually exists. At S=2 no such point exists -- verified by direct
	# enumeration -- so that case is honestly 4 triangles, not forced quads.
	S = segments
	if len(loop) != 3 * S:
		_fanCapFallback(scriptOp, loop)
		return

	cornerA, cornerB, cornerC = loop[0], loop[S], loop[2*S]
	posA = (cornerA.x, cornerA.y, cornerA.z)
	posB = (cornerB.x, cornerB.y, cornerB.z)
	posC = (cornerC.x, cornerC.y, cornerC.z)
	uA, lenA = _norm(_sub(posA, originalVertex)), _len(_sub(posA, originalVertex))
	uB, lenB = _norm(_sub(posB, originalVertex)), _len(_sub(posB, originalVertex))
	uC, lenC = _norm(_sub(posC, originalVertex)), _len(_sub(posC, originalVertex))

	def boundaryPoint(a, b, c):
		if c == 0:
			return loop[b]
		elif a == 0:
			return loop[S + c]
		else:
			return loop[2*S + a]

	interiorCache = {}
	def latticePoint(i, j):
		a, b, c = S - i - j, i, j
		if a == 0 or b == 0 or c == 0:
			return boundaryPoint(a, b, c)
		key = (a, b, c)
		if key not in interiorCache:
			wA, wB, wC = a/float(S), b/float(S), c/float(S)
			d = _norm(_add(_add(_scale(uA, wA), _scale(uB, wB)), _scale(uC, wC)))
			length = wA*lenA + wB*lenB + wC*lenC
			pos = _add(originalVertex, _scale(d, length))
			p = scriptOp.appendPoint()
			p.x, p.y, p.z = pos
			interiorCache[key] = p
		return interiorCache[key]

	def setBaryUV(tri, coords):
		# (i,j) used directly as UV -- continuous across the WHOLE cap, not
		# restarted per triangle, since it's the lattice's own parametrization.
		for idx, (ii, jj) in enumerate(coords):
			tri[idx].uv[0] = ii / float(S)
			tri[idx].uv[1] = jj / float(S)

	for i in range(S):
		for j in range(S - i):
			tri = scriptOp.appendPoly(3, closed=True, addPoints=False)
			tri[0].point = latticePoint(i, j)
			tri[1].point = latticePoint(i+1, j)
			tri[2].point = latticePoint(i, j+1)
			setBaryUV(tri, [(i, j), (i+1, j), (i, j+1)])

	for i in range(S - 1):
		for j in range(S - i - 1):
			tri = scriptOp.appendPoly(3, closed=True, addPoints=False)
			tri[0].point = latticePoint(i+1, j)
			tri[1].point = latticePoint(i+1, j+1)
			tri[2].point = latticePoint(i, j+1)
			setBaryUV(tri, [(i+1, j), (i+1, j+1), (i, j+1)])

def onCook(scriptOp):
	global MODE, BEVEL_WIDTH, SEGMENTS, WELD_TOLERANCE, VERTEX_INDICES, PROFILE_SHAPE, CLAMP_OVERLAP
	MODE = scriptOp.par.Mode.eval()
	BEVEL_WIDTH = scriptOp.par.Width.eval()
	SEGMENTS = max(1, int(scriptOp.par.Segments.eval()))
	WELD_TOLERANCE = scriptOp.par.Weldtolerance.eval()
	PROFILE_SHAPE = scriptOp.par.Profileshape.eval()
	CLAMP_OVERLAP = bool(scriptOp.par.Clampoverlap.eval())
	vIdxStr = scriptOp.par.Vertexindices.eval().strip()
	if vIdxStr:
		VERTEX_INDICES = set(int(s) for s in vIdxStr.split(',') if s.strip() != '')
	else:
		VERTEX_INDICES = None

	scriptOp.clear()
	try:
		scriptOp.vertexAttribs.create('uv')
	except:
		pass
	inputSop = scriptOp.inputs[0]
	if inputSop is None:
		return

	faces = []
	for prim in inputSop.prims:
		nrm = prim.normal
		faces.append({
			"pos": [(v.point.x, v.point.y, v.point.z) for v in prim],
			"idx": [v.point.index for v in prim],
			"normal": (nrm.x, nrm.y, nrm.z),
		})

	_weldIndices(faces, WELD_TOLERANCE)

	if MODE == "vertex":
		_buildVertexBevel(scriptOp, faces)
	else:
		_buildEdgeBevel(scriptOp, faces)

def _computeMinEdgeLength(faces):
	minLen = None
	seen = set()
	for f in faces:
		n = len(f["idx"])
		for i in range(n):
			j = (i+1) % n
			key = frozenset((f["idx"][i], f["idx"][j]))
			if key in seen:
				continue
			seen.add(key)
			L = _len(_sub(f["pos"][j], f["pos"][i]))
			if minLen is None or L < minLen:
				minLen = L
	return minLen

def _buildEdgeBevel(scriptOp, faces):
	effectiveWidth = BEVEL_WIDTH
	if CLAMP_OVERLAP:
		minEdge = _computeMinEdgeLength(faces)
		if minEdge is not None:
			effectiveWidth = min(BEVEL_WIDTH, minEdge * 0.49)

	for f in faces:
		f["inset"] = _faceInsetCorners(f["pos"], f["normal"], effectiveWidth)

	edgeMap = {}
	for fi, f in enumerate(faces):
		n = len(f["idx"])
		for i in range(n):
			j = (i+1) % n
			key = frozenset((f["idx"][i], f["idx"][j]))
			edgeMap.setdefault(key, []).append((fi, i))

	insetPointObjs = {}
	def getInsetPoint(fi, li):
		key = (fi, li)
		if key not in insetPointObjs:
			p = scriptOp.appendPoint()
			p.x, p.y, p.z = faces[fi]["inset"][li]
			insetPointObjs[key] = p
		return insetPointObjs[key]

	vertexArcs = {}

	for key, occurrences in edgeMap.items():
		if len(occurrences) != 2:
			continue
		(fi1, i1), (fi2, i2) = occurrences
		f1, f2 = faces[fi1], faces[fi2]
		n1, n2 = len(f1["idx"]), len(f2["idx"])
		j1, j2 = (i1+1) % n1, (i2+1) % n2

		idxA1, idxB1 = f1["idx"][i1], f1["idx"][j1]

		if f2["idx"][i2] == idxA1:
			idxA2_local, idxB2_local = i2, j2
		else:
			idxA2_local, idxB2_local = j2, i2

		startA, startB = f1["inset"][i1], f1["inset"][j1]
		endA, endB = f2["inset"][idxA2_local], f2["inset"][idxB2_local]
		origA, origB = f1["pos"][i1], f1["pos"][j1]
		edgeLen3D = _len(_sub(origB, origA))

		profA = _slerpProfile(origA, startA, endA, SEGMENTS, PROFILE_SHAPE)
		profB = _slerpProfile(origB, startB, endB, SEGMENTS, PROFILE_SHAPE)

		profAObjs = []
		for k, p in enumerate(profA):
			if k == 0:
				obj = getInsetPoint(fi1, i1)
			elif k == SEGMENTS:
				obj = getInsetPoint(fi2, idxA2_local)
			else:
				obj = scriptOp.appendPoint()
				obj.x, obj.y, obj.z = p
			profAObjs.append(obj)

		profBObjs = []
		for k, p in enumerate(profB):
			if k == 0:
				obj = getInsetPoint(fi1, j1)
			elif k == SEGMENTS:
				obj = getInsetPoint(fi2, idxB2_local)
			else:
				obj = scriptOp.appendPoint()
				obj.x, obj.y, obj.z = p
			profBObjs.append(obj)

		for k in range(SEGMENTS):
			quad = scriptOp.appendPoly(4, closed=True, addPoints=False)
			quad[0].point = profAObjs[k]
			quad[1].point = profAObjs[k+1]
			quad[2].point = profBObjs[k+1]
			quad[3].point = profBObjs[k]
			# U runs along the edge's actual length (continuous from quad to
			# quad, since it's always 0 at the A end / edgeLen3D at the B end);
			# V runs across the rounding profile's width.
			v0, v1 = k/float(SEGMENTS)*BEVEL_WIDTH, (k+1)/float(SEGMENTS)*BEVEL_WIDTH
			quad[0].uv[0], quad[0].uv[1] = 0.0, v0
			quad[1].uv[0], quad[1].uv[1] = 0.0, v1
			quad[2].uv[0], quad[2].uv[1] = edgeLen3D, v1
			quad[3].uv[0], quad[3].uv[1] = edgeLen3D, v0

		vertexArcs.setdefault(idxA1, []).append((fi1, fi2, profAObjs))
		vertexArcs.setdefault(idxB1, []).append((fi1, fi2, profBObjs))

	for fi, f in enumerate(faces):
		n = len(f["idx"])
		poly = scriptOp.appendPoly(n, closed=True, addPoints=False)
		for i in range(n):
			poly[i].point = getInsetPoint(fi, i)
		_setPolyUV(poly)

	origPosByIdx = {}
	for f in faces:
		for li, ix in enumerate(f["idx"]):
			origPosByIdx[ix] = f["pos"][li]

	for idx, arcs in vertexArcs.items():
		loop = _buildVertexCapLoop(arcs)
		if not loop or len(loop) < 3:
			continue

		origin = origPosByIdx[idx]
		involvedFaces = set()
		for (fa, fb, _) in arcs:
			involvedFaces.add(fa)
			involvedFaces.add(fb)
		avgNormal = (0.0, 0.0, 0.0)
		for fi in involvedFaces:
			avgNormal = _add(avgNormal, faces[fi]["normal"])

		n = len(loop)
		loopNormalAcc = (0.0, 0.0, 0.0)
		for i in range(n):
			a = (loop[i].x, loop[i].y, loop[i].z)
			b = (loop[(i+1) % n].x, loop[(i+1) % n].y, loop[(i+1) % n].z)
			loopNormalAcc = _add(loopNormalAcc, _cross(_sub(a, origin), _sub(b, origin)))

		if _dot(loopNormalAcc, avgNormal) < 0.0:
			loop = [loop[(-k) % n] for k in range(n)]

		_triangulateCorner(scriptOp, loop, origin, SEGMENTS)

def _buildVertexBevel(scriptOp, faces):
	pointPos = {}
	pointFaces = {}
	for fi, f in enumerate(faces):
		for li, idx in enumerate(f["idx"]):
			pointPos[idx] = f["pos"][li]
			pointFaces.setdefault(idx, []).append((fi, li))

	targets = VERTEX_INDICES if VERTEX_INDICES else set(pointPos.keys())

	faceLoops = []
	for f in faces:
		n = len(f["idx"])
		loop = []
		for i in range(n):
			idx = f["idx"][i]
			if idx in targets:
				prevIdx = f["idx"][(i-1) % n]
				nextIdx = f["idx"][(i+1) % n]
				loop.append(("cut", idx, prevIdx))
				loop.append(("cut", idx, nextIdx))
			else:
				loop.append(("orig", idx, None))
		faceLoops.append(loop)

	cutPointObjs = {}
	def getCutPoint(vertexIdx, otherIdx):
		key = (vertexIdx, otherIdx)
		if key not in cutPointObjs:
			p0, p1 = pointPos[vertexIdx], pointPos[otherIdx]
			edgeLen = _len(_sub(p1, p0))
			t = min(0.49, BEVEL_WIDTH / edgeLen) if edgeLen > 1e-9 else 0.0
			pos = _add(p0, _scale(_sub(p1, p0), t))
			p = scriptOp.appendPoint()
			p.x, p.y, p.z = pos
			cutPointObjs[key] = p
		return cutPointObjs[key]

	origPointObjs = {}
	def getOrigPoint(idx):
		if idx not in origPointObjs:
			p = scriptOp.appendPoint()
			p.x, p.y, p.z = pointPos[idx]
			origPointObjs[idx] = p
		return origPointObjs[idx]

	for loop in faceLoops:
		poly = scriptOp.appendPoly(len(loop), closed=True, addPoints=False)
		for i, item in enumerate(loop):
			if item[0] == "orig":
				poly[i].point = getOrigPoint(item[1])
			else:
				poly[i].point = getCutPoint(item[1], item[2])
		_setPolyUV(poly)

	for idx in targets:
		touching = pointFaces.get(idx, [])
		if len(touching) < 3:
			continue
		capPts = []
		for fi, li in touching:
			f = faces[fi]
			n = len(f["idx"])
			nextIdx = f["idx"][(li+1) % n]
			capPts.append(getCutPoint(idx, nextIdx))
		cap = scriptOp.appendPoly(len(capPts), closed=True, addPoints=False)
		for i, p in enumerate(capPts):
			cap[i].point = p
		_setPolyUV(cap)
