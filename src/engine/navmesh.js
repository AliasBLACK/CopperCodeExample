// A walkable surface, and the paths across it.
//
// The mesh is whatever geometry hangs under one scene node — in practice a
// folder of flat planes laid over everywhere a character may stand, which is
// what a navmesh is in CopperCube. At build time the triangles are read out of
// the scene's mesh buffers, carried through each node's full parent transform
// into world space, and rasterized onto a grid of cells at a fixed resolution.
// Pathfinding is A* across the cells with a greedy line-of-sight smoothing
// pass on top, so the paths that come out are waypoints, not stair-steps.
//
//   const nav = new NavMesh(ccbGetSceneNodeFromName("navMesh"), 0.5)
//   nav.pickAtMouse()        -> Vec3 where the cursor touches the mesh, or null
//   nav.findPath(from, to)   -> [Vec3, ...] world-space waypoints, or null
//   nav.heightAt(x, z)       -> ground height at a point, or null off the mesh
//
// A cell remembers the height of the surface covering it, so slopes, stairs
// and plateaus at different heights all path correctly as long as the step
// between neighbouring cells stays under maxStep. Only near-horizontal faces
// are walkable; a wall across the mesh is scenery, not floor.

const DEG = Math.PI / 180
const IDENTITY = function(v) { return v }

// Node types carrying no spatial transform: folders report "unknown", and
// overlays position themselves in screen space. Reading TRS on them just logs
// warnings, so they contribute identity instead.
const NO_TRANSFORM = { "unknown": true, "2doverlay": true, "mobile2dinput": true }

// Whether a node Type carries a spatial transform at all.
function hasTransform(t) { return t !== false && t !== undefined && !NO_TRANSFORM[t] }

// A node's local transform as a function of points: local space to parent
// space. Scale, then rotation in CopperCube's euler order — which is what
// Vec3.rotate applies — then translation.
function localTransform(node, type)
{
	const t = type === undefined ? ccbGetSceneNodeProperty(node, "Type") : type
	if (!hasTransform(t)) return IDENTITY

	const p = ccbGetSceneNodeProperty(node, "Position") || { x: 0, y: 0, z: 0 }
	const r = ccbGetSceneNodeProperty(node, "Rotation") || { x: 0, y: 0, z: 0 }
	const s = ccbGetSceneNodeProperty(node, "Scale") || { x: 1, y: 1, z: 1 }
	const rx = r.x * DEG, ry = r.y * DEG, rz = r.z * DEG
	const sx = s.x === undefined ? s : s.x
	const sy = s.y === undefined ? s : s.y
	const sz = s.z === undefined ? s : s.z

	return function(v)
	{
		const q = new Vec3(v.x * sx, v.y * sy, v.z * sz).rotate(rx, ry, rz)
		q.x += p.x; q.y += p.y; q.z += p.z
		return q
	}
}

function compose(outer, inner) { return function(v) { return outer(inner(v)) } }

// The transform that takes points in node's own space out to world space.
// There is no parent pointer and no node identity to compare, so the node is
// marked with a sentinel name, the scene is walked from the root composing
// transforms until the sentinel shows up, and the name is put back.
function worldTransform(node)
{
	const real = ccbGetSceneNodeProperty(node, "Name")
	const targetType = ccbGetSceneNodeProperty(node, "Type")
	ccbSetSceneNodeProperty(node, "Name", "__navmesh_sentinel__")

	let found = null
	const walk = function(n, xf, isRoot)
	{
		if (found) return

		// The root node carries its own property set — no Name or TRS to read,
		// and its transform is identity anyway. Asking just logs warnings. For
		// every other node Type is always readable, the sentinel can only hide
		// behind a node of the target's own type, and TRS is only worth reading
		// on nodes that have one.
		if (!isRoot)
		{
			const t = ccbGetSceneNodeProperty(n, "Type")
			const f = compose(xf, localTransform(n, t))

			if (t === targetType && ccbGetSceneNodeProperty(n, "Name") === "__navmesh_sentinel__")
			{
				found = f
				return
			}
			xf = f
		}
		forEachNode(n, function(child) { walk(child, xf, false) })
	}
	walk(ccbGetRootSceneNode(), IDENTITY, true)

	ccbSetSceneNodeProperty(node, "Name", real)
	return found || IDENTITY
}

// Every triangle under node, in world space. xf already maps node's local
// space to world, so it applies to the node's own vertices as-is; each child
// composes its own local transform on top.
function extractTriangles(node, xf, out)
{
	const buffers = ccbGetSceneNodeMeshBufferCount(node) || 0

	for (let b = 0; b < buffers; ++b)
	{
		const verts = []
		const vc = ccbGetMeshBufferVertexCount(node, b)

		for (let v = 0; v < vc; ++v)
			verts.push(xf(ccbGetMeshBufferVertexPosition(node, b, v)))

		const ic = ccbGetMeshBufferIndexCount(node, b)
		for (let i = 0; i + 2 < ic; i += 3)
		{
			const a = verts[ccbGetMeshBufferIndexValue(node, b, i)]
			const c = verts[ccbGetMeshBufferIndexValue(node, b, i + 1)]
			const d = verts[ccbGetMeshBufferIndexValue(node, b, i + 2)]
			if (a && c && d) out.push([a, c, d])
		}
	}

	forEachNode(node, function(child) { extractTriangles(child, compose(xf, localTransform(child)), out) })
}

// Height of the triangle's plane at a point on the XZ plane.
function planeY(tri, x, z)
{
	return tri.a.y - (tri.nx * (x - tri.a.x) + tri.nz * (z - tri.a.z)) / tri.ny
}

// 2D point-in-triangle on the XZ plane.
function pointInTri(x, z, a, b, c)
{
	const d1 = (b.x - a.x) * (z - a.z) - (x - a.x) * (b.z - a.z)
	const d2 = (c.x - b.x) * (z - b.z) - (x - b.x) * (c.z - b.z)
	const d3 = (a.x - c.x) * (z - c.z) - (x - c.x) * (a.z - c.z)
	const neg = d1 < 0 || d2 < 0 || d3 < 0
	const pos = d1 > 0 || d2 > 0 || d3 > 0
	return !(neg && pos)
}

// Separating-axis test between a triangle and an axis-aligned cell square on
// the XZ plane: the two box axes plus the three triangle edge normals.
function triHitsCell(tri, x0, z0, x1, z1)
{
	if (Math.max(tri.a.x, tri.b.x, tri.c.x) < x0 || Math.min(tri.a.x, tri.b.x, tri.c.x) > x1) return false
	if (Math.max(tri.a.z, tri.b.z, tri.c.z) < z0 || Math.min(tri.a.z, tri.b.z, tri.c.z) > z1) return false

	const px = [tri.a.x, tri.b.x, tri.c.x]
	const pz = [tri.a.z, tri.b.z, tri.c.z]
	const bx = [x0, x1, x0, x1]
	const bz = [z0, z0, z1, z1]

	for (let e = 0; e < 3; ++e)
	{
		const a = e, b = (e + 1) % 3
		const nx = -(pz[b] - pz[a]), nz = px[b] - px[a]

		let tmin = Infinity, tmax = -Infinity
		for (let i = 0; i < 3; ++i)
		{
			const d = px[i] * nx + pz[i] * nz
			if (d < tmin) tmin = d
			if (d > tmax) tmax = d
		}

		let bmin = Infinity, bmax = -Infinity
		for (let i = 0; i < 4; ++i)
		{
			const d = bx[i] * nx + bz[i] * nz
			if (d < bmin) bmin = d
			if (d > bmax) bmax = d
		}

		if (tmax < bmin || bmax < tmin) return false
	}

	return true
}

// Binary min-heap of [cost, value] pairs for the A* open list.
class Heap
{
	constructor() { this.a = [] }
	size() { return this.a.length }

	push(f, v)
	{
		const a = this.a
		a.push([f, v])
		let i = a.length - 1
		while (i > 0)
		{
			const p = (i - 1) >> 1
			if (a[p][0] <= f) break
			a[i] = a[p]
			i = p
		}
		a[i] = [f, v]
	}

	pop()
	{
		const a = this.a
		const top = a[0]
		const last = a.pop()
		if (a.length)
		{
			let i = 0
			for (;;)
			{
				let c = i * 2 + 1
				if (c + 1 < a.length && a[c + 1][0] < a[c][0]) ++c
				if (c >= a.length || a[c][0] >= last[0]) break
				a[i] = a[c]
				i = c
			}
			a[i] = last
		}
		return top ? top[1] : undefined
	}
}

export class NavMesh
{
	// node: the folder the navmesh geometry hangs under. cellSize is the grid
	// resolution in world units — half the character's footprint is a good
	// default, fine enough to keep paths off the edges.
	constructor(node, cellSize)
	{
		this.cellSize = cellSize || 0.5
		this.maxStep = 0.5	// height climb allowed between adjacent cells
		this.snapRange = 12	// cells searched snapping endpoints onto the mesh
		this.tris = []			// {a, b, c, nx, ny, nz} in world space
		this.cells = {}			// "cx,cz" -> {cx, cz, x, y, z}
		this.blocked = {}		// "cx,cz" keys cut out of the grid by obstacles

		if (node) this.build(node)
	}

	build(node)
	{
		const raw = []
		extractTriangles(node, worldTransform(node), raw)

		for (let i = 0; i < raw.length; ++i)
		{
			const t = raw[i]
			const ux = t[1].x - t[0].x, uy = t[1].y - t[0].y, uz = t[1].z - t[0].z
			const vx = t[2].x - t[0].x, vy = t[2].y - t[0].y, vz = t[2].z - t[0].z
			const nx = uy * vz - uz * vy
			const ny = uz * vx - ux * vz
			const nz = ux * vy - uy * vx
			const len = Math.sqrt(nx * nx + ny * ny + nz * nz)

			// Degenerate or steep faces can't be stood on. Both windings count —
			// a plane authored flipped is still a floor.
			if (len < 1e-6 || Math.abs(ny) / len < 0.5) continue

			this.tris.push({ a: t[0], b: t[1], c: t[2], nx: nx / len, ny: ny / len, nz: nz / len })
		}

		this.rasterize()
		console.log("navmesh: " + this.tris.length + " faces, " + this.cellCount() + " cells")
	}

	cellCount()
	{
		let n = 0
		for (const k in this.cells) ++n
		return n
	}

	// Marks every cell a walkable triangle touches. A cell keeps the height of
	// the topmost surface covering it.
	rasterize()
	{
		const cs = this.cellSize

		for (let i = 0; i < this.tris.length; ++i)
		{
			const t = this.tris[i]
			const x0 = Math.floor(Math.min(t.a.x, t.b.x, t.c.x) / cs)
			const x1 = Math.floor(Math.max(t.a.x, t.b.x, t.c.x) / cs)
			const z0 = Math.floor(Math.min(t.a.z, t.b.z, t.c.z) / cs)
			const z1 = Math.floor(Math.max(t.a.z, t.b.z, t.c.z) / cs)

			for (let cx = x0; cx <= x1; ++cx)
			{
				for (let cz = z0; cz <= z1; ++cz)
				{
					if (!triHitsCell(t, cx * cs, cz * cs, (cx + 1) * cs, (cz + 1) * cs)) continue

					const key = cx + "," + cz
					const y = planeY(t, (cx + 0.5) * cs, (cz + 0.5) * cs)
					const cell = this.cells[key]

					if (!cell || y > cell.y)
						this.cells[key] = { cx: cx, cz: cz, x: (cx + 0.5) * cs, y: y, z: (cz + 0.5) * cs }
				}
			}
		}
	}

	// The walkable cell at grid coordinates, or null — absent when off the
	// mesh, and when an obstacle's footprint covers it.
	cellAt(cx, cz)
	{
		const key = cx + "," + cz
		return this.blocked[key] ? null : this.cells[key]
	}

	// Whether a world position sits on walkable ground — a cell exists there
	// and no obstacle covers it.
	isWalkable(x, z)
	{
		const c = this.cellAt(Math.floor(x / this.cellSize), Math.floor(z / this.cellSize))
		return c !== null && c !== undefined
	}

	// Cuts a node's footprint out of the grid: its triangles are read out,
	// transformed to world space and projected onto XZ, and every cell a
	// projected triangle touches is marked blocked. Call it after the node is
	// in its final position — the transform is baked into the marks.
	addObstacle(node)
	{
		const tris = []
		extractTriangles(node, worldTransform(node), tris)

		const cs = this.cellSize
		let marked = 0

		for (let i = 0; i < tris.length; ++i)
		{
			const t = { a: tris[i][0], b: tris[i][1], c: tris[i][2] }
			const x0 = Math.floor(Math.min(t.a.x, t.b.x, t.c.x) / cs)
			const x1 = Math.floor(Math.max(t.a.x, t.b.x, t.c.x) / cs)
			const z0 = Math.floor(Math.min(t.a.z, t.b.z, t.c.z) / cs)
			const z1 = Math.floor(Math.max(t.a.z, t.b.z, t.c.z) / cs)

			for (let cx = x0; cx <= x1; ++cx)
			{
				for (let cz = z0; cz <= z1; ++cz)
				{
					if (!triHitsCell(t, cx * cs, cz * cs, (cx + 1) * cs, (cz + 1) * cs)) continue
					if (!this.blocked[cx + "," + cz]) { this.blocked[cx + "," + cz] = true; ++marked }
				}
			}
		}

		return marked
	}

	// The walkable cell under a world position, or the nearest one within
	// snapRange cells — ring by ring, so the first ring that hits wins.
	cellNear(x, z)
	{
		const cs = this.cellSize
		const cx = Math.floor(x / cs), cz = Math.floor(z / cs)
		const direct = this.cellAt(cx, cz)
		if (direct) return direct

		for (let r = 1; r <= this.snapRange; ++r)
		{
			let best = null, bestD = Infinity

			for (let dx = -r; dx <= r; ++dx)
			{
				for (let dz = -r; dz <= r; ++dz)
				{
					if (Math.max(Math.abs(dx), Math.abs(dz)) !== r) continue

					const c = this.cellAt(cx + dx, cz + dz)
					if (!c) continue

					const d = (c.x - x) * (c.x - x) + (c.z - z) * (c.z - z)
					if (d < bestD) { bestD = d; best = c }
				}
			}

			if (best) return best
		}

		return null
	}

	// Ground height at a world position: the topmost triangle covering it,
	// falling back to the cell underneath. Null when off the mesh entirely.
	heightAt(x, z)
	{
		let best = null

		for (let i = 0; i < this.tris.length; ++i)
		{
			const t = this.tris[i]
			if (x < Math.min(t.a.x, t.b.x, t.c.x) || x > Math.max(t.a.x, t.b.x, t.c.x)) continue
			if (z < Math.min(t.a.z, t.b.z, t.c.z) || z > Math.max(t.a.z, t.b.z, t.c.z)) continue
			if (!pointInTri(x, z, t.a, t.b, t.c)) continue

			const y = planeY(t, x, z)
			if (best === null || y > best) best = y
		}

		if (best !== null) return best

		const cell = this.cellAt(Math.floor(x / this.cellSize), Math.floor(z / this.cellSize))
		return cell ? cell.y : null
	}

	// Whether a straight walk between two points stays on the mesh, sampled
	// finely enough not to skip over a gap between cells.
	lineClear(a, b)
	{
		const dist = Math.sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z))
		const steps = Math.max(1, Math.ceil(dist / (this.cellSize * 0.4)))
		let lastY = null

		for (let s = 0; s <= steps; ++s)
		{
			const t = s / steps
			const x = a.x + (b.x - a.x) * t
			const z = a.z + (b.z - a.z) * t
			const cell = this.cellAt(Math.floor(x / this.cellSize), Math.floor(z / this.cellSize))

			if (!cell) return false
			if (lastY !== null && Math.abs(cell.y - lastY) > this.maxStep) return false
			lastY = cell.y
		}

		return true
	}

	// A* across the cells: 8-connected, no corner cutting through diagonals,
	// and no steps taller than maxStep. When the goal can't be reached the
	// path leads to the reachable cell closest to it instead.
	astar(start, goal)
	{
		const g = {}, came = {}, closed = {}
		const open = new Heap()
		const key = function(c) { return c.cx + "," + c.cz }
		const h = function(c)
		{
			const dx = Math.abs(c.cx - goal.cx), dz = Math.abs(c.cz - goal.cz)
			return dx + dz + (Math.SQRT2 - 2) * Math.min(dx, dz)
		}
		const sk = key(start)
		let best = start, bestH = h(start)

		g[sk] = 0
		open.push(bestH, start)

		while (open.size())
		{
			const cur = open.pop()
			const ck = key(cur)
			if (closed[ck]) continue
			closed[ck] = true

			if (cur === goal) { best = goal; break }

			const ch = h(cur)
			if (ch < bestH) { bestH = ch; best = cur }

			for (let dx = -1; dx <= 1; ++dx)
			{
				for (let dz = -1; dz <= 1; ++dz)
				{
					if (!dx && !dz) continue

					// A diagonal is only free if both cells it cuts across are.
					if (dx && dz && (!this.cellAt(cur.cx + dx, cur.cz) || !this.cellAt(cur.cx, cur.cz + dz)))
						continue

					const n = this.cellAt(cur.cx + dx, cur.cz + dz)
					if (!n || closed[key(n)]) continue
					if (Math.abs(n.y - cur.y) > this.maxStep) continue

					const nk = key(n)
					const ng = g[ck] + (dx && dz ? Math.SQRT2 : 1)

					if (g[nk] === undefined || ng < g[nk])
					{
						g[nk] = ng
						came[nk] = cur
						open.push(ng + h(n), n)
					}
				}
			}
		}

		if (best === start && start !== goal) return null

		const path = [best]
		while (path[0] !== start) path.unshift(came[key(path[0])])
		return path
	}

	// World-space waypoints from 'from' to 'to', both Vec3-likes. Ends are
	// snapped to the nearest walkable cell, the cell path is smoothed by line
	// of sight, and the exact 'to' point closes the list. Null when either end
	// is too far off the mesh.
	findPath(from, to)
	{
		const start = this.cellNear(from.x, from.z)
		const goal = this.cellNear(to.x, to.z)
		if (!start || !goal) return null

		const cells = this.astar(start, goal)
		if (!cells) return null

		const points = []
		for (let i = 0; i < cells.length; ++i)
			points.push(new Vec3(cells[i].x, cells[i].y, cells[i].z))

		// Walk from the actual position, not the start cell's centre — the
		// first corner is often already visible and the detour is free to drop.
		const anchor = new Vec3(from.x, start.y, from.z)
		const out = []
		let i = -1

		for (;;)
		{
			const a = i < 0 ? anchor : points[i]
			let j = points.length - 1
			while (j > i + 1 && !this.lineClear(a, points[j])) --j
			out.push(points[j])
			if (j === points.length - 1) break
			i = j
		}

		// Finish on the exact spot asked for when the goal cell was reached —
		// but only when that spot is itself walkable: a click inside an
		// obstacle's footprint lands on the nearest open cell instead.
		if (cells[cells.length - 1] === goal)
		{
			const y = this.isWalkable(to.x, to.z) ? this.heightAt(to.x, to.z) : null
			out[out.length - 1] = y === null
				? new Vec3(goal.x, goal.y, goal.z)
				: new Vec3(to.x, y, to.z)
		}

		return out
	}

	// Closest point where a ray meets the mesh, or null. The direction needn't
	// be normalised — the nearest hit wins either way.
	raycast(o, d)
	{
		let bestT = Infinity

		for (let i = 0; i < this.tris.length; ++i)
		{
			const t = this.tris[i]
			const e1x = t.b.x - t.a.x, e1y = t.b.y - t.a.y, e1z = t.b.z - t.a.z
			const e2x = t.c.x - t.a.x, e2y = t.c.y - t.a.y, e2z = t.c.z - t.a.z

			const px = d.y * e2z - d.z * e2y
			const py = d.z * e2x - d.x * e2z
			const pz = d.x * e2y - d.y * e2x
			const det = e1x * px + e1y * py + e1z * pz
			if (Math.abs(det) < 1e-9) continue

			const inv = 1 / det
			const tx = o.x - t.a.x, ty = o.y - t.a.y, tz = o.z - t.a.z
			const u = (tx * px + ty * py + tz * pz) * inv
			if (u < 0 || u > 1) continue

			const qx = ty * e1z - tz * e1y
			const qy = tz * e1x - tx * e1z
			const qz = tx * e1y - ty * e1x
			const v = (d.x * qx + d.y * qy + d.z * qz) * inv
			if (v < 0 || u + v > 1) continue

			const hit = (e2x * qx + e2y * qy + e2z * qz) * inv
			if (hit > 1e-6 && hit < bestT) bestT = hit
		}

		if (bestT === Infinity) return null
		return new Vec3(o.x + d.x * bestT, o.y + d.y * bestT, o.z + d.z * bestT)
	}

	// Where the mouse cursor touches the mesh, or null when it misses.
	pickAtMouse()
	{
		const eye = ccbGetSceneNodeProperty(ccbGetActiveCamera(), "Position")
		const at = getMouse3DPos()
		return this.raycast(eye, { x: at.x - eye.x, y: at.y - eye.y, z: at.z - eye.z })
	}
}
