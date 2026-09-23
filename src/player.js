// The click-to-move character.
//
// Left-click lands a destination wherever the cursor touches the navmesh, and
// WASD / the arrow keys steer her directly — either way she walks the surface:
// turning toward the travel direction, keeping her feet on the mesh height,
// playing 'sprint' while she moves and 'idle' once she stops. Keyboard input
// wins over a path in progress. Movement directions are relative to the active
// camera, so 'up' is wherever the camera looks.
//
// The animation names below are the ones authored on the node in CopperCube's
// animation editor — rename them here if they are renamed there. Positions are
// read and written through the node's local 'Position', which assumes the node
// sits under an untransformed parent, as anything top-level in the scene does.

import { Entity } from './engine/index.js'

const ANIM_IDLE = "idle"
const ANIM_MOVE = "sprint"
const DEG = Math.PI / 180

// Key events arrive as Irrlicht codes, which are the VK values for letters and
// arrows — fold the ASCII range onto them in case the backend sends chars.
function normKey(k) { return k >= 97 && k <= 122 ? k - 32 : k }

export class Player extends Entity
{
	reset(nav)
	{
		this.nav = nav
		this.node = ccbGetSceneNodeFromName("Player")
		this.path = []
		this.keys = {}

		// Movement feel, in world units and degrees per second.
		this.speed = 4.5
		this.turnRate = 720
		this.arrive = 0.1
		this.faceOffset = 0	// turn the model if it runs sideways or backwards

		const rot = ccbGetSceneNodeProperty(this.node, "Rotation")
		this.yaw = rot ? rot.y : 0
		this.baseRotX = rot ? rot.x : 0
		this.baseRotZ = rot ? rot.z : 0

		this.anim = null
		ccbSetSceneNodeProperty(this.node, "AnimationBlending", true)
		ccbSetSceneNodeProperty(this.node, "BlendTimeMs", 150)
		this.play(ANIM_IDLE)

		// Listener registration is not idempotent — a pooled entity must not
		// listen twice after a suspend and respawn.
		if (!this.listening)
		{
			this.registerMouseListener()
			this.registerKeyboardListener()
			this.listening = true
		}
	}

	// Setting 'Animation' restarts the clip, so only ever set it on a change.
	play(name)
	{
		if (this.anim === name) return
		this.anim = name
		ccbSetSceneNodeProperty(this.node, "Animation", name)
		ccbSetSceneNodeProperty(this.node, "Looping", true)
	}

	on_keyPress(key) { this.keys[normKey(key)] = true }
	on_keyRelease(key) { this.keys[normKey(key)] = false }

	on_mousePress(btn)
	{
		if (btn !== 0 || isPaused()) return	// left button

		const hit = this.nav.pickAtMouse()
		if (!hit) return

		const pos = ccbGetSceneNodeProperty(this.node, "Position")
		const path = this.nav.findPath(pos, hit)
		if (path && path.length) this.path = path
	}

	// Held WASD/arrow input as a world-space direction, relative to the camera:
	// 'up' moves away from the eye along the camera's forward, flattened to XZ.
	inputDir()
	{
		const ix = (this.keys[68] || this.keys[39] ? 1 : 0) - (this.keys[65] || this.keys[37] ? 1 : 0)
		const iz = (this.keys[87] || this.keys[38] ? 1 : 0) - (this.keys[83] || this.keys[40] ? 1 : 0)
		if (!ix && !iz) return null

		const cam = ccbGetActiveCamera()
		const eye = ccbGetSceneNodeProperty(cam, "Position")
		const tgt = ccbGetSceneNodeProperty(cam, "Target")
		let fx = tgt.x - eye.x, fz = tgt.z - eye.z
		const fl = Math.sqrt(fx * fx + fz * fz)
		if (fl < 1e-6) return null
		fx /= fl; fz /= fl

		// Right of the view direction on the XZ plane — left-handed axes, so
		// facing +Z puts screen-right at +X.
		const dx = fz * ix + fx * iz
		const dz = -fx * ix + fz * iz
		const l = Math.sqrt(dx * dx + dz * dz)
		return { x: dx / l, z: dz / l }
	}

	// Steps the node along a direction, turning to face it and keeping to
	// walkable cells — a blocked step slides along whichever axis is still free.
	move(dx, dz, step, delta)
	{
		const pos = ccbGetSceneNodeProperty(this.node, "Position")

		const want = Math.atan2(dx, dz) / DEG + this.faceOffset
		let diff = (want - this.yaw) % 360
		if (diff > 180) diff -= 360
		if (diff < -180) diff += 360
		const turn = this.turnRate * delta
		this.yaw += diff > turn ? turn : diff < -turn ? -turn : diff
		ccbSetSceneNodeProperty(this.node, "Rotation", this.baseRotX, this.yaw, this.baseRotZ)

		const mx = dx * step, mz = dz * step
		let nx = pos.x, nz = pos.z
		if (this.nav.isWalkable(pos.x + mx, pos.z + mz)) { nx += mx; nz += mz }
		else if (this.nav.isWalkable(pos.x + mx, pos.z)) nx += mx
		else if (this.nav.isWalkable(pos.x, pos.z + mz)) nz += mz
		else return

		const gy = this.nav.heightAt(nx, nz)
		ccbSetSceneNodeProperty(this.node, "Position", nx, gy === null ? pos.y : gy, nz)
		this.play(ANIM_MOVE)
	}

	on_update(delta)
	{
		if (isPaused()) return

		// Manual steering always wins — it also cancels a click-path in flight.
		const input = this.inputDir()
		if (input)
		{
			this.path = []
			this.move(input.x, input.z, this.speed * delta, delta)
			return
		}

		if (!this.path.length)
		{
			this.play(ANIM_IDLE)
			return
		}

		const pos = ccbGetSceneNodeProperty(this.node, "Position")
		const next = this.path[0]
		const dx = next.x - pos.x, dz = next.z - pos.z
		const dist = Math.sqrt(dx * dx + dz * dz)

		// The last waypoint wants a real stop; the ones before it only want to
		// be passed through.
		const radius = this.path.length === 1 ? this.arrive : Math.max(this.arrive, this.speed * delta)
		if (dist <= radius)
		{
			this.path.shift()
			if (!this.path.length) this.play(ANIM_IDLE)
			return
		}

		this.move(dx / dist, dz / dist, Math.min(this.speed * delta, dist), delta)
	}

	on_suspend()
	{
		if (this.listening)
		{
			this.removeMouseListener()
			this.removeKeyboardListener()
			this.listening = false
		}
	}
}
