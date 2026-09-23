import { Entity, NavMesh } from './engine/index.js'
import { Player } from './player.js'

export class Main extends Entity
{
	reset()
	{
		// Bake the walkable surface out of the planes under the navMesh folder,
		// then hand the character to it. Clicks on the mesh are hers to walk to.
		global.nav = new NavMesh(ccbGetSceneNodeFromName("navMesh"), 0.5)

		// Table obstacles: the authored table is parked under 'props' as a
		// prefab source, so clones are re-parented to scene-root and forced
		// visible — a clone under a hidden folder would never render.
		const table = ccbGetSceneNodeFromName("table")
		if (table)
		{
			const rot = ccbGetSceneNodeProperty(table, "Rotation")
			const spots = [[0, -5, 0], [-4.5, 1, 90], [3.5, 3, 35]]
			for (const s of spots)
			{
				const t = ccbCloneSceneNode(table)
				ccbSetSceneNodeParent(t, sceneRoot)
				ccbSetSceneNodeProperty(t, "Position", s[0], 0, s[1])
				ccbSetSceneNodeProperty(t, "Rotation", rot.x, rot.y + s[2], rot.z)
				global.nav.addObstacle(t)
			}
		}

		spawnEntity(Player, global.nav)
	}
}
