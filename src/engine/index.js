// CopperCode: the engine behind a CopperCube project.
//
// Importing this module is what installs the runtime — the shims, the vector
// types, the collision engine, and the globals the rest of it is reached
// through. Import it before anything else in your entry module, then call
// boot() with your root Entity once your own modules have loaded.
//
//     import { boot, Entity } from './engine/index.js'
//     import { Main } from './main.js'
//
//     boot(Main)
//
// This folder is the engine and nothing else — no import in it reaches outside
// it. Keep it that way and it can be copied whole into another project, or
// replaced wholesale by a newer one from upstream, without a merge.
//
// Much of the engine is reached off global rather than by import, in keeping
// with how CopperCube's own API reads. Installed by the runtime:
//
//   sceneRoot, stash            the authored nodes everything is parented to
//   spawnEntity, runningEntities  the entity pool and the frame it runs in
//   Vec2, Vec3, crash           vectors and 2D collision
//   Random, localize            dice and strings
//   console, forEachNode, findNode, getMouse3DPos
//   readMouseRay, hitsNode, rayPlaneHit
//
// and by boot:
//
//   scheduler, setTimeout, sleep
//   pause, isPaused
//   picker, interface
import './runtime.js'

export { boot } from './boot.js'
export { Entity } from './entity.js'
export { Scheduler } from './scheduler.js'
export { Interface, Frame, Panel, Text, Button, fontMultiplier } from './interface.js'
export { Tween, TweenManager, Easing } from './tween.js'
export { Picker, readMouseRay, hitsNode, rayPlaneHit, setRayLength } from './picking.js'
export { pickPrefab, turnPrefab } from './prefabs.js'
export { NavMesh } from './navmesh.js'
