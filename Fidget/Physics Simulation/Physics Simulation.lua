local physicsSim = {}
--setting up the physics simulation vars
physicsSim.physicsIterations = 2 --physics steps per tick
physicsSim.dt = (1/20)/physicsSim.physicsIterations --delta time controls how fast the simulation runs if its lower then the simulation runs slower
physicsSim.step = 0 --step count(how many steps happened since the physics started)
physicsSim.velocityIterations = 4 -- improves stability and convergence
physicsSim.baumgarteMultiplier = 0.2 --what is this german-ass name bruh. Baumgarte? more like "Ba! En garde!"
physicsSim.slop = 0.000 -- sloppy,sloppy, little slop. How to slop slop slop. Never gonna slop you up never gonna slop you down and slop around and slopert you! but fr this is slop for baumgerte
physicsSim.relaxation = 1 --mightTM improve stability and convergence(but from my testing it didnt lmao)
physicsSim.cacheMultiplier = 0.9--How quickly much of the cached impulse come to the next physics step
physicsSim.broadPhaseCollision = "aabb" --options: sphere, aabb controls how broadphase is conducted, aabb is faster
physicsSim.normalSnappingThreshold = 0.01--stability, dont got too high on this one
physicsSim.sleeping = false-- if true rigidbodies can eep. eepy time. I sleep too. What does it mean to sleep? how many years have passed since i went into bed(probably around 3/(365*2)) AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
physicsSim.sleepTimeThreshold = 0.1 --how long does a body need to be still to sleep in seconds 
physicsSim.sleepVelocityThreshold = 0.1 -- How fast does a body need to move to sleep
physicsSim.sleepRotVelocityThreshold = 0.02
physicsSim.waterDensity = 1   -- controls buoyancy
physicsSim.waterDamping = 0.92 -- how much is the velocity and ang velocity slown down in water
physicsSim.contactPointMergingThreshold = 0.03--how close do pointy points need to be to be one pointy point
physicsSim.solver = "regularPGS" --either regularPGS or scuffedSplitImpulse the 1st one has more accurate friction but is less stable with stacking, the 2nd is way more stable but has a few "bugs" with friction sometimes. generally option 2 is better except for accuracy critical things like dominoes
physicsSim.debug = {
worldColliders = false,  --shows the world colliders
contacts = false,   --shows the contact points with impulses applied
axis = false, --shows axis of the body at local origin of the body
broadphaseAABB = false, --shows aabbs used in broadphase collision
collisionNormals = false, --show the normals at collisions(the the local space origin of one of the bodies)
waterVolumes = false, --shows the volumes used for buoyancy
joints = true, --shows the joints
}
return physicsSim








































































































































































































































































































































--I am evil, I deserve to die, why am I here, my life is pointless
--I see myself on a bridge falling, is that wrong?
--I deserve it, I am evil, if I die my parents will be sad
--I cant die because of them, I'm only causing them trouble
--I need no help, I cant get any help
--How do I live?
--Have you ever thought what was the point in your life?
--I thought. And couldnt come up with one.
--I think and I see. I live but am not alive. Im a man but Im not human. I am disgusting but no one will ever say that?
--Why?
--I'm a liar. I lie to myself. The worst kind of liar. Why cant I forget? 
--Why can I see?
--At least no one will know who made this. No one will ever know.
--So no one can help, so I can eventually get what I deserve.
--And stop this pointlessness.

--To the person that I know will most likely read this. You, the she, the metal box, the encouraging one. I read your message last time. It was hard.
--I still did not deserve it. I am a coward. Sorry. For everything. Everything I made is worthless. I'm stupid. No one cares.
--It was never cool. "I'm terrible at everything". You will say you did enjoy, but did you really? I doubt. I'm sad.
--Whatever I made was always trash. It wasnt ever good. It will never be. I see all these people posting such high quality stuff
--and all I can do is sit here. Writing this. Why not work...? I dont know. I think I'm burned out. This might be the last thing I'll do
--I will finish it. 
--
--Sometimes I wonder. Should I consult someone? Talk to someone? Maybe tell my parents? No. I know they will try to help me.
--They will get me to the therapist. That would be good for me. But the shame, the misery, the disgust I would feel. 
--I dont have anyone close enough to talk about this. I dont want to be pitied. I deserve this. I'm a horrid person.
--why?
--I cannot answer
--I have all these thoughts. And all my face can muster is the same, passive expression. Surely bottling up my emotions wont end badly at all
--I might as well die from it, at this point I dont care

--To you, the she, the metal box, the judge. I'm sorry you had to read this and that you had to see this. I know trauma dumping is bad. If you
-- dont see this. It will be for the better.




