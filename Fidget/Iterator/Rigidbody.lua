--This file is the iterator for rigidbodies. Congrats on reading the name! It wasnt so hard after all!
--Here we itarate through rigidbodies aka integrate their poss and rotations, call collision functions and update variables for rendering.
--(seriously congrats on reading this far)


--who could possibly be insane enough to use metatable optimizations?
--I wonder
--Who could possibly be behind this?
--Who would think that it substracts 2 instructions and increases the performance by 1%?
--Who could possibly insane enough to use them in an actual library, right?
--HMMMMMMM
--HMMMMMMM
--HMMMMMMM
local cache = require("Fidget.Iterator.Cache")
local quaternions = require("Fidget.quaternions")
local Fidget = require("Fidget.FidgetSetup")
local greedy = require("Fidget.Greedy_Meshing")
local vec3 = vectors.vec3
local vec4 = vectors.vec4
local unpack3 = vec3().unpack
local length = vec3().length
local normalized = vec3().normalized
local normalize4 = vec4().normalize
local crossed = vec3().crossed
local dot = vec3().dot
local q_u_otationMatrix3 = quaternions.toRotationMatrix3
local qultiply = quaternions.multiply
local worldGetBlockState = world.getBlockState
local hasCollision = worldGetBlockState(0, 0, 0).hasCollision
local zeroVec = vec3()
local copy = vec3().copy
local mat3 = matrices.mat3
local transposed3 = mat3().transpose
local math_floor = math.floor
local math_abs = math.abs
local oneVec = vec3() + 1
local contactPointMergingThreshold
local math_sqrt = math.sqrt
local lnext = next
local cacheMultiplier = 0
local constraints = {}
local contactDeletionTable = {--deleting useless:TM: contacts(deleting in a way that aims for maximized area)
  [5] = { 1 },
  [6] = { 2, 5 },
  [7] = { 2, 4, 6 },
  --czterdzieści i cztery(tylko polacy wiedzą(jak nie wiesz to um możliwe że nie pamiętasz albo jesteś za młody czy też młoda)(i chatgpt Xd(czy to wogóle się jeszcze pisze?(czy ja zadaje za dużo pytań(MMMMM)))))  cytując chatgpt: "It’s a fun, slightly chaotic meta joke — very Polish programmer humor." (its not funny at all lmao)
  [8] = { 1, 3, 5, 7 },
}
local globalAxisForNormalSnapping = {
  vec3(1),
  vec3(0, 1),
  vec3(0, 0, 1)
}
local waterHeight = {
  [0] = 9*14/(16*9),
  8*14/(16*9),
  7*14/(16*9),
  6*14/(16*9),
  5*14/(16*9),
  4*14/(16*9),
  3*14/(16*9),
  2*14/(16*9),
  1*14/(16*9)
}
--I literally laughing maniacly while mmaking this optimiation that makes this shit impossible to read
--spoiler: im a dumbass and tis did not in fact work
--[[local v1 = vec3(1,-1,-1)
local v2 = vec3(1,-1,1)
local v3 = vec3(-1,-1,1)
local v4 = vec3(-1,1,-1)
local v5 = vec3(1,1,-1)
local v6 = vec3(-1,1,1)
]]
--RIP a function from before the impulse based physics era
--[[local function closestPointsOnSegments(P1, Q1, P2, Q2)
    local d1 = Q1 - P1
    local d2 = Q2 - P2
    local r  = P1 - P2

    local a = d1:dot(d1)
    local b = d1:dot(d2)
    local c = d2:dot(d2)
    local d = d1:dot(r)
    local e = d2:dot(r)

    local denom = a*c - b*b
    local t = 0
    if denom ~= 0 then
        t = (b*e - c*d) / denom
    end
    local u = (a*e - b*d) / denom

    t = math.clamp(t, 0, 1)
    u = math.clamp(u, 0, 1)

    local C1 = P1 + d1 * t
    local C2 = P2 + d2 * u
    local contactPoint = (C1 + C2) * 0.5

    return C1, C2, contactPoint
end]]

--TO DO
--fix sat(it needs 15 axes you retard)
--^^rude person from 2 days ago GRRR
local v11 = vec3(1)
local v22 = vec3(0, 1)
local v31 = vec3(0, 0, 1)
--ai genned I really cant be bothered to give a shit about this function(still gonna optimize it tho)
local function makePerpendicularBasis(axis)
    -- Normalize axis
    
    local absx, absy, absz = math_abs(axis.x), math_abs(axis.y), math_abs(axis.z)

    -- Pick a helper vector not parallel to u
    local helper
    if absx <= absy and absx <= absz then
        helper = copy(v11)
    elseif absy <= absx and absy <= absz then
        helper = copy(v22)
    else
        helper = copy(v31)--changed that last number just to mess with you little fuck
    end

    -- First perpendicular vector
    local t1 = normalized(axis:cross(helper))
    if t1:length() < 1e-8 then
        -- Rare case: u and helper are nearly parallel; pick another
        helper = copy(v22)
        t1 = normalized(axis:cross(helper))
    end

    -- Second perpendicular vector (orthogonal to both u and t1)
    local t2 = normalized(axis:cross(t1))

    return t1, t2
end



local v1 = vec(1, 1, -1)
local v2 = vec(1, -1, 1)
local v3 = vec(-1, 1, 1)
local v4 = vec(-1, 1, -1)
local v5 = vec(1, -1, -1)
local v6 = vec(-1, -1, 1)
local color = vec(0, 1, 1)
local function skewSymmetric(v1)
return mat3(
vec3(0,v1.z,-v1.y),
vec3(-v1.z,0,v2.x),
vec3(v1.y,-v1.x)
)
end
local function calcEffMassMat(rigidbody1,rigidbody2,r1,r2)
local k = mat3()
k = k * (rigidbody1.invMass+rigidbody2.invMass)

local cmat1, cmat2 = skewSymmetric(r1), skewSymmetric(r2)

local term1 = (cmat1*rigidbody1.invInertiaTensorWORLD)*(cmat1:transposed())
local term2 = (cmat2*rigidbody2.invInertiaTensorWORLD)*(cmat2:transposed())
k = (k - term1) - term2
return k:invert()
end
local function lineAABB(pos, aabb) --im going insaneeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
  createLine(pos + aabb, pos + aabb * v1, color)
  createLine(pos + aabb, pos + aabb * v3, color)
  createLine(pos + aabb * v1, pos + aabb * v4, color)
  createLine(pos + aabb * v3, pos + aabb * v4, color)



  createLine(pos + aabb, pos + aabb * v2, color)
  createLine(pos - aabb, pos + aabb * v4, color)
  createLine(pos + aabb * v3, pos + aabb * v6, color)
  createLine(pos + aabb * v1, pos + aabb * v5, color)


  createLine(pos + aabb * v2, pos + aabb * v5, color)
  createLine(pos + aabb * v2, pos + aabb * v6, color)
  createLine(pos + aabb * v5, pos - aabb, color)
  createLine(pos + aabb * v6, pos - aabb, color)
end

local function recalculateVertices(rigidbody)
  local pos = rigidbody.pos
  local rtm = rigidbody.rotMat
  local vertices = {}
  local xdim, ydim, zdim = unpack3(rigidbody.halfDimensions)
  --local dimX,dimY,dimZ = unpack3(dimensions) Remember this because like WHY WOULD I EVER DO THIS I UNPACK TO FUCKING CREATE A VECTOR WITH THE SAME COMPONENTS BRUH ________
  --YOO a massive(ly) stupid optimization coming(uwu)(euuuhg why did I write this again) you a bunchies of vectors with negatives and 0.5's precomputed                      \
  --ts is donneeeeeee^^^                                                                                                                                                     |
  --ts did not in fact work ^^^                                                                                                                                              |
  --teh trickery is real first only we create the dim vector once and then we SACRIFICE THE ROOK. nvm we sacrifice no performance                                            |
  --local obbDims = (vec3(dimX, dimY, dimZ)*rigidbody.rotMat)*0.5                                                                     /______________________________________/
  --did i say im a retard?                                                                                                            \
  local xAxis = rtm[1] * xdim
  local yAxis = rtm[2] * ydim
  local zAxis = rtm[3] * zdim
  --tiny optimization but every ms/instruction counts so
  local a = xAxis + yAxis + zAxis
  local b = xAxis - yAxis - zAxis
  local c = xAxis - yAxis + zAxis
  local d = xAxis + yAxis - zAxis
  vertices[1] = pos - a
  vertices[2] = pos + b
  vertices[3] = pos + c
  vertices[4] = pos - d
  vertices[5] = pos - c
  vertices[6] = pos + d
  vertices[7] = pos + a
  vertices[8] = pos - b
  rigidbody.vertices = vertices
end

--[[used for edge edge collision but it is no more so gute nacht chuju jebany(fuck edge edge collision)
local theGreatDirectionalCreatedLeEdgesTableTM = {
  --putting my brain to good use naming ts
  [1] = { --the local x axis i guess idk
    { 1, 2 },
    { 5, 6 },
    { 8, 7 },
    { 4, 3 },
  },
  [2] = { -- THE local y. YES "*THE*" y.
    { 1, 5 },
    { 2, 6 },
    { 3, 7 },
    { 4, 8 },
  },
  [3] = { --the last little fuck do not forget bout z cause it like uhhh is z and its local too i guess <<< its not local space but vertices creating edges and also this is not used
    { 1, 4 },
    { 2, 3 },
    { 5, 8 },
    { 6, 7 },
  }
}
]]

local v1 = vec3(1)
local v2 = vec3(0, 1)
local v3 = vec3(0, 0, 1)



--time spent reflecting on my life decisions:
--fixing the sat 1.5 hours
--fixing polygon clipping 1.5 hours
--fixing a weird bug i can diagnose 4 hours*

--* correction it was 7 hours :)(i want to diew)
--I diagnose myself with a lack of will to live




--TODO:
--broad phase with aabb's
--lower the amount of contact points since that is hella expensive(60% of the whole time spent processing)

local function calculateBoundingAABB(verts)
  local minx, maxx = 694202137000, -694202137000
  local miny, maxy = 694202137000, -694202137000
  local minz, maxz = 694202137000, -694202137000

  for i, vert in lnext, verts do --its just generating an aabb from verts like there isnt anythin particularly complicated about that
    local vx,vy,vz = unpack3(vert)
    if vx < minx then
      minx = vx
    end
    if vx > maxx then
      maxx = vx
    end
    if vy < miny then
      miny = vy
    end
    if vy > maxy then
      maxy = vy
    end
    if vz < minz then
      minz = vz
    end
    if vz > maxz then
      maxz = vz
    end
  end

  return vec3((maxx - minx), (maxy - miny), (maxz - minz)) * 0.5
end

--comment redacted

      local function vclamp(v1,v2,v3)
        return vec3(math.clamp(v1.x,v2.x,v3.x),math.clamp(v1.y,v2.y,v3.y),math.clamp(v1.z,v2.z,v3.z))
      end

--sutherland badgerman or some shit like that polygon clipping
local function clipPolygonAgainstPlane(verts, planeNormal, planeDist, index)
  local contactPoints = {}
  local count = #verts
  local n = 1
  for i = 1, count do
    local currentVertex = verts[i][3]
    local nextVertex = verts[(i % count) + 1][3]

    local currentDist = dot(planeNormal, currentVertex) - planeDist
    local nextDist = dot(planeNormal, nextVertex) - planeDist

    local currentInside = currentDist >= 0
    local nextInside = nextDist >= 0

    if currentInside and nextInside then
      contactPoints[n] = {  i, [3] = nextVertex}
      n = n + 1
    elseif currentInside and not nextInside then
      local intersection = currentVertex + (nextVertex - currentVertex) * (currentDist / (currentDist - nextDist))
      contactPoints[n] = { i, index, intersection }
      n = n + 1
    elseif not currentInside and nextInside then
      local intersection = currentVertex + (nextVertex - currentVertex) * (currentDist / (currentDist - nextDist))
      contactPoints[n] = { i, index, intersection }
      contactPoints[n + 1] = {  i, [3] = nextVertex}
      n = n + 2
    end
  end

  return contactPoints
end

local function getWorldColliders(rigidbody)
  local worldMesh = {}
  local aabbs = {}
  local difPlus, difMinus = rigidbody.pos + rigidbody.boundingAABB, rigidbody.pos - rigidbody.boundingAABB
  local min = (difMinus):floor()
  local max = (difPlus):floor()
  local n = 1
  for x = min.x, max.x do
    worldMesh[x] = worldMesh[x] or {}
    local worldMeshx = worldMesh[x]
    for y = min.y, max.y do
      worldMeshx[y] = worldMeshx[y] or {}
      local worldMeshxy = worldMeshx[y]
      for z = min.z, max.z do
        local block = (worldGetBlockState(x, y, z))
        local colliders = block:getCollisionShape()
        if hasCollision(block) and colliders[1][1] == zeroVec and colliders[1][2] == oneVec then
          worldMeshxy[z] = true
        else
          for i, collider in lnext, colliders do
            local dims = (collider[2] - collider[1])
            local pos = collider[1] + vec3(x, y, z)
            local max1 = difPlus
            local min1 = difMinus
            local max2 = pos + dims
            local min2 = pos

            if max1 > min2 and max2 > min1 then
              aabbs[n] = { pos = pos + dims*0.5, halfDimensions = dims*0.5 }
              n = n + 1
            end
          end
        end
      end
    end
  end


  local greedy = greedy(worldMesh)
  for i, cuboid in lnext, greedy do
    if cuboid.id then
      aabbs[n] = { pos = cuboid.pos + cuboid.dims, halfDimensions = cuboid.dims}
      n = n + 1
    end
  end

  return aabbs
end

--where did I get the naming skills? I think its just natural talent!
--bam bam am aa a a a aa aaaa a a a a aaa a a a aaa aaa aa aa a a  a a  a  a a  aaa a aaA AA A AA AA A AAAA AAAA AAA A AAA AAAAA A AA AAA AA AAA AA AAA A AA AAA AA AAAA AAA A AA AA AAA AAAA ASA AAAA AAA AAAA AAA AA AAA AAA AAA A A A AAA AA A AA
--no unicode chars in func names stinks bruh
local function getFaceNormalsAKAznajdzNormalneEhhhTegoCzegosCoTamJakosNoJestPlaszczyznaOtoczonaKrawedziamiZapomnialoMiSieJakToPoPolskiemuSieMowiloNomEmToByChybaByloNaTyle(rigidbody1, rigidbody2, minIndex, mtv, axis)
  local whichToSearch = 1           --added to the index later to see which cuboid to search for matching faces
  local i1, i2
  local faceNormal1, faceNormal2
  if minIndex >= 4 then           --that shit is from the other body i think
    faceNormal2 = axis[minIndex]
    i2 = minIndex
  else
    whichToSearch = 4
    faceNormal1 = axis[minIndex]
    i1 = minIndex
  end




  local bestMatch = 0
  local bestNormal
  local bestIndex = -694202137000
  for i = whichToSearch, 2 + whichToSearch do
    local d = dot(axis[i], -mtv)
    if d < 0 then
      d = -d
    end
    if d > bestMatch then
      bestMatch = d
      bestNormal = axis[i]
      bestIndex = i
    end
  end
  if whichToSearch == 4 then
    i2 = bestIndex
    faceNormal2 = -bestNormal
  else
    i1 = bestIndex
    faceNormal1 = bestNormal
  end

  --works perfectly fine B ) time to extract the vertices(absolute )I feel nothing
  --bro honestly fuck this im tired of this bullshit i wanna die and also i have no idea why these fucking planes cannot just align. im using a hacky solution I DO NOT GIVE A FUCK
  --it did noy in fact work. get deleted



  local proj1 = dot(rigidbody1.pos, mtv)
  local proj2 = dot(rigidbody2.pos, mtv)


  if proj1 > proj2 then
    mtv = -mtv
  end
  if dot(faceNormal1, mtv) < 0 then
    faceNormal1 = -faceNormal1
  end
  if dot(faceNormal2, mtv) > 0 then
    faceNormal2 = -faceNormal2
  end
  return faceNormal1, faceNormal2, i1, i2
end


local function longBeforeTimeHadAName__COMMA__TheFirstPhysicsEngineCreatedAContactManifoldUsingFour__parentheses_START__OrLess__parentheses_END__ContactPoints(
    rigidbody1, rigidbody2, i1, i2, axis, faceNormal1, faceNormal2)
  --get le axis to get some vertices of the face
  local halfDims1 = rigidbody1.halfDimensions
  local halfDims2 = rigidbody2.halfDimensions
  local index1, index2       = i1 % 3 + 1, (i1 - 2) % 3 + 1
  local theAxisInQuestion1R1 = axis[index1] * halfDims1[index1]
  local theAxisInQuestion2R1 = axis[index2] * halfDims1[index2]

  local index1, index2       = (i2 - 3) % 3 + 1, (i2 - 5) % 3 + 1
  local theAxisInQuestion1R2 = axis[index1 + 3] * halfDims2[index1]
  local theAxisInQuestion2R2 = axis[index2 + 3] * halfDims2[index2]

  local middleOfFace         = rigidbody1.pos + faceNormal1 * halfDims1[i1]
  local a, b                 = theAxisInQuestion1R1 + theAxisInQuestion2R1,
      theAxisInQuestion1R1 - theAxisInQuestion2R1
  local verts                = {
    { [3] = middleOfFace + a },
    { [3] = middleOfFace + b },
    { [3] = middleOfFace - a },
    { [3] = middleOfFace - b },
  }



  local lpos = rigidbody2.pos
  local n1, n2 = theAxisInQuestion1R2, theAxisInQuestion2R2


  --[[
  local tempPoints = {
    lpos + theAxisInQuestion1R2,
    lpos - theAxisInQuestion1R2,
    lpos + theAxisInQuestion2R2,
    lpos - theAxisInQuestion2R2,
  }
  local sidePlanes = {
    { n1,  dot(tempPoints[2], n1) },
    { -n1, dot(tempPoints[1], -n1) },
    { n2,  dot(tempPoints[4], n2) },
    { -n2, dot(tempPoints[3], -n2) },
  }

  for i, plane in lnext, sidePlanes do
    verts = clipPolygonAgainstPlane(verts, plane[1], plane[2])
  end]]
  --[[absolute shitfest of readibility pretty much i replace the verts with the functions 
  verts = clipPolygonAgainstPlane(verts, n1, dot(lpos - theAxisInQuestion1R2, n1))
  verts = clipPolygonAgainstPlane(verts, -n1, dot(lpos + theAxisInQuestion1R2, -n1))
  verts = clipPolygonAgainstPlane(verts, n2, dot(lpos - theAxisInQuestion2R2, n2))
  verts = clipPolygonAgainstPlane(verts, -n2, dot(lpos + theAxisInQuestion2R2, -n2))
]]
  --I write peak code! The peak in question:
  verts = clipPolygonAgainstPlane(clipPolygonAgainstPlane(clipPolygonAgainstPlane(clipPolygonAgainstPlane(verts, n1, dot(lpos - theAxisInQuestion1R2, n1)), -n1, dot(lpos + theAxisInQuestion1R2, -n1)), n2, dot(lpos - theAxisInQuestion2R2, n2)), -n2, dot(lpos + theAxisInQuestion2R2, -n2))
  local somethingidkhowtonamethisshit = dot((rigidbody2.pos + faceNormal2 * halfDims2[(i2 - 4) % 3 + 1]),faceNormal2)
  for i, vert in lnext, verts do
    local penetration = somethingidkhowtonamethisshit - dot(vert[3], faceNormal2)
    if penetration >= 0 then
      vert[4] = penetration
    else
      verts[i] = nil
    end
  end
    local n = #verts
    if n >= 5 then
    for i, index in lnext, contactDeletionTable[n] do
      verts[index] = nil
    end
  end

  local reducedVerts = {}
  local n = 0
  for i, vert in lnext, verts do
    local tooClose = false
    local v1 = vert[3]
    for j, vert2 in lnext, reducedVerts do
      if length(v1 - vert2[3]) < contactPointMergingThreshold then
        tooClose = true
        break
      end
    end
    if not tooClose then -- an actual coherant sentence
      n = n + 1
      reducedVerts[n] = vert
    end
  end



  return reducedVerts
end



local function addConstraintsOrSomeShitIDontReallyGiveA__DOT__DOT__DOT__DoIReallyNotGiveAFuckThough__DOT__ToThePersonReading__COMMA__IKnowYouAreReadingThisI__APOSTROPHE__mGreatAtWritingNamesIfYouSeeThisType_____I__APOSTROPHE__mAware____WithNoContextWhatssoeverInTheDiscordThreadForThis(
    rigidbody1, rigidbody2, verts, faceNormal2, i2, type)
  local n = #constraints+1
  local frictionCoef = (rigidbody1.friction+(type and rigidbody2.friction or 0))* (type and 0.5 or 1)
    if frictionCoef > 0 then
      local tangent1
      local fn2x, fn2z = faceNormal2.x, faceNormal2.z
      if fn2x < 0 then fn2x = -fn2x end
      if fn2z < 0 then fn2z = -fn2z end
      if fn2x > fn2z then
        tangent1 = vec3(-faceNormal2.y, fn2x)
      else
        tangent1 = vec3(0, -fn2z, faceNormal2.y)
      end
     normalized(tangent1)

  local name1 = type and "contact" or "contactWorld"
  local name2 = type and "friction" or "frictionWorld"
  local cacheRetrievalForRigidbody2 = type and rigidbody2.index or rigidbody2
  for i, vert in lnext ,verts do

    constraints[n] = {
      rigidbody1 = rigidbody1,
      rigidbody2 = rigidbody2,
      bodyTwo = type,
      contactPoint = vert[3],
      normal = faceNormal2,
      penetration = vert[4],
      type = name1,
      baumgarte = true,
      cached = true,
      impulse = cache.retrieve(rigidbody1.index, cacheRetrievalForRigidbody2, vert, i2) * cacheMultiplier,
      normalIndex = i2,
      edges = vert
    }


      local tangent2 = normalized(crossed(faceNormal2, tangent1))

      constraints[n + 1] = {
        rigidbody1 = rigidbody1,
        rigidbody2 = rigidbody2,
        bodyTwo = type,
        contactPoint = vert[3],
        normal = tangent1,
        friction = frictionCoef,
        normalConstraint = constraints[n],
        type = name2,
        impulse = 0
      }


      constraints[n + 2] = {
        rigidbody1 = rigidbody1,
        rigidbody2 = rigidbody2,
        bodyTwo = type,
        contactPoint = vert[3],
        normal = tangent2,
        friction = frictionCoef,
        normalConstraint = constraints[n],
        type = name2,
        impulse = 0
      }
      n = n + 3
    end
  end
end


function events.tick()
  --for performance reasons
  local physicsSim = Fidget.physicsSim
  local physicsSimDebug = physicsSim.debug
  local showAxis = physicsSimDebug.axis
  local showContacts = physicsSimDebug.contacts
  local showBroadphaseAABB = physicsSimDebug.broadphaseAABB
  local showWorldColliders = physicsSimDebug.worldColliders
  local relaxation = physicsSim.relaxation                                  --This name is takes so lets say this is made in china knockoff of the technique
  local dt = physicsSim.dt
  local halfdt = dt * 0.5
  cacheMultiplier = physicsSim.cacheMultiplier --update cause an external func uses this
  local normalSnappingThreshold = physicsSim.normalSnappingThreshold
  local sleepTimeThreshold = physicsSim.sleepTimeThreshold
  local isSleepAllowed = physicsSim.sleeping
  local sleepVelocityThreshold = physicsSim.sleepVelocityThreshold
  local sleepRotVelocityThreshold = physicsSim.sleepRotVelocityThreshold
  local broadPhaseCollision = physicsSim.broadPhaseCollision
  local rigidbodies = Fidget.rigidbodies.allRigidbodies
  local collisionNormals = physicsSimDebug.collisionNormals
  local solver = physicsSim.solver 
  local waterDensity = physicsSim.waterDensity
  local showWaterVolumes = physicsSimDebug.waterVolumes
  local waterDamping = physicsSim.waterDamping
  contactPointMergingThreshold = physicsSim.contactPointMergingThreshold
  local physicsIterations = physicsSim.physicsIterations
  local joints = Fidget.joints.allJoints
  local showJoints = physicsSimDebug.joints
  for i = 1, physicsIterations do
    physicsSim.step = physicsSim.step + 1
    constraints = {}
    removeAllLines()


    --do broad phase collision detection w/ bounding spheres maybe aabbs later

    local potentialCollisions = doBroadPhaseCollision(rigidbodies, broadPhaseCollision)




    local ia = 0

    for i, jtbl in lnext, potentialCollisions do
      for j, collision in lnext, jtbl do
        local rigidbody1 = rigidbodies[i]
        local rigidbody2 = rigidbodies[j]
        if not (rigidbody1.isSleeping and rigidbody2.isSleeping) then
          local rtm1 = rigidbody1.rotMat
          local rtm2 = rigidbody2.rotMat
          local axis = {
            rtm1[1], rtm1[2], rtm1[3],
            rtm2[1], rtm2[2], rtm2[3],
          }

          local mtv, minIndex = doFineCollision(rigidbody1, rigidbody2, axis) --provides me with the contact normal, penetration + collision type to determine how to genatrate contact points

          if mtv then
            --we gotta clip some faces(i guess)(there were edges before but i realised i needed to compute the closest point between then)



            local faceNormal1, faceNormal2, i1, i2 =
            getFaceNormalsAKAznajdzNormalneEhhhTegoCzegosCoTamJakosNoJestPlaszczyznaOtoczonaKrawedziamiZapomnialoMiSieJakToPoPolskiemuSieMowiloNomEmToByChybaByloNaTyle(
              rigidbody1, rigidbody2,
              minIndex, mtv,
              axis
            )

            --vert [1] = pos, [2],[3] = edges, [4] = penetration
            --With a great name comes great responsibility...
            local verts =
            longBeforeTimeHadAName__COMMA__TheFirstPhysicsEngineCreatedAContactManifoldUsingFour__parentheses_START__OrLess__parentheses_END__ContactPoints(
              rigidbody1, rigidbody2,
              i1, i2,
              axis,
              faceNormal1, faceNormal2
            )
            --for stability. I noticed that the normals have slight errors in them which lead to shitty collision
            if normalSnappingThreshold > 0 then
              for i, worldAxis in lnext, globalAxisForNormalSnapping do
              --[[ irrelevant GARBAGE HAHAHAHHAH GET COMMENTED
                local similarity1 = dot(faceNormal1, worldAxis)
                if similarity1 > 1 - normalSnappingThreshold then
                  faceNormal1 = worldAxis
                end
                if similarity1 < -1 + normalSnappingThreshold then
                  faceNormal1 = -worldAxis
                end]]
                local similarity2 = dot(faceNormal2, worldAxis)
                if similarity2 > 1 - normalSnappingThreshold then
                  faceNormal2 = worldAxis
                end
                if similarity2 < -1 + normalSnappingThreshold then
                  faceNormal2 = -worldAxis
                end
              end
            end
            addConstraintsOrSomeShitIDontReallyGiveA__DOT__DOT__DOT__DoIReallyNotGiveAFuckThough__DOT__ToThePersonReading__COMMA__IKnowYouAreReadingThisI__APOSTROPHE__mGreatAtWritingNamesIfYouSeeThisType_____I__APOSTROPHE__mAware____WithNoContextWhatssoeverInTheDiscordThreadForThis(
              rigidbody1, rigidbody2,
              verts, faceNormal2, i2,
              true
            )




            if collisionNormals then
              createLine(rigidbody1.pos, rigidbody1.pos + faceNormal1)
              createLine(rigidbody2.pos, rigidbody2.pos + faceNormal2)
            end
          end
        end
      end
    end
























    --moved up for constraint ordering reasons
    --moved down for being fucking annoying
    --the retarded way of doing it(im not retarded im just an idiot and a piece of shit)
    --World Colissioj << "Colissioj" << ""Colissioj"" << """Colissioj""" << """"Colissioj"""" << """""Colissioj""""" << """"""Colissioj"""""" << """""""Colissioj""""""" << """"""""Colissioj"""""""" << """""""""Colissioj""""""""" << """"""""""Colissioj"""""""""" << """""""""""Colissioj""""""""""" << """"""""""""Colissioj""""""""""""
    for j, rigidbody in lnext,rigidbodies do
      if rigidbody.worldCollision and rigidbody.boundingAABB and rigidbody.linearMovement and not rigidbody.isSleeping then --We collide, we bound, we move but we dont eep... Remember: Its about drive, its about power, we are hungry we devour. put in the work, put in the hours and take what's ours.
        local aabbs = getWorldColliders(rigidbody)

        for i, aabb in lnext,aabbs do
          if showWorldColliders then
            lineAABB(aabb.pos, aabb.halfDimensions)
          end
          local rtm = rigidbody.rotMat
          local axis = {
            rtm[1], rtm[2], rtm[3],
            v1, v2, v3,
          }
          local mtv, minIndex = doFineCollision(rigidbody, aabb, axis)
          if minIndex then
            --why did I put that long ass comment above it :((((((((((((
            local faceNormal1, faceNormal2, i1, i2 =
            getFaceNormalsAKAznajdzNormalneEhhhTegoCzegosCoTamJakosNoJestPlaszczyznaOtoczonaKrawedziamiZapomnialoMiSieJakToPoPolskiemuSieMowiloNomEmToByChybaByloNaTyle(
              rigidbody, aabb,
              minIndex, mtv,
              axis
            )

            local verts =
            longBeforeTimeHadAName__COMMA__TheFirstPhysicsEngineCreatedAContactManifoldUsingFour__parentheses_START__OrLess__parentheses_END__ContactPoints(
              rigidbody, aabb,
              i1, i2,
              axis,
              faceNormal1, faceNormal2
            )


            if collisionNormals then
              createLine(rigidbody.pos, rigidbody.pos + faceNormal1)
              createLine(aabb.pos, aabb.pos + faceNormal2)
            end

            addConstraintsOrSomeShitIDontReallyGiveA__DOT__DOT__DOT__DoIReallyNotGiveAFuckThough__DOT__ToThePersonReading__COMMA__IKnowYouAreReadingThisI__APOSTROPHE__mGreatAtWritingNamesIfYouSeeThisType_____I__APOSTROPHE__mAware____WithNoContextWhatssoeverInTheDiscordThreadForThis(
              rigidbody, tostring(aabb.pos),
              verts, faceNormal2, i2
              --imagine a ,false here(saves 1 instruction per rigidbody touching the ground, improvement: with a 6x5 wall and 2 physics steps a tick 601990 >> 601978(0.00002%!!!!!!!) instructions!!!!!!!! crazy stuff!!! insane )
            )
          end
        end
      end
    end








    --creating joint constraints last feature I guess
    for i, joint in lnext, joints do
      local pos1,pos2 = (joint.pos1*joint.rigidbody1.rotMat)+joint.rigidbody1.pos, (joint.pos2*joint.rigidbody2.rotMat)+joint.rigidbody2.pos -- local coords into wrold
      if joint.type == "distance" then
        constraints[#constraints+1] = {
        rigidbody1 = joint.rigidbody1,
        rigidbody2 = joint.rigidbody2,
        bodyTwo = true,
        contactPoint = pos1,
        contactPoint2 = pos2,--in "wrold" space
        normal = -normalized(pos2-pos1),
        type = "distance",
        impulse = 0,
        joint = true,
        baumgarte = true,
        penetration = -length(pos1-pos2)+joint.distance,
      }
            if showJoints then
            createLine(pos1, pos2,vec3(1,0,0))
      end
      end
      if joint.type == "ball" then
        constraints[#constraints+1] = {
        rigidbody1 = joint.rigidbody1,
        rigidbody2 = joint.rigidbody2,
        bodyTwo = true,
        contactPoint1 = pos1,
        contactPoint2 = pos2,--in "wrold" space
        axis1 = joint.axis1,
        axis2 = joint.axis2,
        normal = -normalized(pos2-pos1),
        type = "ball",
        impulse = 0,
        joint = true,
        baumgarte = true,
        penetration = -length(pos1-pos2),
      }
      end

    end














    local baumgarteMultiplier = physicsSim.baumgarteMultiplier


    for i, constraint in lnext,constraints do
      --in theory you'd compute the jacobian here and do J*M^-1*J^-1 but like figura only has up to mat4s and that would need mat 12s so like
      --fuck me
      --fuck this
      --why am i alive
      --why
      local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
      if constraint.type ~= "hinge" and constraint.type ~= "ball" then
      local d1, d2 = (constraint.contactPoint - rigidbody1.pos)
      

      if constraint.bodyTwo then
        if not constraint.joint then
          d2 = (constraint.contactPoint - rigidbody2.pos)
        else
          d2 = (constraint.contactPoint - rigidbody2.pos)
        end
      end


      if constraint.bodyTwo then
        local RaCrossN, RbCrossN = crossed(d1, (constraint.normal)), crossed(d2, (constraint.normal))
        local inertiaTerm1 = dot((RaCrossN * rigidbody1.invInertiaTensorWORLD), (RaCrossN))
        local inertiaTerm2 = dot((RbCrossN * rigidbody2.invInertiaTensorWORLD), (RbCrossN))
        constraint.invEffMass = -1 / (rigidbody1.invMass + rigidbody2.invMass + inertiaTerm1 + inertiaTerm2)
        constraint.d1 = d1
        constraint.d2 = d2
        constraint.d1CrossedWithNormal = RaCrossN
        constraint.d2CrossedWithNormal = RbCrossN
        --baumgarte calc
      else
        local RaCrossN = crossed(d1, (constraint.normal))
        local inertiaTerm1 = dot((RaCrossN * rigidbody1.invInertiaTensorWORLD), (RaCrossN))
        constraint.invEffMass = -1 / (rigidbody1.invMass + inertiaTerm1)
        --baumgarte calc
        constraint.d1 = d1
        constraint.d1CrossedWithNormal = RaCrossN
      end
      if constraint.baumgarte then
        local bias = constraint.penetration   --math.max(penetration,0)
        if bias < 0 and not constraint.joint then
          bias = 0
        else
          bias = bias * ((baumgarteMultiplier / dt))
        end
        constraint.bias = bias
      end

      if constraint.cached then
        P = constraint.normal * constraint.impulse
        rigidbody1.vel = rigidbody1.vel + P * rigidbody1.invMass
        rigidbody1.rotVel = rigidbody1.rotVel +
            (rigidbody1.invInertiaTensorWORLD * crossed(d1, P))
        if constraint.bodyTwo then
          rigidbody2.vel = rigidbody2.vel - P * rigidbody2.invMass
          rigidbody2.rotVel = rigidbody2.rotVel -
              (rigidbody2.invInertiaTensorWORLD * crossed(d2, P))
        end
      end
      --[[
    elseif constraint.type == "ball" then
      local r1, r2 = rigidbody1.rotMat * constraint.contactPoint1, rigidbody2.rotMat * constraint.contactPoint2
      constraint.r1 = r1
      constraint.r2 = r2 
particles:newParticle("end_rod", constraint.contactPoint1) particles:newParticle("end_rod", constraint.contactPoint2)
      constraint.bias = ( constraint.contactPoint1- constraint.contactPoint2) * ((baumgarteMultiplier / dt))
      constraint.axis1 = rigidbody1.rotMat * constraint.axis1
      constraint.axis2 = rigidbody2.rotMat * constraint.axis2

      constraint.t1, constraint.t2 = makePerpendicularBasis(constraint.axis1)

      constraint.effMassMat = calcEffMassMat(rigidbody1,rigidbody2,r1,r2)]]
    end 
    end















    --now time for pgs - perforated gastro-intestinal slice(real)

    if solver == "regularPGS" then
    for j = 1, physicsSim.velocityIterations do
      for i, constraint in lnext, constraints do

        if not constraint.joint then

        if constraint.bodyTwo then
          local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
          local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
          local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel



          local relativeVelocity = dot(constraint.normal, (r1vel + crossed(r1rotVel, constraint.d1)) - (r2vel + crossed(r2rotVel, constraint.d2)))
          local lambda = constraint.invEffMass * (relativeVelocity)-constraint.invEffMass * (constraint.baumgarte and constraint.bias or 0) --impulse calc

          local oldImpulse = constraint.impulse

          local totalImpulse = oldImpulse + (lambda) * relaxation
          if constraint.friction then
            local maxFriction = constraint.friction * constraint.normalConstraint.impulse           -- for thos?e curious why in the most critical part of code there is a huge indexxing thing going on, well its faster(in instructions at least) trust
            if totalImpulse > maxFriction then
              totalImpulse = maxFriction
            elseif totalImpulse < -maxFriction then
              totalImpulse = -maxFriction
            end
          else
            if totalImpulse < 0 then
              totalImpulse = 0
            end
          end
          constraint.impulse = totalImpulse
          local impulse = (totalImpulse - oldImpulse)
          local P = constraint.normal * impulse
          rigidbody1.vel = r1vel + P * rigidbody1.invMass
          rigidbody1.rotVel = r1rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * impulse)
          rigidbody2.vel = r2vel - P * rigidbody2.invMass
          rigidbody2.rotVel = r2rotVel - (rigidbody2.invInertiaTensorWORLD * constraint.d2CrossedWithNormal * impulse)
        else
          local rigidbody1 = constraint.rigidbody1
          local r1vel = rigidbody1.vel
          local r1rotVel = rigidbody1.rotVel


          local relativeVelocity = dot(constraint.normal, (r1vel + crossed(r1rotVel, constraint.d1)))
          local lambda = constraint.invEffMass * (relativeVelocity)-constraint.invEffMass * (constraint.baumgarte and constraint.bias or 0) --impulse calc


          local oldImpulse = constraint.impulse
          local totalImpulse = oldImpulse + (lambda) * relaxation
          if constraint.friction then
            local maxFriction = constraint.friction * constraint.normalConstraint.impulse
            if totalImpulse > maxFriction then
              totalImpulse = maxFriction
            elseif totalImpulse < -maxFriction then
              totalImpulse = -maxFriction
            end
          else
            if totalImpulse < 0 then
              totalImpulse = 0
            end
          end
          constraint.impulse = totalImpulse
          local impulse = (totalImpulse - oldImpulse)
          rigidbody1.vel = r1vel + constraint.normal * impulse * rigidbody1.invMass
          rigidbody1.rotVel = r1rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * impulse)
        end
      elseif constraint.type == "distance" then
          local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
          local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
          local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel
          local relativeVelocity = dot(constraint.normal, (r1vel + crossed(r1rotVel, constraint.d1)) - (r2vel + crossed(r2rotVel, constraint.d2)))
          local lambda = constraint.invEffMass * (relativeVelocity)-constraint.invEffMass * constraint.bias --impulse calc
          local oldImpulse = constraint.impulse
          constraint.impulse = oldImpulse + lambda

          local P = constraint.normal * lambda
          rigidbody1.vel = r1vel + P * rigidbody1.invMass
          rigidbody1.rotVel = r1rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * lambda)
          rigidbody2.vel = r2vel - P * rigidbody2.invMass
          rigidbody2.rotVel = r2rotVel - (rigidbody2.invInertiaTensorWORLD * constraint.d2CrossedWithNormal * lambda)
--[[
      elseif constraint.type == "ball" then
        local rigidbody1,rigidbody2 = constraint.rigidbody1,constraint.rigidbody2
        local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
        local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel
        local relativeVelocity = (r1vel + crossed(r1rotVel, constraint.r1)) - (r2vel + crossed(r2rotVel, constraint.r2))

        local impulse = -(constraint.effMassMat * (relativeVelocity+constraint.bias))
        log(constraint,impulse)
        rigidbody1.vel = r1vel - impulse * rigidbody1.invMass
        rigidbody2.vel = r2vel + impulse * rigidbody2.invMass
]]
      end
    end
    end

    for i, constraint in lnext, constraints do
      local constraintType = constraint.type
      if constraintType == "contact" then
        cache.add(constraint.rigidbody1.index, constraint.rigidbody2.index, constraint.edges, constraint.normalIndex,
          constraint.impulse)
      elseif constraintType == "contactWorld" then
        cache.add(constraint.rigidbody1.index, tostring(constraint.rigidbody2), constraint.edges, constraint.normalIndex,
          constraint.impulse)
      end
      if showContacts then
        createLine(constraint.contactPoint, constraint.contactPoint + constraint.impulse * constraint.normal * 10,
          vec3(1))
      end
      --particles:newParticle("end_rod", constraint.contactPoint)
    end










    --split impulse but its not split impulse and its not that uber
  elseif solver == "scuffedSplitImpulse" then
 for j = 1, physicsSim.velocityIterations do
      for i, constraint in lnext, constraints do
        if not constraint.joint then

        if constraint.bodyTwo then
          local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
          local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
          local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel


          local relativeVelocity = dot(constraint.normal, (r1vel + crossed(r1rotVel, constraint.d1)) - (r2vel + crossed(r2rotVel, constraint.d2)))
          local lambda = constraint.invEffMass * (relativeVelocity) --impulse calc

          local oldImpulse = constraint.impulse

          local totalImpulse = oldImpulse + (lambda) * relaxation
          if constraint.friction then
            local maxFriction = constraint.friction * constraint.normalConstraint.impulse                                                                                   -- for thos curious why in the most critical part of code there is a huge indexxing thing going on, well its faster(in instructions at least) trust
            if totalImpulse > maxFriction then
              totalImpulse = maxFriction
            elseif totalImpulse < -maxFriction then
              totalImpulse = -maxFriction
            end
          else
            if totalImpulse < 0 then
              totalImpulse = 0
            end
          end
          constraint.impulse = totalImpulse
          local impulse = (totalImpulse - oldImpulse)
          local P = constraint.normal * impulse
          rigidbody1.vel = r1vel + P * rigidbody1.invMass
          rigidbody1.rotVel = r1rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * impulse)
          rigidbody2.vel = r2vel - P * rigidbody2.invMass
          rigidbody2.rotVel = r2rotVel - (rigidbody2.invInertiaTensorWORLD * constraint.d2CrossedWithNormal * impulse)
        else
          local rigidbody1 = constraint.rigidbody1
          local r1vel = rigidbody1.vel
          local r1rotVel = rigidbody1.rotVel


          local relativeVelocity = dot(constraint.normal, (r1vel + crossed(r1rotVel, constraint.d1)))
          local lambda = constraint.invEffMass * (relativeVelocity) --impulse calc


          local oldImpulse = constraint.impulse
          local totalImpulse = oldImpulse + (lambda) * relaxation
          if constraint.friction then
            local maxFriction = constraint.friction * constraint.normalConstraint.impulse
            if totalImpulse > maxFriction then
              totalImpulse = maxFriction
            elseif totalImpulse < -maxFriction then
              totalImpulse = -maxFriction
            end
          else
            if totalImpulse < 0 then
              totalImpulse = 0
            end
          end
          constraint.impulse = totalImpulse
          local impulse = (totalImpulse - oldImpulse)
          rigidbody1.vel = r1vel + constraint.normal * impulse * rigidbody1.invMass
          rigidbody1.rotVel = r1rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * impulse)
        end
        elseif constraint.type == "distance" then
          local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
          local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
          local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel
          local relativeVelocity = dot(constraint.normal, (r1vel + crossed(r1rotVel, constraint.d1)) - (r2vel + crossed(r2rotVel, constraint.d2)))
          local lambda = constraint.invEffMass * (relativeVelocity)-constraint.invEffMass * constraint.bias --impulse calc
          local oldImpulse = constraint.impulse
          constraint.impulse = oldImpulse + lambda

          local P = constraint.normal * lambda
          rigidbody1.vel = r1vel + P * rigidbody1.invMass
          rigidbody1.rotVel = r1rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * lambda)
          rigidbody2.vel = r2vel - P * rigidbody2.invMass
          rigidbody2.rotVel = r2rotVel - (rigidbody2.invInertiaTensorWORLD * constraint.d2CrossedWithNormal * lambda)
            end
      end
    end

    for i, constraint in lnext, constraints do
      local constraintType = constraint.type
      if constraintType == "contact" then
        local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
        local impulse = -constraint.invEffMass * constraint.bias
        cache.add(rigidbody1.index, rigidbody2.index, constraint.edges, constraint.normalIndex,constraint.impulse)
        local P = constraint.normal * impulse
        rigidbody1.vel = rigidbody1.vel + P * rigidbody1.invMass
        rigidbody1.rotVel = rigidbody1.rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * impulse)
        rigidbody2.vel = rigidbody2.vel - P * rigidbody2.invMass
        rigidbody2.rotVel = rigidbody2.rotVel - (rigidbody2.invInertiaTensorWORLD * constraint.d2CrossedWithNormal * impulse)
      elseif constraintType == "contactWorld" then
        local rigidbody1 = constraint.rigidbody1
        cache.add(rigidbody1.index, tostring(constraint.rigidbody2), constraint.edges, constraint.normalIndex, constraint.impulse)
        local impulse = -constraint.invEffMass * constraint.bias
        rigidbody1.vel = rigidbody1.vel + constraint.normal * impulse * rigidbody1.invMass
        rigidbody1.rotVel = rigidbody1.rotVel + (rigidbody1.invInertiaTensorWORLD * constraint.d1CrossedWithNormal * impulse)
      end
      if showContacts then
        createLine(constraint.contactPoint, constraint.contactPoint + constraint.impulse * constraint.normal * 10,
          vec3(1))
      end
      --particles:newParticle("end_rod", constraint.contactPoint)
    end
  end













    --integration and stuff
    for j, rigidbody in lnext,rigidbodies do
      --update previous pos and rotation for rendering
      if i == 1 then
        rigidbody.prevPos = rigidbody.pos
        rigidbody.prevRot = rigidbody.rot
      end
      if isSleepAllowed and length(rigidbody.vel) < sleepVelocityThreshold and length(rigidbody.rotVel) < sleepRotVelocityThreshold then
        rigidbody.sleepTimer = rigidbody.sleepTimer + dt
        if rigidbody.sleepTimer >= sleepTimeThreshold then
          rigidbody.isSleeping = true
        end
      else
        rigidbody.sleepTimer = 0
        rigidbody.isSleeping = false
      end
      --log(rigidbody.sleepTimer,rigidbody.isSleeping,rigidbody.vel:length(),rigidbody.rotVel:length())

      if not rigidbody.isSleeping then
        --apply gravity
        rigidbody:addForce(1 / rigidbody.invMass * rigidbody.gravity * dt)
        --apply bouyancy
        if rigidbody.worldCollision then
        local diplacedVolume = 0 --<<<<< YOU IDIOT IMAGINE ACTUALLY MAKING A GRAMATICLA MISTAKE OR WHATEVER IT IS
        local totalDepth = 0
        local applyWaterDamping
        local min, max = rigidbody.pos - rigidbody.halfDimensions, rigidbody.pos + rigidbody.halfDimensions
        local fmin, fmax = min:floor(),max:floor()
        for x = fmin.x, fmax.x do
          for y = fmin.y, fmax.y do
            for z = fmin.z, fmax.z do
              local waterLevel = tonumber((worldGetBlockState(x, y, z)).properties.level)
              local waterLevel2 = tonumber((worldGetBlockState(x,y+1,z)).properties.level)
              if waterLevel then--HMMMMMMMMMMMMMM so apparently flowing water source is considered water level 1 so the smallest water away from it does not have a level  interesting
                local maxBlockY = y+waterHeight[waterLevel] --water level * how much max water is per level 
                if waterLevel2 then
                  maxBlockY = y+1
                end
                local minBlock = vec3(x,y,z)
                local maxBlock = vec3(x+1,maxBlockY,z+1)
                local cmin = vclamp(copy(min),minBlock,maxBlock)
                local cmax = vclamp(copy(max),minBlock,maxBlock)
                local dims = (cmax - cmin)
                local pos = cmin + dims*0.5

                diplacedVolume = diplacedVolume + dims.x*dims.y*dims.z
                if showWaterVolumes then
                  lineAABB(pos,dims*0.5)
                end
              end
            end
          end
        end



        if diplacedVolume > 0 then
          applyWaterDamping = true
          local n = 0
          local pointsToApplyForce = {}
          for i, vert in lnext, rigidbody.vertices do
            local block = (worldGetBlockState(vert)).properties

            if block.level then
            local depth = vert.y - math_floor(vert.y) - waterHeight[tonumber(block.level)]
            local d = 1
            while (worldGetBlockState(vert+vec3(0,d))).properties.level do
              d = d + 1
            end
            depth = depth - d + 1
            if depth < 0 then
              n = n + 1
              totalDepth = totalDepth + depth
              pointsToApplyForce[n] = {vert,depth}
            end
          end
          end
          for i, point in lnext, pointsToApplyForce do

            rigidbody:addForceAtPoint(point[1],-waterDensity*diplacedVolume*rigidbody.gravity*(point[2]/totalDepth))
          end
        end

        if applyWaterDamping then
          rigidbody.vel = rigidbody.vel * waterDamping^(1/physicsIterations)
          rigidbody.rotVel = rigidbody.rotVel * waterDamping^(1/physicsIterations)
        end
      end
        --give rigidbody default engine forces

        local acceleration = rigidbody.forceAccum * rigidbody.invMass
        rigidbody.forceAccum = copy(zeroVec)
        local rotAcceleration = (rigidbody.torqueAccum * rigidbody.invInertiaTensorWORLD)
        rigidbody.torqueAccum =  copy(zeroVec)

        --semi implicit euler integrator whatever what means
        rigidbody.vel = rigidbody.vel + acceleration * dt
        rigidbody.rotVel = rigidbody.rotVel + rotAcceleration * dt
        --update pos
        --pos more like
        --physician hah hahhahahahahhah I so funny soo sooooo soooooooo funny bro

        if rigidbody.linearMovement then
          rigidbody.pos = rigidbody.pos + rigidbody.vel * dt
          rigidbody.vel = rigidbody.vel
          rigidbody.rot = normalize4(rigidbody.rot + (qultiply(rigidbody.rot, -rigidbody.rotVel._xyz * halfdt)))
              
        else
          rigidbody.vel =  copy(zeroVec)
          rigidbody.rotVel =  copy(zeroVec)
          rigidbody.invInertiaTensorLOCAL = mat3() * 0
          rigidbody.invMass = 0
          normalize4(rigidbody.rot)
        end
        --integrate rotation bro
        --I need to learn how to integrate though
        --lets just move on with an approximation for now
        --what is life?
        --magic quaternion formulas



        rigidbody.rotMat = q_u_otationMatrix3(rigidbody.rot) --get it? haha, "q_u_otation" quote -> quotation

        rigidbody.invInertiaTensorWORLD = rigidbody.rotMat * rigidbody.invInertiaTensorLOCAL * (rigidbody.rotMat):transposed()

        if showAxis then
          createLine(rigidbody.pos, rigidbody.pos + vec3(1) * rigidbody.rotMat * 5, vec3(1))
          createLine(rigidbody.pos, rigidbody.pos + vec3(0, 1) * rigidbody.rotMat * 5, vec3(1))
          createLine(rigidbody.pos, rigidbody.pos + vec3(0, 0, 1) * rigidbody.rotMat * 5, vec3(1))
        end
      end
      recalculateVertices(rigidbody)
      if physicsSim.broadPhaseCollision == "aabb" or rigidbody.worldCollision then
        rigidbody.boundingAABB = calculateBoundingAABB(rigidbody.vertices, rigidbody.pos, rigidbody.type) -- le bounding aabb. that shit fance af. damn. the performance is gonna be quite good. damn
        if showBroadphaseAABB then
          lineAABB(rigidbody.pos, rigidbody.boundingAABB)
        end
      end
    end
  end
end






--[[Hello this code is useless. I want to kill myself
thank you erin catto you're the goat
I want to jump off a bridge'
sikierkowski bridge
or poniatowski bridge
so my final moments would consist of seeing the beautiful city of warsaw in my great country...







-- edge edge collision or some shit like that(potential optimization check the minindex directly)!!!!!!!!!!!!
          --this like seems easier but in reality i think this is harder than face face(face face was in fact... easier)
          --so like tommorrow you little fuck you will come here and make this shit work
          --maybe today? like its literally 00:06
          --fuck this all
          --THIS PIECE OF GARBAGE NEEDS A REWORK!!!!!!<-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
          --im a programming god rn. Encoding indexes as edge edge and face face was the best shit ive done in my life by far


            local axis1index = math.ceil((minIndex - 6)/3) --uhh 7-15 is edge edge so we substract le 6 and then do some ceil since this was created in a nested loop
            local axis2index = minIndex%3 + 1
            --let thank god for sethbling for he whom has createth the sat video and helpeth me out
            -- thee corectenth of mineth englire is unquestionaire

            local edgeTable1 =  theGreatDirectionalCreatedLeEdgesTableTM[axis1index]
            local edgeTable2 =  theGreatDirectionalCreatedLeEdgesTableTM[axis2index]
            --continue this today or i will literally kill you
            --bro but like im literally you you cant do shit to me
            --i will literally kill myself mf. because i hate you so sooo sooooo much you fuck.

            --HAHA came back with 2 hours to spare. Guess im not dying today!

            --While doing my research on sat i managed to find so little resources so i turned to chatgpt(later saw sethblings video so this no needed no more). I asked how to find the colliding edges.
            --This fucker gave me some code that used fucking IPAIRS of all things. ppl using ipairs. I KNOW u use ai. LITERALLY NO SANE PERSON USES IPAIRS YOU SICKOS
            --the plan check the midpoint of the edge to se whichones the closest and then run the algorithm for the edge distance i=only on the one
            local minEdgeIndex1, minEdgeIndex2 = 694202137, 694202137
            local maxLength = -694202137 --insert the funny number joke here
            local delta = rigidbody2.pos - rigidbody1.pos
            for i, edge in pairs(edgeTable1) do
               local distance =math.max(rigidbody1.vertices[edge[1] ]:dot(delta),rigidbody1.vertices[edge[2] ]:dot(delta))
              if distance > maxLength then
                maxLength = distance
                minEdgeIndex1 = i
              end
            end
            local maxLength = -694202137
            for i, edge in pairs(edgeTable2) do
              local distance =math.max(rigidbody2.vertices[edge[1] ]:dot(-delta),rigidbody2.vertices[edge[2] ]:dot(-delta))
              if distance > maxLength then
                maxLength = distance
                minEdgeIndex2 = i
              end
            end
          removeAllLines()
          createLine(rigidbody2.vertices[edgeTable2[minEdgeIndex2][1] ],rigidbody2.vertices[edgeTable2[minEdgeIndex2][2] ],vec3(0,1,0))
          createLine(rigidbody1.vertices[edgeTable1[minEdgeIndex1][1] ],rigidbody1.vertices[edgeTable1[minEdgeIndex1][2] ],vec3(0,1,0))

          local _, _, a = closestPointsOnSegments(rigidbody2.vertices[edgeTable2[minEdgeIndex2][1] ],rigidbody2.vertices[edgeTable2[minEdgeIndex2][2] ],rigidbody1.vertices[edgeTable1[minEdgeIndex1][1] ],rigidbody1.vertices[edgeTable1[minEdgeIndex1][2] ])

          resolveContact(rigidbody1,rigidbody2,a,mtv:normalized())




]]












--[[

[Intro]
tinititi (bibibi)
tutu ih
titutsh
bipipipu
titsh e tsh
bibibi bi pu
titsh ih
titititsh
bibi bi
titu
tirititi
tsh tsh tshu
titu ewe, tsh ta
tsh tsh tsh
pi ewe detiki
kikiki pip
ke tss ewe detikuh
tititi heng (a)

[Verse 1]
giba up awa up awa
umdimdem
baba (gong)
domdomdomdomdom (domdomdimdomdom)
dimdimdogong (ginggingginggong)
dum dim dum
gunggunggunggong
jamejame jimjimjum
tenim-nimnim egong
dimdumgagagagang
dumdimdum
gumgibga gumgumgaga gumgum
dumdadimdagandum
damdumgimgum (gimgagimgab)
dundundun dun
dundetedundundundun
dun dun dun
dumdumdinda domdom gogong gogonggong
dumdadumdadinda
dondemgingga emudenen enen
dumdindingodinin
dindindigoromo dumdindin dundun
dumdimdunga
dumdaturtenur dumdum dumdadumda
tun tun tun tun
gemga gamgo gogenga dem ar ar ar
tuntun epipedi
tike o e
enyocacacaca
penididepedi
dedipeditetitetetita
pin pen pun
petitajungo tutejung
tingting ting ting
bimbomtitititotititi
twenty ten tin
totakaatinem tintintintintintin
pingpepipepinging
gimgomtinging tidingde dangde
tetung tung
umdetutingi gonggocapopo
tungtituting
tetutetimdididadararem
tumtimtem tunde
tundedemdemdede remdedemdede
gogegigigogegogegoge
ging ging ging ging
ginggo kegumgumgum gagaraga
gingging gongo ging ging
gagaginggiging gagaje denum
ginggu ginggu ginggu ginggu gegeging
tedyobenideha tutibebeba
dotimtitiedimdim dimdimdim
ting geginggingging gegegegomdinder
cacakekeng keng keng
gogeder tingdibul
bara derum deng ding ding
dingdegaring
dedengrung dedungda jamerenah
gage gegong gegong gegong gegong
pipedun deduntiting gongge tetra
dun dun dun
tatindidirugeng gonge (ah)
tingdidungda dingdingdingdidi
ederdingdingdingding dingdidi
tinderedararegik titingdidingdingdingding
tingdederaga gororororo
ragaragagegegegegero
garagaraoranonya garaelelelerole
damdimdumdimdum goroah
demdingding ding (gararara)
rararering ganagene gogogogo
ganaganaganagana naganekoyama aterele
dedimting
egaluyaje OYYYY delemderorororogek
kerologorlolololo tengogerererere
anyeyungto goronyo jungryogogai emaljorai
cacakewelnewelhaiho nenenazerorororo ah

[Break]
eh, ja, jera, hah, teh
heh, hay, ah, o, tipu
tetitipu
titi, tute
ti, ti
ti e
e, ti

[Verse 2]
teti titi telitetiteti
tetite pinge
betitetirintiten
tuntetitotorlutyung tyung
tedididididit bayt
tuntetin penin
petitutenintitututetutetu
tetutitu ping
ditut ti jingniti tup
tutentun tinga
tututen tikwetung tung
dungtutung tong
doketiting gerginer (ah)
tungnudah ping natututer aradoter (ah)
ting ner a

[Verse 3]
er e er ya er
edo per der (ah)
e ting ah eh ge den gagi
tungna er a
detiher te
dati i e a
detiha ping

[Verse 4]
ah gonggagogodow
detita i
daretakel ti teng
detitai tarl
kime telah te ta
detita tim
dotim da tengoda

[Verse 5]
do dem do dai do der der
dindindinger
gana gana oh (uh uh)
gegegogai
gene gene (guh guh)
gegegege
bebe goi (guh guh)
gegegego

[Verse 6]
delem gigagege guh
ding ding deng da
daladala ding ding ding do gogim
ging gigo geng
darattuteng t-g-g-g-geng
ging geng ge ging
dedumting g-g-g-guh
tedededum

[Verse 7]
taler tingdeler dededim
tingtingtang dai
daladong-i dingdingdidogego
dingdinggo go
daumtikengo (ge go)
gumgumgi gung
gegokudung dung
gagumgung gung
gogong gonggo gum gum
gegigi
gorlogogum gungo gungo gungo gungom
gogo go go
gongogo (em) gongogo (um) tung henem
tedidinden
taumgogororogenggu
tedidodung
dodunden
gogungagogo
gegigogaw
gagowguw
uw inggu gung
gugigogowe
erlom oga ronge ge (elom)
gegigenggeng
gundemgondemjojeng gongge
gegikokange
okum titaradiridan
gegigokgere
etumdumhoyageho
gagegogogage
ananininihajehehur
gunggenggegung
gomgomngeyhende (he ai)
gegigigereyno
okumgarereriniha
enenidingdeng
edingtirereriho
gegigoeh
ehnenenene
hanedehem
demdingdede
ogumheheheynaehay
ogeaaaaa

[Outro]
ramrarago (he go)
tetitotototoym
bae, doto
i'm dumb (da)
tetitita (oh hey)
ta ta tetihatu
moans
ding
tetihatu
hatutatuta (eh)
go
e
pepipapa
pepipipu


]]

--Is it all just unfunny garbage...?

--It always has been.
