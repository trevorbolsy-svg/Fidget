local rigidbodyMT = {}
local rigidbodies = {}
rigidbodies.allRigidbodies = {}
rigidbodyMT.__index = {}
models.Fidget.Model_Placeholders.placeholders.cube:setVisible(false)
local mtIndex = rigidbodyMT.__index

function mtIndex.remove(self)
self.model:getParent():removeChild(self.model)
self.model:remove()
local index = self.index


for i, joint in pairs(Fidget.joints.allJoints) do
  if joint.rigidbody1.index == index then
    Fidget.joints.allJoints[i] = nil
  end
  if joint.rigidbody2.index == index then
    Fidget.joints.allJoints[i] = nil
  end
end
rigidbodies.allRigidbodies[self.index] = nil
end

function mtIndex.addForce(self,force)
self.forceAccum = self.forceAccum + force
end

function mtIndex.addForceAtPoint(self, point,force)
    local newPoint = point-self.pos--transformWorldToLocal(point,PhysicsObjects[objectIndex].rotMat,PhysicsObjects[objectIndex].position)
    self.torqueAccum = self.torqueAccum+newPoint:crossed(force)
    self.forceAccum = self.forceAccum + force
end
function mtIndex.setPos(self,pos)
  self.pos = pos
end

function mtIndex.getPos(self)
  return self.pos
end
function mtIndex.setVel(self,vel)
  self.vel = vel
end

function mtIndex.getVel(self)
  return self.vel
end
function mtIndex.setRot(self,rot)
  self.rot = rot
end

function mtIndex.getRot(self)
  return self.rot
end
function mtIndex.setRotVel(self,rotVel)
  self.rotVel = rotVel
end

function mtIndex.getRotVel(self)
  return self.rotVel
end

function mtIndex.setrotVel(self,rotVel)
  self.rotVel = rotVel
end

function mtIndex.getrotVel(self)
  return self.rotVel
end

function mtIndex.setDimensions(self,dims)
  self.dimensions = dims
end

function mtIndex.getDimensions(self)
  return self.dimensions
end

function mtIndex.setGravity(self,gravity)
  self.gravity = gravity
end

function mtIndex.getGravity(self)
  return self.gravity
end

function mtIndex.setFriction(self,friction)
  self.friction = friction
end

function mtIndex.getFriction(self)
  return self.friction
end

function mtIndex.setMass(self,mass)
  self.mass = mass
end

function mtIndex.getMass(self)
  return self.mass
end

function mtIndex.setLinearMovement(self,yn)
  self.linearMovement = yn
end

function mtIndex.getLinearMovement(self)
  return self.linearMovement
end

function mtIndex.setlinearMovement(self,yn)
  self.linearMovement = yn
end

function mtIndex.getlinearMovement(self)
  return self.linearMovement
end

function mtIndex.setWorldCollision(self,yn)
  self.worldCollision = yn
end

function mtIndex.getWorldCollision(self)
  return self.worldCollision
end

function mtIndex.setworldCollision(self,yn)
  self.worldCollision = yn
end

function mtIndex.getworldCollision(self)
  return self.worldCollision
end

function mtIndex.setIsSleeping(self,yn)
  self.isSleeping = yn
end

function mtIndex.getIsSleeping(self)
  return self.isSleeping
end

function mtIndex.setisSleeping(self,yn)
  self.isSleeping = yn
end

function mtIndex.getisSleeping(self)
  return self.isSleeping
end

function mtIndex.setModelScale(self,yn)
  self.modelScale = yn
end

function mtIndex.getModelScale(self)
  return self.modelScale
end
function mtIndex.setmodelScale(self,yn)
  self.modelScale = yn
end

function mtIndex.getmodelScale(self)
  return self.modelScale
end












--[[Example:

rigidbodyParams = {

mass = 100,
gravity = vec(0,0,0),
pos = player:getPos(),
vel = vec(0,1,0),
model = models.beerBottle,
friction = 0.3
}





]]
local vec3 = vectors.vec3
local vec4 = vectors.vec4
local unpack3 = vec3().unpack
--Ill always be using these since they are faster than plain old vec() and you can omit arguments in them to get a zero vector so vec3() == vec(0,0,0) this saves time and instructions for sending arguments to the function


local rigidbodyCopyStorage = models:newPart("copyStorage", "WORLD")
local function createCopy(modelpart)
  local copy = modelpart:copy("name"):setParentType("WORLD"):setVisible(true)
  rigidbodyCopyStorage:addChild(copy)
  return copy
end

local function getPlaceholderModel(type)
  if type == "cube" then
    --why can figura just not take fucking folders into account when indexxing modelparts bruhhhhh
    return models.Fidget.Model_Placeholders.placeholders.cube
  end
end

local function getInverseInertiaTensor(params)
  if params.noRot then
    return matrices.mat3()*0
  end
  if params.type == "cube" then
    local a, b, c = unpack3(params.dimensions or vec3(1, 1, 1))
    return matrices.mat3(
      vec3((1 / 12) * params.mass * (b * b + c * c)),
      vec3(0, (1 / 12) * params.mass * (a * a + c * c)),
      vec3(0, 0, (1 / 12) * params.mass * (a * a + b * b))
    ):inverted()
  elseif params.type == "sphere" then
    local r = params.radius or 1
    local inertia = (2 / 5) * params.mass * r * r
    return matrices.mat3(
      vec3(inertia, 0, 0),
      vec3(0, inertia, 0),
      vec3(0, 0, inertia)
    ):inverted()
  end
end

local function getBoundingSphere(type,dimensions)
if type == "cube" then
    return (dimensions or vec3(1,1,1)):length()/2
end
end



function rigidbodies.createRigidbody(params)
  if type(params.pos) ~= "Vector3"  then
    error("§4§nRigidbody Creation: Position(vec3) expected, got: "..tostring(params.pos).."§r")
  end
  if not params.type then 
    params.type = "cube" --default type is cube
  end
  if not params.modelScale then
    params.modelScale = vec3()+1 --default model scale is 1,1,1
  end
  if not params.dimensions then
    params.dimensions = vec3()+1 --default dimensions for cube is 1,1,1
  end
  if params.linearMovement == nil then
    params.linearMovement = true --by default rigidbodies can move linearly
  end
  if not params.mass then
    params.mass = 1
  end
  if params.mass <= 0 then
    error("§4§nRigidbody Creation: Mass must be a positive number, got: "..tostring(params.mass).."§r")
  end
  local rigidbody = {

    --general attributes
    invMass = (1 / params.mass),
    invInertiaTensorLOCAL = getInverseInertiaTensor(params),
    invInertiaTensorWORLD = getInverseInertiaTensor(params),
    gravity = params.gravity or vec3(0,-100,0),
    friction = params.friction or 0.5,
    restitution = params.restitution or 0.5,
    rotMat = matrices.mat3(), 

    --rigidbody movement
    pos = params.pos,
    vel = params.vel or vec3(),
    rot = params.rot or vec4(0,0,-1,0),
    rotVel = params.rotVel or vec3(),
    forceAccum = vec3(),
    torqueAccum = vec3(),
    linearMovement = params.linearMovement,

    --render stuff
    model = createCopy(params.model or getPlaceholderModel(params.type)), --set the model to the one given by user or to a placeholder
    prevPos = params.pos or vec3(),
    prevRot = params.rot or vec4(0,0,-1,0),
    modelScale = params.modelScale or vec3(1,1,1),

    --stuff specific to certain rigidbodies
    --**cuboid**--
    dimensions = params.dimensions,
    halfDimensions = params.dimensions:copy()*0.5,
    --**sphere**--
    --radius = params.radius or 1,

    --**mesh**--
    --mesh = params.mesh
    --meshTriangles =


    --engine stuff
    boundingSphereRadius = getBoundingSphere(params.type or "cube",params.dimensions,params.radius,params.mesh),
    boundingAABB = vec(0.5,0.5,0.5),
    vertices = {},
    type = params.type,
    cache = {}, --for storing impulses cause it kinda converges really badly without this apparently
    index = #rigidbodies.allRigidbodies+1,
    worldCollision = params.worldCollision or true,--only supported with aabb broadphase
    isSleeping = params.isSleeping or false,
    sleepTimer = -1,
  }
  setmetatable(rigidbody,rigidbodyMT)
  table.insert(rigidbodies.allRigidbodies, rigidbody)
  return rigidbody
end

return rigidbodies


