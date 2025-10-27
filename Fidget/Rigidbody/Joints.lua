local jointMT = {}
local joints = {}
joints.allJoints = {}
jointMT.__index = {}

local mtIndex = jointMT.__index

function mtIndex.remove(self)
joints.allJoints[self.index] = nil
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
function mtIndex.setPos1(self,pos)
  self.pos1 = pos
end
function mtIndex.setPos2(self,pos)
  self.pos2 = pos
end
function mtIndex.getPos1(self)
  return self.pos1
end
function mtIndex.getPos2(self)
  return self.pos2
end
function mtIndex.setDistance(self,pos)
  self.distance = pos
end
function mtIndex.getDistance(self)
  return self.distance
end

function joints.createJoint(params)

  if not params.rigidbody1 or not params.rigidbody2 or not params.rigidbody1.index or not params.rigidbody2.index then
    error("§4§nJoints Creation: Both rigidbodies must be valid!")
  end
  if params.rigidbody1.index == params.rigidbody2.index then
    error("§4§nJoints Creation: Both rigidbodies must be different!")
  end
  if not params.type then 
    params.type = "distance" --default type is distance
  end
  local joint = {
    --joint attributes
    pos1 = params.pos1 or vec3(), -- in local space
    pos2 = params.pos2 or vec3(),
    type = params.type, --distance, ball, hinge
    rigidbody1 = params.rigidbody1,
    rigidbody2 = params.rigidbody2,
    distance = params.distance or 0,
    axis1 = params.axis1 or vec3(1),
    axis2 = params.axis2 or vec3(1),
    index = #joints + 1
  }
  setmetatable(joint,jointMT)
  table.insert(joints.allJoints, joint)
  return joint
end

return joints
