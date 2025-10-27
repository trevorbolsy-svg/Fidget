local Fidget = require("Fidget.FidgetSetup")
local quaternions = require("Fidget.quaternions")
local vec3 = vectors.vec3
local vec4 = vectors.vec4
local math_lerp = math.lerp
local normalize = vec4().normalize
local augmented = matrices.mat3().augmented
local translate = matrices.mat4().translate
function events.render(delta)
  --for performance reasons
  for i, rigidbody in pairs(Fidget.rigidbodies.allRigidbodies) do
    local pos = math.lerp(rigidbody.prevPos, rigidbody.pos, delta) * 16
    local scale = matrices.mat3(vec3(rigidbody.modelScale.x),vec3(0,rigidbody.modelScale.y),vec(0,0,rigidbody.modelScale.z)) 
    local THEONEANDONLYPLEASEWELCOMETHEROTATIONMATRIXPRETENDSOMECOOLASSMUSICPLAYSHERE = augmented((quaternions.toRotationMatrix3(normalize(math_lerp(rigidbody.prevRot, rigidbody.rot, delta)))*(scale)))
    local mat = translate(THEONEANDONLYPLEASEWELCOMETHEROTATIONMATRIXPRETENDSOMECOOLASSMUSICPLAYSHERE,pos)
    rigidbody.model:setMatrix(mat):setLight(15)
  end
end
