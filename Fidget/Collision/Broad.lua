function doBroadPhaseCollision(rigidbodies, collisionType)

  --for now just a nested loop, maybe later it will be a grid or use bvh
if collisionType == "sphere" then
    local potentialCollisions = {}
  for i, rigidbody1 in pairs(rigidbodies) do
    
    for j = i + 1, #rigidbodies do
      local rigidbody2 = rigidbodies[j]
    if (rigidbody1.linearMovement or rigidbody2.linearMovement) and not (rigidbody1.isSleeping and rigidbody2.isSleeping) then
      local pos1, pos2 = rigidbody1.pos, rigidbody2.pos
      local radius1 = rigidbody1.boundingSphereRadius
      local radius2 = rigidbody2.boundingSphereRadius
      local distance = (pos1 - pos2):length()
      if distance < radius1 + radius2 then
        potentialCollisions[i] = potentialCollisions[i] or {}
        potentialCollisions[i][j] = true
      end
    end
  end
  end
  
    return potentialCollisions
elseif collisionType == "aabb" then
    local potentialCollisions = {}
    for i, rigidbody1 in pairs(rigidbodies) do
    for j = i + 1, #rigidbodies do

      local rigidbody2 = rigidbodies[j]
      if rigidbody1.linearMovement or rigidbody2.linearMovement and not (rigidbody1.isSleeping and rigidbody2.isSleeping) then
      local pos1, pos2 = rigidbody1.pos, rigidbody2.pos
      local max1 = pos1 + rigidbody1.boundingAABB
      local min1 = pos1 - rigidbody1.boundingAABB
      local max2 = pos2 + rigidbody2.boundingAABB
      local min2 = pos2 - rigidbody2.boundingAABB

      if max1 > min2 and max2 > min1 then
        potentialCollisions[i] = potentialCollisions[i] or {}
        potentialCollisions[i][j] = true
      end
    end
    end
  end
    return potentialCollisions
end

end
