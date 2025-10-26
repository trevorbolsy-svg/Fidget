



local math_abs = math.abs
local vec3 = vectors.vec3
local mat3 = matrices.mat3
local v1 = vec3(1)
local v2 = vec3(0, 1)
local v3 = vec3(0, 0, 1)
local dot3 = vec3().dot
local lnext = next
local unpack = vec3().unpack
local normalize = vec3().normalize
local crossed3 = vec3().crossed
local transposed = mat3().transposed
local err = 0.000001
local length = vec3().length


--https://gamma.cs.unc.edu/users/gottschalk/main.pdf

--This code is so separated but I still no nocturnal emissions(there is a pretty good song with this name in the ror2 ost) :(
--WHY IS THIS FUCKING VSCODE PLACING EXTRA BRACKETS IN COMMENTS BRUH
function doFineCollision(rigidbody1, rigidbody2,separatingAxis)
  --local collisionType = 1 --face-face => 1 , edge-edge => 2 this is important later cause we gotta clip one face if its face face and find the closest points on the edges if edge edge
  local delta = rigidbody2.pos - rigidbody1.pos

  --[[
  local Rb = transposed(rtm1)*rtm2

  --absing the matrix. Apply func exists for vectors but not matrices? did you know that? I didnt.
  --But apply func is slow as fuck anyways so thats trash.
  --It just has so much overhead compared to just doing this
  --You first call it, with a function as an argument(not good for performance because function call) and then it calls the function given 3 TIMES giving it 2 arguments. SO YOU NEED TO HAVE 4 FUCKING FUNCTION CALLS
  --if it existed for matrices it would be nine(and they would literally do what im doing here) so it would literally just add 10 functions + 2 args for them all of overhead so pointless
  
  local _1 = Rb[1]
  local _2 = Rb[2] 
  local _3 = Rb[3]
  if _1.x < 0 then
    _1.x = -_1.x
  end
  if _1.y < 0 then
    _1.y = -_1.y
  end
  if _1.z < 0 then
    _1.z = -_1.z
  end
  if _2.x < 0 then
    _2.x = -_2.x
  end
  if _2.y < 0 then
    _2.y = -_2.y
  end
  if _2.z < 0 then
    _2.z = -_2.z
  end
  if _3.x < 0 then
    _3.x = -_3.x 
  end
  if _3.y < 0 then
    _3.y = -_3.y
  end
  if _3.z < 0 then
    _3.z = -_3.z
  end
  RbAbs = mat3(_1,_2,_3)

  local Tb = transposed(rtm1)*delta

]]
  --I ve got no one to talk to and no friends this project makes me want to die
  --Every day i go to sleep my mind reminds me of how big of a failure i am
  --The worst part. I know all my problems
  --I do not solve them. why? why? WHY?
  --I want to complete this. This is my dream. but its killing me from the inside. Im disappointed. Im sorry

  


  -- uhhh i couldnt find a better way to do this without 3 separeta(which would be shit) tables so I just do this
  --axis encoded like this
  --1-3 >> axis1
  --4-6 >> axis2
  --7-15>> edges --> banished to the shadow realm(edges are genareted with face face colission)

    local l = 7
  for i = 1, 3 do
    for j = 4, 6 do
      separatingAxis[l] = normalize(crossed3(separatingAxis[i],separatingAxis[j]))
      l = l + 1
    end
  end

  local minIndex = 0
  local minimumTranslationVector
  local minimumTranslationDistance = 694202137000--haha funny number
  --ok assuming you do not know why this is a funny number let me break it down for you
  --<<69>>4202137000 is a famous posi... This shit is gonna get posted to the discord bruh search it up yourself
  --69<<420>>2137000 is the hour at which weed is smoked???? or maybe when hitler died or rather was born????? I do not fuckin know
  --69420<<2137>>000 is when the polish pope died(Only poles can appreciate this)
  --694202137<<000>> is just 0's so it doesnt break at large distances
  --uhh now that i explained this is not funny no more
  --ok another funny number 66611124441
  --this is the number of kg your mom wei... 
  local rdims1 = rigidbody1.halfDimensions
  local rdims2 = rigidbody2.halfDimensions
  local rdims1x,rdims1y,rdims1z = unpack(rdims1)
  local rdims2x,rdims2y,rdims2z = unpack(rdims2)
  --[[
  local RbAbs1, RbAbs2, RbAbs3 = RbAbs[1], RbAbs[2] ,RbAbs[3]]
  for i, axis in lnext,separatingAxis do -- not many people know about this way to write for loops
    if length(axis) == 0 then
      goto leEdgeIsSoSmallIts___Its___UHHHH_WHATISIT__ItsLiterallyAllZeros
    end
    local s,r1,r2
    if i <= 3 then
      --[[
      s = Tb[i]
      r1 = rdims1[i]
      r2 = dot3(rdims2,RbAbs[i])]]
      s = dot3(delta,axis)
      r1 = rdims1[i]
      r2 = (rdims2x*math_abs(dot3(separatingAxis[4],axis))+rdims2y*math_abs(dot3(separatingAxis[5],axis))+rdims2z*math_abs(dot3(separatingAxis[6],axis)))
    elseif i <= 6 then
      s = dot3(delta,axis)
      r1 = (rdims1x*math_abs(dot3(separatingAxis[1],axis))+rdims1y*math_abs(dot3(separatingAxis[2],axis))+rdims1z*math_abs(dot3(separatingAxis[3],axis)))
      r2 = rdims2[i-3]
    else
      s = dot3(delta,axis)
      r1 = (rdims1x*math_abs(dot3(separatingAxis[1],axis))+rdims1y*math_abs(dot3(separatingAxis[2],axis))+rdims1z*math_abs(dot3(separatingAxis[3],axis)))
      r2 = (rdims2x*math_abs(dot3(separatingAxis[4],axis))+rdims2y*math_abs(dot3(separatingAxis[5],axis))+rdims2z*math_abs(dot3(separatingAxis[6],axis)))
    end

  local penetration = (r1+r2)  - (s < 0 and -s or s)
  --log(r2,r1,s,i)
  if penetration <= 0 then 
    return
  end

  if penetration < minimumTranslationDistance then
    minIndex = i
    minimumTranslationVector = (separatingAxis[minIndex]*(s < 0 and -1 or 1))
    minimumTranslationDistance = penetration
  end

    --[[
        local minProj1 = 694202137
    local maxProj1 = -694202137--the negative funny number??? does it mean its so funny it overflowed and is now negative or is it just super unfunny
    --philosophical questions worth asking!
    local minProj2 = 694202137
    local maxProj2 = -694202137
    --this shit is quadratic time or linear time???<<<<its quadratic(?) you retard
    --NO FUCKING IDEA
    --I DO NOT WISH TO THINK
    --BUT I WISH FOR WAFFLES(not blue waffles)(do not search that up)
    for i, vertex in lnext,rigidbody1Verts do
      local projection = dot3(vertex,axis)
      if minProj1 > projection then
        minProj1 = projection
      elseif maxProj1 < projection then
        maxProj1 = projection
      end
    end
    for i, vertex in lnext,rigidbody2Verts do
      local projection = dot3(vertex,axis)
      if minProj2 > projection then
        minProj2 = projection
      elseif maxProj2 < projection then
        maxProj2 = projection
      end
    end

    if maxProj1 < minProj2 or maxProj2 < minProj1 then
      return nil -- no collision
    end

    local overlap 
    local a,b = maxProj1 - minProj2,maxProj2 - minProj1
    if a < b then
      overlap = a
    else
      overlap = b
    end

    --They say size doesnt matter but when you have a negative --penetration-- overlap something is wrong bro
    if overlap > 0 and overlap < minimumTranslationDistance then
      minimumTranslationDistance = overlap
      minIndex = i
      minimumTranslationVector = axis * (minimumTranslationDistance)
    end]]
    ::leEdgeIsSoSmallIts___Its___UHHHH_WHATISIT__ItsLiterallyAllZeros::
  end
  --collisionType = (minIndex <= 6) and 1 or 2 --very noice. Cause i did the indexxing like this this is super easy to implement and super clean too. love it --> erin catto to the rescue(this is not used)
  --I cant wait to impelemnent the suther-badgerman (whatever it was named) polygon plane clipping algorithm. so beautiful
  --hmm i dont thin anyones gonna read ts



  --Garbage ahhh shit turns out you do in fact need to check all 15 axis. fuck me
  local axis = separatingAxis[minIndex]
  local maxMTV, maxIndex = dot3(axis,(separatingAxis[1])), 1
  if minIndex > 6 then
    for i = 2, 6 do
      local a = dot3(axis,(separatingAxis[i]))
      local b = a
      if a < 0 then
        a = -a
      end
      if maxMTV < 0 then
        maxMTV = - maxMTV
      end
      if maxMTV < a then
        maxMTV = b
        maxIndex = i
      end
    end
    minIndex = maxIndex
    --this is bs i this is not correct but hopefully it worksTM    
    minimumTranslationVector = separatingAxis[minIndex] * (minimumTranslationDistance)
  end
  return minimumTranslationVector, minIndex
end


--I Made this very readable with the novel I wrote in between every line of code
--A couple days later here. Yup old soomuchlag you were right. the comments are very readable. and they are cool to read.




