--creating modelpart for storing sprites and the texture
local spriteStorage = models:newPart("copyStorage", "WORLD")
local newSprite = spriteStorage.newSprite
figuraMetatables.ModelPart.__add = newSprite
local spriteStorage = models:newPart("copyStorage", "WORLD")

local lineText = textures:newTexture("white", 1, 1):setPixel(0,0,1,1,1)
local lines = {}
--optimizing functions
local rng = math.random
local vec3 = vectors.vec3

local funcSprite = newSprite(spriteStorage,rng())
local setTexture = funcSprite.setTexture
funcSprite = setTexture(funcSprite,lineText)
local setRenderType = funcSprite.setRenderType
local sremove = funcSprite.remove
local setColor = funcSprite.setColor
local getVerts = funcSprite.getVertices
local funcVert = funcSprite:getVertices()[1]
local vsetPos = funcVert.setPos
--creating the transformation vector for transforming world coord into vertex coords which flip x anf y for whatever reason
local vertexTransformVector = vec3(-1,-1,1)*16
local nlines = 0
--optimize spritetask functions for instructions :cryingemoji:
figuraMetatables.SpriteTask.__add = setRenderType
figuraMetatables.SpriteTask.__mul = setTexture
figuraMetatables.SpriteTask.__sub = setColor
figuraMetatables.SpriteTask.__unm = getVerts

figuraMetatables.Vertex.__add = vsetPos

--- Creates a line
--- @param pos1 Vec3 Start of the line
--- @param pos2 Vec3 End of the line
--- @param color Vec3|Vec4 Color of the line
--- @return spritetask sprite 
--- @return number index 

function createLine(pos1,pos2,color)
--calculating the position in vertex space
  pos1 = pos1*vertexTransformVector
  pos2 = pos2*vertexTransformVector
  nlines = nlines + 1
--creating the sprite
  local sprite = (spriteStorage+(""..nlines))*lineText+"LINES"-color
--getting the vertices
  local verts = -sprite
--setting vertex positions so the spritetask creates a line
  local _ = verts[1]+pos1,verts[4]+pos2,verts[2]+pos2,verts[3]+pos1
return sprite
end

local off1 = vec3(1,0,0)*1
local off2 = vec(0,-1,0)*1
function createLineEx(pos1,pos2,color)
--calculating the position in vertex space
  pos1 = pos1*vertexTransformVector
  pos2 = pos2*vertexTransformVector
  nlines = nlines + 1
--creating the sprite
  local sprite = (spriteStorage+(""..nlines))*lineText+"LINES"-color
--getting the vertices
  local verts = -sprite
--setting vertex positions so the spritetask creates a line
  local _ = verts[1]+(pos1+off1),verts[4]+(pos2-off1),verts[2]+(pos2+off1),verts[3]+(pos1-off1)
    nlines = nlines + 1
--creating the sprite
  local sprite = (spriteStorage+(""..nlines))*lineText+"LINES"-color
--getting the vertices
  local verts = -sprite
--setting vertex positions so the spritetask creates a line
  local _ = verts[1]+(pos1+off2),verts[4]+(pos2-off2),verts[2]+(pos2+off2),verts[3]+(pos1-off2)
return sprite
end
--- Removes the line without removing it from the lines table 
--- @param line spritetask
function removeLine(line)
spriteStorage:removeTask(line)
end
--- Removes all lines
function removeAllLines()
spriteStorage:removeTask()
end


