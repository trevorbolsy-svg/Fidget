--MADE BY 4P5
--!!!!!!!!!!!!
--!!!!!!!!!!!!
--!!!!!!!!!!!!
--!!!!!!!!!!!!
--!!!!!!!!!!!!
local vec3 = vectors.vec3
---@param blocks table<integer, table<integer, table<integer, string>>>
---@param like? fun(a: string, b: string): boolean
---@return {x: integer, y: integer, z: integer, width: integer, height: integer, depth: integer, id: string}[]
return function (blocks, like)
    local meshes = {}
    local visited = {}

    local function ensureVisited(x, y, z)
        visited[x] = visited[x] or {}
        visited[x][y] = visited[x][y] or {}
        visited[x][y][z] = true
    end

    local function isVisited(x, y, z)
        return visited[x] and visited[x][y] and visited[x][y][z]
    end

    local function canMerge(x, y, z, id)
        local block = blocks[x] and blocks[x][y] and blocks[x][y][z]
        if not block then return false end
        return not isVisited(x, y, z) and (block == id)
    end

    local function expand(x, y, z, id)
        local width, height, depth = 1, 1, 1

        while canMerge(x + width, y, z, id) do
            local match = true
            for i = 0, height - 1 do
                for j = 0, depth - 1 do
                    if not canMerge(x + width, y + i, z + j, id) then
                        match = false
                        break
                    end
                end
                if not match then break end
            end
            if not match then break end
            width = width + 1
        end

        while true do
            local match = true
            for i = 0, width - 1 do
                for j = 0, depth - 1 do
                    if not canMerge(x + i, y + height, z + j, id) then
                        match = false
                        break
                    end
                end
                if not match then break end
            end
            if not match then break end
            height = height + 1
        end

        while true do
            local match = true
            for i = 0, width - 1 do
                for j = 0, height - 1 do
                    if not canMerge(x + i, y + j, z + depth, id) then
                        match = false
                        break
                    end
                end
                if not match then break end
            end
            if not match then break end
            depth = depth + 1
        end

        for i = x, x + width - 1 do
            for j = y, y + height - 1 do
                for k = z, z + depth - 1 do
                    ensureVisited(i, j, k)
                end
            end
        end

        return {pos = vec3(x,y,z), dims = vec3(width,height,depth)*0.5, id = id }
    end

    for x, yz_tbl in pairs(blocks) do
        for y, z_tbl in pairs(yz_tbl) do
            for z, id in pairs(z_tbl) do
                if not isVisited(x, y, z) then
                    meshes[#meshes + 1] = expand(x, y, z, id)
                end
            end
        end
    end

    return meshes
end