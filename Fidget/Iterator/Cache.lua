--na ten plik zmieniam język na polski bo chcę tutaj zademonstrować swój brak kultury. 
local cache = {}
local cacheFuncs = {}
local Fidget = require("Fidget.FidgetSetup")--achh czy to nie za dużo instrukcji nie dodaje????
local function addToCache(index1,index2,edges,normal,impulse)--cache? kasz? kasza? kaszel? kurwa, co to ma być

    if not edges[2] then
        edges[2] = 0
    end

    cache[index1..index2..","..edges[1]..","..edges[2]..","..normal] = {impulse,Fidget.physicsSim.step} --chujowy indeks szczerze mówiąc. chciałbym stworzyć coś lepszego ale nie za bardzo jest możliwość
end
cacheFuncs.add = addToCache--leppiej by było zrobić function cacheFuncs.aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
local function retrieveFromCache(index1,index2,edges,normal)
    if not edges[2] then
        edges[2] = 0
    end

    local cacheIndex = index1..index2..","..edges[1]..","..edges[2]..","..normal
    if cache[cacheIndex] and cache[cacheIndex][2] - Fidget.physicsSim.step >= -8 then
        return cache[cacheIndex][1]
    else
        return 0
    end
end
cacheFuncs.retrieve = retrieveFromCache

return cacheFuncs
