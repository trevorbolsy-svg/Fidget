local quaternions = {}
--Wait your quaternions were always vec4's?
--They always have been
local vec4 = vectors.vec4
local vec3 = vectors.vec3
local unpack4 = vec4().unpack
--i was to lazy to rewrite this from the formula and let github copilot write this for me
--hopefully that doesnt bite me in the ass later lmao
function quaternions.multiply(q1, q2)
    local q1w, q1x, q1y, q1z = unpack4(q1)
    local q2w, q2x, q2y, q2z = unpack4(q2) --ya cannot stack unpacks which is bad because I WANTED TO STACK THE FUKIN ANPAKS
    --it apparently be faster when no vector indexxing involved :)))))
    return vec4(
        q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z,
        q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y,
        q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x,
        q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w
    )
end

function quaternions.toRotationMatrix3(q)
    local w, x, y, z = unpack4(q)

    local xx, yy, zz = x*x, y*y, z*z
    local xy, xz, yz = x*y, x*z, y*z
    local wx, wy, wz = w*x, w*y, w*z

    return matrices.mat3(
        vec3(1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)),
        vec3(2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)),
        vec3(2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy))
    )
end


return quaternions