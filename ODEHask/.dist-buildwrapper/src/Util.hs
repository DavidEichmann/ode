module Util where

import Linear

type Vec3 = V3 Double
type M3 = M33 Double
type Quat = Quaternion Double

unitX :: Vec3
unitX = V3 1 0 0

unitY :: Vec3
unitY = V3 0 1 0

unitZ :: Vec3
unitZ = V3 0 0 1

identity :: Quat
identity = axisAngle unitZ 0

(*-*) :: Quat -> Quat -> Quat
a *-* b = a*b
{-
(Quaternion a1 (V3 b1 c1 d1)) *-* (Quaternion a2 (V3 b2 c2 d2)) =
    Quaternion (a1*a2 - b1*b2 - c1*c2 - d1*d2) (V3
        (a1*b2 + b1*a2 + c1*d2 - d1*c2)
        (a1*c2 - b1*d2 + c1*a2 + d1*b2)
        (a1*d2 + b1*c2 - c1*b2 + d1*a2)
    )
-}

degreeToRadian :: Fractional a => a -> a
degreeToRadian d = 0.01745329251 * d

xToDirQuat :: Vec3 -> Quat
xToDirQuat (V3 x 0 0)
            | x < 0     = axisAngle unitY pi
            | otherwise = identity
xToDirQuat dir = axisAngle axis angle where
    dirU = normalize dir
    axis = normalize $ unitX `cross` dirU
    angle = acos (dirU `dot` unitX)
