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
identity = axisAngle (V3 0 0 1) 0

degreeToRadian :: Fractional a => a -> a
degreeToRadian d = 0.01745329251 * d