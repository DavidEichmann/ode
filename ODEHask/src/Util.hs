module Util where

import Linear hiding (slerp)
import Linear.Matrix

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
    
-- average a number of rotations, this is done by repeated application of Slerp 
avgRot :: [Quat] -> Quat
avgRot [] = identity
avgRot qs = fst . head . avgRot' $ map (flip (,) 1) qs where
    avgRot' :: [(Quat,Integer)] -> [(Quat,Integer)]
    avgRot' [] = []
    avgRot' (qw:[]) = [qw]
    avgRot' (qw1:qw2:qws) = avgRot' $ (merge qw1 qw2) : (avgRot' qws)
    merge (q1,w1) (q2,w2) = (slerp q1 q2 (fromInteger w2 / fromInteger (w1+w2)), w1+w2)
    
slerp :: RealFloat a => Quaternion a -> Quaternion a -> a -> Quaternion a
slerp q p t
  | 1.0 - cosphi < 1e-8 = q
  | otherwise           = ((sin ((1-t)*phi) *^ q) + (sin (t*phi)) *^ (f p)) ^/ (sin phi)
  where
    dqp = dot q p
    (cosphi, f) = if dqp < 0 then (-dqp, negate) else (dqp, id)
    phi = acos cosphi

