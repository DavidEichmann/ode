module Motion.Util where

import Linear
import Util
import Motion.Joint

getCapsulMassInertiaM :: Double -> Double -> Vec3 -> (Double,M3)
getCapsulMassInertiaM d r boneVec = --(mass, rotM !*! xm !*! rotInvM) where
    (mass, rotM !*! xm !*! rotInvM) where
        (mass, xm) = getXCapsulMassInertiaM d r (norm boneVec)
        rot     = xToDirQuat boneVec
        rotInv  = conjugate rot
        rotM    = fromQuaternion rot
        rotInvM = fromQuaternion rotInv

getXCapsulMassInertiaM :: Double -> Double -> Double -> (Double,M3)
getXCapsulMassInertiaM d r l = (m, im) where
    r2 = r ** 2
    l2 = l ** 2
    m1 = pi * l * r2 * d
    m = m1 {- +m2

    For now just use a cylinder
    m2 = (4/3) * pi * r2 * r * d
    ia = m1 * (0.25 * r2 + (1/12) * l2) +
         m2 * (0.4  * r2 + 0.375  * r * l + 0.25 * l2)
    ib = ((m1 * 0.5) + (m2 * 0.4)) * r2;

    im = V3 (V3  ib  0   0)
            (V3  0   ia  0)
            (V3  0   0   ia)
            -}

    iX  = 0.5 * m1 * r2
    iZY = (1/12) * m1 * ((3 * r2) + (l2))
    im = V3 (V3  iX  0   0)
            (V3  0   iZY  0)
            (V3  0   0   iZY)


toAngularVel :: Quat -> Quat -> Double -> Vec3
toAngularVel qi qf dt = if angleHalf == 0 then zero else ((2 * angleHalf) *^ rotAxis) ^/ dt where
                        Quaternion qlnW qlnV = qf * (conjugate qi)
                        angleHalf = acos $ max (-1) (min qlnW 1)
                        rotAxis = qlnV ^/ (sin angleHalf)


blendFrames :: (Int -> Joint) -> (Int -> Joint) -> Double -> (Int -> Joint)
blendFrames j1 j2 t i = j1i{
        offset = ((offset j1i) ^* (1-t)) + ((offset j2i) ^* t),
        rotationL = Util.slerp (rotationL j1i) (rotationL j2i) t
    } where
        j1i = j1 i
        j2i = j2 i
        
