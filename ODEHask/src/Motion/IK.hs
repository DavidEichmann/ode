module Motion.IK where

import Data.Array
import Linear hiding (_j)

import FFI
import Util
import Constants
import Motion.Joint
import Motion.MotionDataVars


-- Move (via IK) feet in the second MDV argument to the position of the feet in the first
-- update the motion data. Each frame is updated independantly (individual updates are collected from "correctFrame")
feetIK :: MotionDataVars -> MotionDataVars -> MotionDataVars
feetIK target@MotionDataVars{_xj=xjT,_fN=fN,_pj=pj,_bj=bj,_footBs=((lfbI,_),(rfbI,_))} md =

    ankleIK md aTs where

    lajI = pj . bj $ lfbI   -- left ankel joint
    rajI = pj . bj $ rfbI   -- right ankel joint

    aTs = buildArray (0,fN-1) aT

    -- target ankel position
    aT fi = let fI = F fi in (xjT fI lajI, xjT fI rajI)

-- Move (via IK) the ankels (keeping the global orientation of the feet) to the target positions
ankleIK :: MotionDataVars -> Array Int (Vec3,Vec3) -> MotionDataVars
ankleIK md@MotionDataVars{_jBase=jBase,_xj=xj,_pj=pj,_bj=bj,_fN=fN,_fs=fs,_j=j,_rj=rj,_bByName=bByName,_footBs=((lfbI,_),(rfbI,_))} aTs =
--    trace "Done correcting feet with IK"
     (modifyMotionDataVars md (concat [correctFrame fI | fI <- fs])) where

        -- returns the updates to apply to a given frame
        correctFrame :: FrameIx -> [((Int,Int), Joint)]
        correctFrame fI@(F fi) = (correctLeg lajI) ++ (correctLeg rajI) where

            correctLeg ajI@(J aji) = [
                     ((fi, hji), jointH{ rotationL = rotationLHF})
                    ,((fi, kji), jointK{ rotationL = rotationLKF})
                    ,((fi, aji), jointA{ rotationL = rotationLAF})
                ] where
                    

                    kjI@(J kji) = pj ajI
                    hjI@(J hji) = pj kjI

                    -- joints

                    jointH = j fi hji
                    jointK = j fi kji
                    jointA = j fi aji
                
                    -- Helper functions (efficient update of rotation and joint functions)
                    -- only works for hip joint and decendants
                    
                    jMod :: Quat -> Quat -> ((JointIx -> Quat), (JointIx -> Vec3))
                    jMod hipRotL kneeRotL = (rjMod, xjMod) where
                        rotRootOrig = rj fI (pj hjI)
                        xjMod jI
                            | jI == hjI     = xj fI hjI
                            | otherwise     = (xjMod $ pj jI) + ((rjMod $ pj jI) `rotate` (offset $ jBase jI))
                            
                        rjMod jI@(J ji)
                            | jI == hjI     = rotRootOrig * hipRotL
                            | otherwise     = (rjMod (pj jI)) * rotL
                            where
                                rotL
                                    | jI == kjI     = kneeRotL
                                    | otherwise     = rotationL $ j fi ji


                    -- clamped asin/acos

                    asinc v = asin $ max (-1) (min v 1)
                    acosc v = acos $ max (-1) (min v 1)

                    -- target ankel position (normalized for max extention)

                    aT = (if ajI == lajI then fst else snd) (aTs ! fi)

                    -- setp 1   Rotate hip to point ankel toward target ankel

                    ha1  = (xj fI ajI)  - (xj fI hjI)
                    hat1 = aT - (xj fI hjI)

                    axisH = normalize $ ha1 `cross` hat1
                    angleDH2 = acosc $ (ha1 `dot` hat1) / ((norm ha1) * (norm hat1))

                    rotationLH2 = (rotationL jointH) * (axisAngle ((conjugate $ rj fI hjI) `rotate` axisH) angleDH2)
                    rotationLK2 = rotationL jointK
                    (rj2,xj2) = jMod rotationLH2 rotationLK2

                    -- step 2   Extend

                    -- some vectors
                    haT  = aT  - (xj2 hjI)
                    hatNorm = norm haT
                    ha  = (xj2 ajI)  - (xj2 hjI)
                    kh  = (xj2 hjI)  - (xj2 kjI)
                    hk  = (xj2 kjI)  - (xj2 hjI)
                    ka  = (xj2 ajI)  - (xj2 kjI)

                    -- angles
                    angleK = acosc $ (kh `dot` ka) / ((norm kh) * (norm ka))
                    angleH = acosc $ (ha `dot` hk) / ((norm ha) * (norm hk))

                    -- extend
                    axisKExtend = normalize $ kh `cross` ka
                    rotationLH22 = (rotationLH2 * (axisAngle ((conjugate $ rj2 hjI) `rotate` axisKExtend) (negate angleH)))
                    rotationLK22 = (rotationLK2 * (axisAngle ((conjugate $ rj2 kjI) `rotate` axisKExtend) (pi - angleK)))
                    (rj22,xj22) = jMod rotationLH22 rotationLK22

                    -- step 3 retract
                    
                    axisKRetract = (rj22 hjI) `rotate` (negate unitX)
                    -- new angles
                    angleKT = acosc $ ((hatNorm ** 2) - (norm2 hk) - (norm2 ka)) / (-2 * (norm hk) * (norm ka))
                    angleHT = asinc $ ((norm ka) * (sin angleKT)) / (hatNorm)
                    
                    rotationLHF = rotationLH22 * (axisAngle ((conjugate $ rj22 hjI) `rotate` axisKRetract) angleHT)
                    rotationLKF = rotationLK22 * (axisAngle ((conjugate $ rj22 kjI) `rotate` axisKRetract) (angleKT - pi))
                    (rj3,_) = jMod rotationLHF rotationLKF
                    
                    -- step 4 correct ankle
                    
                    rotationLAF = (conjugate $ rj3 kjI) * (rj fI ajI)


        lajI = pj . bj $ lfbI   -- left ankel joint
        lkjI = pj lajI          -- left knee joint
        lhjI = pj lkjI          -- left hip joint
        rajI = pj . bj $ rfbI   -- right ankel joint
        rkjI = pj rajI          -- right knee joint
        rhjI = pj rkjI          -- right hip joint
