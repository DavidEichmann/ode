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
ankleIK md@MotionDataVars{_xj=xj,_pj=pj,_bj=bj,_fN=fN,_fs=fs,_j=j,_rj=rj,_bByName=bByName,_footBs=((lfbI,_),(rfbI,_))} aTs =
--    trace "Done correcting feet with IK"
     (modifyMotionDataVars md (concat [correctFrame fI | fI <- fs])) where

        -- returns the updates to apply to a given frame
        correctFrame :: FrameIx -> [((Int,Int), Joint)]
        correctFrame fI@(F fi) = (correctLeg lajI) ++ (correctLeg rajI) where

            correctLeg ajI@(J aji) = [
                     ((fi, hji), jF fi hji) -- jointH{ rotationL = (rotationL jointH) * dRotHL})
                    ,((fi, kji), jF fi kji) -- jointK{ rotationL = (rotationL jointK) * dRotKL})
                    ,((fi, aji), jF fi aji) -- jointA{ rotationL = (rotationL jointA) * dRotAL})
                ] where

                    kjI@(J kji) = pj ajI
                    hjI@(J hji) = pj kjI

                    -- joints

                    jointH = j fi hji
                    jointK = j fi kji
                    jointA = j fi aji


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

                    md2@MotionDataVars{_j=j2,_xj=xj2,_rj=rj2} = modifyMotionDataVars md [
                            ((fi,hji), jointH{rotationL = (rotationL jointH) * (axisAngle ((conjugate $ rj fI hjI) `rotate` axisH) angleDH2)})
                        ]
                    jointH2 = j2 fi hji
                    jointK2 = j2 fi kji

                    -- step 2   adjust knee/hip (rota aroun knee asxis) to be reach the correct distance (hipt to ankel)

                    -- some vectors
                    haT  = aT  - (xj2 fI hjI)
                    hatNorm = norm haT
                    ha  = (xj2 fI ajI)  - (xj2 fI hjI)
                    kh  = (xj2 fI hjI)  - (xj2 fI kjI)
                    hk  = (xj2 fI kjI)  - (xj2 fI hjI)
                    ka  = (xj2 fI ajI)  - (xj2 fI kjI)

                    -- original angles
                    angleK = acosc $ (kh `dot` ka) / ((norm kh) * (norm ka))
                    angleH = acosc $ (ha `dot` hk) / ((norm ha) * (norm hk))
                    -- new angles
                    angleKT = acosc $ ((hatNorm ** 2) - (norm2 hk) - (norm2 ka)) / (-2 * (norm hk) * (norm ka))
                    angleHT = asinc $ ((norm ka) * (sin angleKT)) / (hatNorm)

                    -- 2.1  extend
                    axisKExtend = normalize $ kh `cross` ka
                    md22@MotionDataVars{_j=j22,_xj=xj22,_rj=rj22} = modifyMotionDataVars md2 [
                            ((fi,hji), jointH2{rotationL = (rotationL jointH2) * (axisAngle ((conjugate $ rj2 fI hjI) `rotate` axisKExtend) (negate angleH))}),
                            ((fi,kji), jointK2{rotationL = (rotationL jointK2) * (axisAngle ((conjugate $ rj2 fI kjI) `rotate` axisKExtend) (pi - angleK))})
                        ]
                    jointH22 = j22 fi hji
                    jointK22 = j22 fi kji
                    jointA22 = j22 fi aji

                    -- 2.2 retract and correct foot
                    axisKRetract = (rj22 fI hjI) `rotate` (negate unitX)

                    md3@MotionDataVars{_rj=rj3} = modifyMotionDataVars md22 [
                             ((fi,hji), jointH22{rotationL = (rotationL jointH22) * (axisAngle ((conjugate $ rj22 fI hjI) `rotate` axisKRetract) angleHT)})
                            ,((fi,kji), jointK22{rotationL = (rotationL jointK22) * (axisAngle ((conjugate $ rj22 fI kjI) `rotate` axisKRetract) (angleKT - pi))})
                        ]

                    -- 3 correct foot
                    mdF@MotionDataVars{_j=jF} = modifyMotionDataVars md3 [
                            -- target global ankel rotation is (rj fI ajI) we want this equal to the current global rotation (rj3 fI ajI = rj3 fI kjI * rotaionL ajI) hence
                            --      rj3 fI kjI * rotaionL ajI = rj fI ajI
                            --                    rotaionL ajI = (conjugate $ rj3 fI kjI) * (rj fI ajI)
                            ((fi,aji), jointA22{rotationL = (conjugate $ rj3 fI kjI) * (rj fI ajI)})
                        ]


        lajI = pj . bj $ lfbI   -- left ankel joint
        lkjI = pj lajI          -- left knee joint
        lhjI = pj lkjI          -- left hip joint
        rajI = pj . bj $ rfbI   -- right ankel joint
        rkjI = pj rajI          -- right knee joint
        rhjI = pj rkjI          -- right hip joint
