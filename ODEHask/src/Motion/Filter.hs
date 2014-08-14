module Motion.Filter where

import Data.List
import Control.Arrow
import Data.Array
import Linear hiding (_i,_j,_k,_w,trace)
import Debug.Trace

import FFI
import Util
import Constants
import Motion.Util
import Motion.Joint
import Motion.MotionDataVars
import Motion.IK


-- addapted from "Online Generation of Humanoid Walking Motion based on a Fast Generation Method of Motion Pattern that Follows Desired ZMP"
-- attempts to move zmp of each frame such that it is in the support polygon
fitMottionDataToSP :: MotionDataVars -> (Array Int (Double, Double),MotionDataVars)
fitMottionDataToSP mdvOrig@MotionDataVars{_fN=fN,_g=g,_dt=dt,_bs=bs,_fs=fs,_zmp=zmpO,_sp=spO} = newMDV where

    newMDV = calculateOffsetMap mdvOrig (listArray (0,fN-1) (map (\_ -> (0,0)) fs)) 0

    -- take:    current mdv, current offset map, current frame index
    -- return:  new motion data vars
    calculateOffsetMap :: MotionDataVars -> Array Int (Double,Double) -> Int -> (Array Int (Double, Double),MotionDataVars)
    calculateOffsetMap (mdv@MotionDataVars{_zmpIsInSp=zmpIsInSp,_j=j,_m=m,_xb=xb,_l'=l',_com=com,_zmp=zmp,_sp=sp}) e fi
        -- end of Motion data.. then we are finished
        | fi > (snd $ bounds e)     = (e,mdv)
        -- ZMP already in the SP, move to the next frame
        | zmpIsInSp (F fi)          = calculateOffsetMap mdv e (fi + 1)
        -- ZMP outside of the SP, modify the frame and move to the next frame
        | otherwise                 = calculateOffsetMap shiftedMdv e' (fi + 1)
        where
            fI = F fi

            -- MDV with frame fi shifted by the new shift for the current frame (ce)
            shiftedMdv = modifyMotionDataVars mdvOrig ([((fi,0), fiRootJ{offset = (offset fiRootJ) + (V3 cex 0 cez)})])
            fiRootJ = j fi 0

            e' = e // [(fi,ce)]


            -- target ZMP position offset: via projection of zmp onto the SP towards the CoM
            -- TODO handle case where number of intersection points is 0 or more than 1 (take closest point to zmp -> smallest offset)
            (V2 zmpex zmpez)
                | spis == []  = V2 0 0
                | otherwise   = let zmpe = head $ sortBy (\a b -> compare (dot a a) (dot b b)) spis in trace ((show fi ++ " : zmpe : " ++ (show zmpe))) zmpe
            spis = map  (\x -> x-(toXZ (zmpO fI))) (polyEdgeLineSegIntersect poly (let zmp2Com = (toXZ (zmp fI), toXZ (com fI)) in trace (show $ zmp2Com) zmp2Com))

            poly = sp fI
--            poly = spO fI
--            expandFactor = 2
--            expandedSPO = map (\v -> ((v-centroidSPO)^*expandFactor) + centroidSPO ) poly
--            centroidSPO = (foldr1 (+) poly) ^/ (fromIntegral (length poly))

            -- current shift to get to the target ZMP
            ce@(cex,cez) = (
                    (zmpex - (b*(xe_2 - (2*xe_1)))) / (b), -- x
                    (zmpez - (b*(ze_2 - (2*ze_1)))) / (b)  -- z
                )
            (xe_2,ze_2) = if fi-2 >= fst (bounds e) then e!(fi-2) else (0,0)
            (xe_1,ze_1) = if fi-1 >= fst (bounds e) then e!(fi-1) else (0,0)

            b = negate $ sum [(m bi) * (vy (xb fI bi)) | bi <- bs]
                                        /
                     (sum [(m bi) * (vy (l' fI bi) + g) | bi <- bs] * dt2)


    dt2 = dt*dt
    


shiftPerFrame :: Array Int Vec3 -> MotionDataVars -> MotionDataVars
shiftPerFrame v md@MotionDataVars{_fs=fs,_j=j} = modifyMotionDataVars md [let joint = j fi 0 in ((fi,0), joint{offset = (offset joint) + v!fi}) | (F fi) <- fs]

shift :: Vec3 -> MotionDataVars -> MotionDataVars
shift v md@MotionDataVars{_fs=fs,_j=j} = modifyMotionDataVars md [let joint = j fi 0 in ((fi,0), joint{offset = (offset joint) + v}) | (F fi) <- fs]

blendIntoMotionData :: Double -> (Int -> Joint) -> MotionDataVars -> MotionDataVars
blendIntoMotionData time pose (mdv@MotionDataVars{_dt=dt,_j=j,_jN=jN,_fN=fN}) = modifyMotionDataVars mdv newJA where
    blendFrameI = ceiling $ time / dt
    blendFrameF = floor $ (time + blendTime) / dt
    newJ :: Int -> Int -> Joint
    newJ f
        | blendFrameI <= f && f <= blendFrameF   = blendFrames pose (j f) ((((fromIntegral f) * dt) - time) / blendTime)
        | otherwise                              = error "AAAAA"
    newJA = [(ix, newJ f i) | ix@(f, i) <- range ((blendFrameI,0),(blendFrameF,jN-1))]

dip :: Double -> MotionDataVars -> MotionDataVars
dip amount = shift (V3 0 (-amount) 0)

-- based off of "Online Generation of Humanoid Walking Motion based on a Fast Generation Method of Motion Pattern that Follows Desired ZMP"
-- origional motion data
-- target ZMP trajectory
-- dip (lower the body to extend leg reach)
-- number of itterations
fitMottionDataToZmp :: MotionDataVars -> Array Int Vec3 -> Double -> Int -> MotionDataVars
fitMottionDataToZmp = fitMottionDataToZmp' True

fitMottionDataToZmpLooseInitVel :: MotionDataVars -> Array Int Vec3 -> Double -> Int -> MotionDataVars
fitMottionDataToZmpLooseInitVel = fitMottionDataToZmp' False

fitMottionDataToZmp' :: Bool -> MotionDataVars -> Array Int Vec3 -> Double -> Int -> MotionDataVars
fitMottionDataToZmp' constrainInitVel mdvOrig zmpX dipHeight its = fitMottionDataToZmp' mdvOrig (listArray (bounds zmpX) [1,1..]) its where
    fitMottionDataToZmp' :: MotionDataVars -> Array Int Double -> Int -> MotionDataVars
    fitMottionDataToZmp' mdv _ 0 = mdv
    fitMottionDataToZmp' mdv weights itsI = finalModifiedMdv where
        MotionDataVars {
            _j       = j,
            _g       = g,
            _m       = m,
            _dt      = dt,
            _fN      = fN,
            _fs      = fs,
            _bs      = bs,
            _xb      = posb,
            _l'      = l',
            _zmp     = zmp
        } = mdv

        x f i = vx (posb (F f) i)
        y f i = vy (posb (F f) i)
        z f i = vz (posb (F f) i)

        -- here we setup:    M xe = xep
        --           and:    M ze = zep
        -- M is a matrix
        -- xe is the vector of shifts in body position (to be solved with a matrix solver)
        -- xep is the shift is ZMP poisitons through time (note that first and last elements are 0)

        -- generate the matrix M (list of position and values... sparse matrix)
        constraintFramesRix = (zip [0,fN-1] [0,fN-1]) ++ (if constrainInitVel then [(1,fN)] else [])
        mrn = fN - 2 + (length constraintFramesRix)
        _M :: [((Int,Int),Double)]
        _M =    -- fN-2 rows are the zmp = target zmp constraint for all frames other than the first and last
                (concat  [zip [(r,r-1), (r,r), (r,r+1)] (fmap (*(weights!r)) [f r, diag r, f r]) | r <- [1..fN-2]]) ++
                -- remaining rows ensure that there is no shift for the constraintFrames
                (map (\(f,rix) -> ((rix,f), 10000000000)) constraintFramesRix)
            where
                dt2         = dt ** 2
                f :: Int -> Double
                f           = memoize (0,fN-1) f' where
                                f' :: Int -> Double
                                f' r =  (negate (sum (map (\bI -> (m bI) * (y r bI)) bs))) /
                                            (dt2 * (sum (fmap (\bI -> (m bI) * ((vy $ l' fI bI) + g)) bs))) where
                                           fI = F r
                diag r = 1 - (2 * (f r))

        -- ep
        ep :: Array Int Vec3
        ep = array (0,mrn-1) (
                            -- first fN-2 rows is the zmp = target zmp constraint for all frames other than the first and last
                            [(fi, (weights!fi) *^ ((zmpX!fi) - (zmp (F fi)))) | fi <- range (1,fN-2)] ++
                            -- remaining rows ensure that ther is no shift for the constraintFrames
                            [(rix,V3 0 0 0) | (_,rix) <- constraintFramesRix])
        -- xep
        xep = fmap vx ep
        zep = fmap vz ep

        -- use matrix solver to get xe
        --  sparseMatrixSolve :: [((Int,Int),Double)] -> [Array Int Double] -> [Array Int Double]
        (ze:xe:_) = leastSquareSparseMatrixSolve mrn fN _M [zep,xep]

        shiftedFrames = [ let joint@Joint{offset=offset} = j fi 0 in ((fi, 0), joint{ offset = offset + (V3 (xe!fi) 0 (ze!fi)) })  | fi <- [0..fN-1] ]
        modifiedMdv@MotionDataVars{_zmp=zmpActualResult} = modifyMotionDataVars mdv shiftedFrames

        weightsLearnRateA = 30
        weightsLearnRateB = 2
        newWeightsRaw = array (bounds weights) [ (fi, (weights!fi) + ((weightsLearnRateA * (norm $ (zmpX!fi) - (zmpActualResult (F fi)))) ** weightsLearnRateB)) | fi <- indices weights]
        windowHalf = 2
        newWeights = weights -- let x = listArray (bounds weights) [ (sum (map (\fi -> if inRange (bounds weights) fi then newWeightsRaw!fi else 0) [fi-windowHalf..fi+windowHalf])) / (1 + (2*(fromIntegral windowHalf))) | fi <- indices weights] in traceShow (elems x) x

        --modifyMotionDataVars :: MotionDataVars -> FJIndexedFrames -> MotionDataVars
        doFeetIK = True
        finalModifiedMdv = fitMottionDataToZmp' ((if doFeetIK then (feetIK mdvOrig) else id) $ (if itsI == its then dip dipHeight else id) $ modifiedMdv) newWeights (itsI-1)



-- makes the soles of the feet parralle to the floor in all frames, and emphasises floor contact / ground clearance
flattenFeet :: MotionDataVars -> MotionDataVars
flattenFeet mdv@MotionDataVars{_dt=dt,_jBase=jBase,_fN=fN,_pj=pj,_bj=bj,_footBs=((lfbI,ltbI),(rfbI,rtbI))}  = mdv' where
    
    mdv' = 
        -- lower to allow feet to reach the target location
        ((\x -> feetIK mdv (shift (V3 0 (negate zeroAnkleClearance) 0) mdv))
        -- Move ankles to the correct height
        >>> fixAnkleClearance
        -- make the feet flat
        >>> fixFootOrientation
        -- Remove foot sliding
        >>> removeSliding
        ) mdv


    lejI = bj ltbI  -- left toe endaffector
    rejI = bj rtbI  -- right toe endaffector
    ltjI = pj lejI  -- left toe joint
    rtjI = pj rejI  -- right toe joint
    lajI = pj ltjI  -- left ankel joint
    rajI = pj rtjI  -- right ankel joint

    zeroEndClearance = 0     -- height of the end affector joint when the foot is flat on the floor
    zeroToeClearance = zeroEndClearance - (vy $ offset (jBase lejI))     -- height of the toe joint when the foot is flat on the floor
    zeroAnkleClearance = zeroToeClearance - (vy $ offset (jBase ltjI))     -- height of the ankle joint when the foot is flat on the floor
    
    -- return the distance a foot is from the floor
    footClearance :: MotionDataVars -> Bool -> FrameIx -> Double
    footClearance mdv@MotionDataVars{_soleCorners=soleCorners} isLeft fI = minimum $ map vy sole where
        (lSole,rSole) = soleCorners fI
        sole = if isLeft then lSole else rSole
    
    -- takes True if left foor or False for right foot, and MDV and frame index and returns True if the foot is in in a supporting phase
    isFootInSupportPhase :: MotionDataVars -> Bool -> FrameIx -> Bool
    isFootInSupportPhase mdv@MotionDataVars{_l=l} isLeft fI = clearance <= floorContactThreshold && vel <= floorContactVelocityThreshold where
        footBone = if isLeft then lfbI else rfbI
        vel = norm (l fI footBone)
        clearance = footClearance mdv isLeft fI
        

    -- exadurates the feet height, such that it is clearer when the foot is contacting the floor
    fixAnkleClearance mdv@MotionDataVars{_xj=xj,_l=l,_soleCorners=soleCorners} = ankleIK mdv (buildArray (0,fN-1) aT) where

        -- takes the clearance from the floor, and ankle velocity, then returns the target height of the ankle
        heightMap :: Double -> Vec3 -> Double
        heightMap clearance vel = newClearance + zeroAnkleClearance where

            newClearance =
                -- anything within floorContactThreshold clearance and floorContactVelocityThreshold velocity should have 0 clearance
                -- if withing the floorContactThreshold, but velocity is too large, then keep just above floorContactThreshold
                if clearance <= floorContactThreshold && norm vel <= floorContactVelocityThreshold then 0 else
                 -- else if less than safeFloorClearance, heavily push clearance toward safeFloorClearance
                 if clearance <= safeFloorClearance      then  (safeFloorClearance * (((max floorContactThreshold clearance)/safeFloorClearance)**floorClearanceExponent)) else
                 -- else leave as it is
                  clearance

        aT fi = let
                    fI = F fi
                    hj = vy . (xj fI)
                    (lSole,rSole) = soleCorners fI
                in
                        (let (V3 x y z) = xj fI lajI in V3 x (heightMap (minimum $ map vy lSole) (l fI lfbI)) z,
                         let (V3 x y z) = xj fI rajI in V3 x (heightMap (minimum $ map vy rSole) (l fI rfbI)) z)

    -- Set the feet to be parallel to the ground
    fixFootOrientation mdv@MotionDataVars{_j=j,_pj=pj,_fs=fs,_rj=rj,_jByName=jByName} = modifyMotionDataVars mdv (concat [modFrame fi | (F fi) <- fs]) where

        (ankleJis,toeJis) = splitAt 2 (map ((\(J ji) -> ji) . jByName) ["LeftAnkle", "RightAnkle","LeftToe","RightToe"])
        modFrame fi = (map (modAnkles fi) ankleJis) ++ (map (modToes fi) toeJis)
        modAnkles fi ji = let joint = j fi ji in ((fi,ji), joint{ rotationL = (rotationL joint) * (conjugate $ rj (F fi) (J ji)) })
        modToes fi ji = let joint = j fi ji in ((fi,ji), joint{ rotationL = identity })
        
    
    removeSliding :: MotionDataVars -> MotionDataVars
    removeSliding mdv@MotionDataVars{_xj=xj} = ankleIK mdv ankleTargets where
        -- do this for each foot independently
        -- identify floor contact frames / runs. A left and right foot list of start and end indexes of the contact runs
        -- contactRuns :: [(Int,Int)]
        (lContactRuns,rContactRuns) = (toRuns lContactFrames, toRuns rContactFrames) where
            
            (lContactFrames,rContactFrames) = (buildArray (0,fN-1) ((isFootInSupportPhase mdv True) . F), buildArray (0,fN-1) ((isFootInSupportPhase mdv False) . F))
            
            -- loop through the array maintaining a list of runs
            toRuns :: Array Int Bool -> [(Int,Int)]
            toRuns vals = reverse $ init $ toRuns' 0 [(-2,-2)] where
                toRuns' i runs'@((s,e):rs)
                    | i == fN   = runs'
                    -- if in run, check if this is part of the last run or the start if a new one, and update runs' accordingly
                    | vals!i    = toRuns' (i+1) $ if i == e+1 then (s,i):rs else (i,i):runs'
                    | otherwise = toRuns' (i+1) runs'
                    
        
        -- pick a single foot position for each floor contact run
        -- ankleRunPos :: [Vec3]
        (lAnkleRunPos,rAnkleRunPos) = (map (runAnkleAvg lajI) lContactRuns, map (runAnkleAvg rajI) rContactRuns) where
            runAnkleAvg ajI (runBnds@(s,e)) = (sum [xj (F fi) ajI | fi <- range runBnds]) ^/ (fromIntegral (1+e-s))
            
        -- calculate new ankle positions per frame, blending floor contact XZ components into non-floor contact phases
        ankleTargets :: Array Int (Vec3,Vec3)
        ankleTargets = array (0,fN-1) (zipWith (\(i1,v1) (i2,v2) -> if i1 /= i2 then error "flattenFeet.ankleTargets: incorrect array indexing" else (i1,(v1,v2))) (getTargets lajI 0 (zip lContactRuns lAnkleRunPos)) (getTargets rajI 0 (zip rContactRuns rAnkleRunPos))) where
            -- the 3rd argument to getTargets:
            --   ensure that this always contains the rest of the runs, starting with the run just before fi (or the run that fi is in). 
            --   if there is no run before fi, then the list starts with the next run.
            --   if there is no run at all (an unusual situation) then this is simply an empty list
            getTargets :: JointIx -> Int -> [((Int,Int),Vec3)] -> [(Int,Vec3)]
            getTargets ajI fi [] = map (\fi' -> (fi', xj (F fi') ajI)) [fi..(fN-1)]
            getTargets ajI fi (runs'@[((s,e),aPos)])
                | fi >= fN              = []
                | fi < s                = (fi, useYandZX xjjI ((u*^(xjjI)) + ((1-u)*^aPos))) : getTargets ajI (fi+1) runs'
                | fi <= e               = (zip [fi..e] (replicate (1+e-fi) aPos)) ++ (getTargets ajI (e+1) runs')
                | e < fi                = (fi, useYandZX xjjI ((v*^(xjjI)) + ((1-v)*^aPos))) : getTargets ajI (fi+1) runs'
                where
                    fI = F fi
                    xjjI = xj fI ajI
                    u = min 1 ((fromIntegral (s-fi) * dt) / stepBlendTime)
                    v = min 1 ((fromIntegral (fi-e) * dt) / stepBlendTime)
            getTargets ajI fi (runs'@(((s1,e1),aPos1):((s2,e2),aPos2):_))
                -- stop at the last frame
                | fi >= fN              = []
                -- just before run 1, so blend into it
                | fi <  s1              = (fi, useYandZX xjjI ((u*^(xjjI)) + ((1-u)*^aPos1))) : getTargets ajI (fi+1) runs'
                -- in run 1, so just use aPos1 for the whole run
                | fi <= e1              = (zip [fi..e1] (replicate (1+e1-fi) aPos1)) ++ (getTargets ajI (e1+1) runs')
                -- inbetween run 1 and 2, so blend into both and average the results
                | fi <  s2              = (fi, useYandZX xjjI ((((v + w)*^(xjjI)) + ((1-v)*^aPos1) + ((1-w)*^aPos2)) ^* 0.5)) : getTargets ajI (fi+1) runs'
                -- in run 2, so just use aPos2 for the whole run. Note that run 1 has no affect in the future frames, so we tail runs' (also run 3 may be needed next)
                | fi <= e2              = (zip [fi..e2] (replicate (1+e2-fi) aPos2)) ++ (getTargets ajI (e2+1) (tail runs'))
                where
                    fI = F fi
                    xjjI = xj fI ajI
                    u = min 1 ((fromIntegral (s1-fi) * dt) / stepBlendTime)
                    v = min 1 ((fromIntegral (fi-e1) * dt) / stepBlendTime)
                    w = min 1 ((fromIntegral (s2-fi) * dt) / stepBlendTime)
                    x = min 1 ((fromIntegral (fi-e2) * dt) / stepBlendTime)
    
            -- combine 2 Vec3, using y from the first vector and x and z from the second vector 
            useYandZX (V3 _ y _) (V3 x _ z) = V3 x y z


    

-- use the feedback control described in "Posture/Walking Control for Humanoid Robot Based on Kinematic Resolution of CoM Jacobian With Embedded Motion"
-- section V (this is only the P-controler like feedback signal for the desired CoM velocity..... equation (44))
applyCoMVelFeedbackControler :: Double -> Double -> MotionDataVars -> Array Int Vec3 -> MotionDataVars
applyCoMVelFeedbackControler kc kp targetMotion comError = newMotion where

    MotionDataVars{
        _dt=dt,
        _fN=fN,
        _com=comT,
        _zmp=zmpT
    } = targetMotion

    newMotion = applyCoMVelFeedbackControler' 0 targetMotion
    applyCoMVelFeedbackControler' fi (mdvA@MotionDataVars{_j=jA,_com=comA,_zmp=zmpA}) = if fi < fN-1
        then
            applyCoMVelFeedbackControler' (fi+1) (seq nextOffset mdvA')
        else
            mdvA where

                fI = F fi
                fiNext = fi+1
                fINext = F fiNext

                -- errors (ZMP and CoM) comparing frame fI in target and actual
                ep = (zmpT fI) - (zmpA fI)
                ec = (comT fI) - (comA fI)

                -- current desired CoM velocity (use current and next frame only)
                lT = ((comT fINext) - (comT fI)) ^/ dt

                -- adjusted CoM velocity (based on equation (44))
                ld = lT - (kp *^ ep) + (kc *^ ec)

                -- now we adjust the next frame (fi + 1) to reflect the adjusted CoM velocity (ld')
                -- in this case we consider only frames fi and fi+1 when considering velocity. This
                -- means we only calculate the offset over 1 frame given the velocity and apply that
                -- to frame fi+1 (we don't consider a 3rd frame as is done in finite difference)
                --
                -- here we convert shift in CoM to shift in Root node (they are not the same thing)
                rootJoint = (jA fi 0)
                nextFrameRootJoint = (jA fiNext 0)
                targetCoM = (comA fI) + (ld ^* dt)
                --     This is the desired shift in root node
                rootShift = (targetCoM + ((offset nextFrameRootJoint) - (comA fINext))) - (offset rootJoint)
                --     here is the resulting offset (origional offset + shift) and now we add some noise (from comError)
                nextOffset = (offset rootJoint) + rootShift + (comError!fiNext)

--                mdvA' = mdvA
                mdvA' = modifyMotionDataVarsFN mdvA (fi+2) [((fiNext,0), nextFrameRootJoint{
                    offset = nextOffset
                })]


