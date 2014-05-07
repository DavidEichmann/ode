{-# LANGUAGE BangPatterns #-}

module Main where

import MotionData
import Constants
import Data.List
import Data.TreeF
import Data.Array
import Data.Color
import Simulation
import FFI
import Linear
import Util
import Data.Time.Clock
import System.Environment


debug =
--    True
    False


doDebug md = do

    --print $ treeMap' (name $-) testF


--    let
--        printDynamics tf = do
--            putStrLn "----------"
--            print $ view tf
--            putStrLn ""
--            mapM_ (\f -> print $ f $ tf) [
----                --getTotalMass,
--
--               ]
--    print $ getTotalMass (baseSkeleton md)
--    treeMapM_ printDynamics (frames md !! 10)
--    mapM_ (printDynamics) (take 30 (map (fromJust . child0) (frames md)))
--    mapM_ (print . getZMP) (frames md)
    return ()

-- get (bvhFilePath, preProcessInteger
getArguments :: IO (String, Integer)
getArguments = do
    ws <- getArgs
    return (getPath ws, getPP ws) where
        getPath ws = case find (not . (isPrefixOf "-")) ws of
                Just path   -> if elem '/' path then path else "/home/david/git/ode/ODE_01/Data/Animation/" ++ path
                Nothing     -> "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-004_David.bvh"

        getPP ws = case find (isPrefixOf "-pp") ws of
                Just w      -> read (drop 3 w)
                Nothing      -> 0


main :: IO ()
main = do

    (file, pp) <- getArguments

    md <- fmap (scaleAndTranslate 2 0) $ parseBVH file pp
--    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-025_David.bvh" pp
--    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-011_David.bvh" pp

--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/testDrop.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test_sin.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test_sinZ.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test_spin.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test_spin_acc.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test_spinZ_acc.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/test_arm_spin.bvh" pp

    if debug
        then
            doDebug md
        else do
            seq (foldl1' seq (show md)) (putStrLn "Done processing data.")
            initOgre
            sim <- startSim md
            mainLoopOut sim

mainLoopOut :: Sim -> IO ()
mainLoopOut sim = do
    ti <- getCurrentTime
--    mapM_ (putStrLn . show) (zip5 (repeat "zmpinsp xz^e zmpMod zmp") (map zmpIsInSp fs) (elems e) (map (vx . zmpMod) fs) (map (vx . zmp) fs))
--    mapM_ (putStrLn . show) (zip3 (repeat "comMod com") (map (vx . comMod) fs) (map (vx . com) fs))
    loop sim ti ti where
        (mdvActual@MotionDataVars{_fN=fN,_fs=fs,_pT=pT,_l=l,_com=com,_zmp=zmp,_sp=sp,_zmpIsInSp=zmpIsInSp}) = getMotionDataVariablesFromMotionData $ targetMotion sim

        comI = zmp (F 0)
        comIF = (zmp (F (fN-1))) - comI
        bnds = (0,fN-1)
        targetZmp = array bnds [(i, let t = ((fromIntegral i) / (fromIntegral (fN-1))) in comI + (t*comIF) )  | i <- range bnds]

        mdvMod = fitMottionDataToZmp mdvActual targetZmp

        loop sim ti tl = do
            tc <- getCurrentTime
            sim' <- mainLoop sim mdvActual mdvMod targetZmp (realToFrac $ diffUTCTime tc ti) (realToFrac $ diffUTCTime tc tl)
            loop sim' ti tc

mainLoop :: Sim -> MotionDataVars -> MotionDataVars -> Array Int Vec3 -> Double -> Double -> IO Sim
mainLoop
  sim
  mvdOrig@MotionDataVars{_zmpIsInSp=aniZmpIsInSp,_com=targetZmp'@aniCom,_zmp=aniZmp,_sp=aniSp}
  mdv@MotionDataVars{_zmpIsInSp=zmpIsInSp,_dt=dt,_fs=fs,_fN=fN,_bs=bs,_zmp=zmp}
  targetZmp
  t'
  simdt = do
    --sim' <- step sim simdt
    let
        t = t'
        sim' = sim
    cSimFrame <- getSimSkel sim'
    let
        maxFIndex = fN - 1
        frameix = 1 + ((floor $ t / dt) `mod` maxFIndex)
        frameIx = F frameix

    -- sim visualization
--    drawSkeleton $ cSimFrame

    -- Annimation
    let aniOffset =   zero --V3 (-2) 0 0
    drawFrameIx White aniOffset mdv frameIx
    drawFrameIx (BlackA 0.25) aniOffset mvdOrig frameIx
    -- SP
--    drawPolygonC (OrangeA 0.3) (map (\(V2 x z) -> V3 x 0 z) (aniSp frameIx))
    drawPolygonEdgesC Black (map (\(V2 x z) -> V3 x 0 z) (aniSp frameIx))
    mapM_ ((\p -> drawPointC Blue p 0.02) . (\(V2 x z) ->V3 x 0 z)) (polyEdgeLineSegIntersect (aniSp frameIx) (toXZ (aniZmp frameIx), toXZ (aniCom frameIx)))

    -- ZMP
    putStrLn $ "zmp: " ++ (show $ zmp frameIx)
    drawPointC (Red) (zmp frameIx) 0.02
    drawPointC (WhiteA 0.4) (targetZmp!(frameix)) 0.04
--    drawPointC (if zmpIsInSp frameIx then White else Red) (aniOffset + (zmp frameIx)) 0.02
--    drawPointC (if aniZmpIsInSp frameIx then (WhiteA 0.5) else (OrangeA 0.5)) (aniOffset + (aniZmp frameIx)) 0.04
--    drawPointC Red (aniOffset + (targetZmp frameIx)) 0.04
    -- COM
    drawPointC Yellow ((* (V3 1 0 1)) $ aniCom frameIx) 0.02


    -- do render, sleep, then loop
    notDone <- doRender



    --threadDelay $ floor $ (frameTime md) * 1000000
--    if notDone
--        then mainLoop sim (t+0.03)
--        else return ()
    return sim'

drawPolygonEdgesC :: Color -> [Vec3] -> IO ()
drawPolygonEdgesC _ [] = return ()
drawPolygonEdgesC c poly = mapM_ (\(a,b) -> drawBoneC c a b 0.005) (zip poly ((tail poly) ++ [head poly]))

drawFrameIx ::  Color -> Vec3 -> MotionDataVars -> FrameIx -> IO ()
drawFrameIx c offset' MotionDataVars{_bs=bs,_xsb=xsb,_xeb=xeb} fi = mapM_ drawBone' bs where
     drawBone' :: BoneIx -> IO ()
     drawBone' bi =  drawBoneC c (offset' + xsb fi bi) (offset' + xeb fi bi) boneRadiusDisplay

drawFrame ::  Vec3 -> Frame -> IO ()
drawFrame offset' j = treeMapM_ drawBone' (set ((view j){offset = (offset $- j) + offset'}) j) where
     drawBone' :: JointF -> IO ()
     drawBone' j
            | hasParent j   = drawBoneC (BlueA 0.5) (getPosStart j) (getPosEnd j) boneRadiusDisplay
            | otherwise     = return ()

drawSkeleton :: [Bone] -> IO ()
drawSkeleton = mapM_ drawBone' where
     drawBone' :: Bone -> IO ()
     drawBone' (Long start end) = drawBoneC (RedA 0.5) start end boneRadiusDisplay
     drawBone' (Box lengths center rot) = drawBoxC (RedA 0.5) lengths center rot



testF = toFocus (Tree nullJoint{name="root"} [Tree nullJoint{name="c1"} [Tree nullJoint{name="c11"} [],Tree nullJoint{name="c12"} []]])


