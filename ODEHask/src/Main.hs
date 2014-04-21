{-# LANGUAGE BangPatterns #-}

module Main where

import MotionData
import Constants
import Data.Maybe
import Data.List
import Data.TreeF
import Data.Color
import Simulation
import FFI
import Linear
import Util
import Control.Concurrent
import Control.DeepSeq
import Data.Time.Clock
import System.Environment


debug =
--    True
    False


doDebug md = do
    
    --print $ treeMap' (name $-) testF
    
    
    let
        printDynamics tf = do
            putStrLn "----------"
            print $ view tf
            putStrLn ""
            mapM_ (\f -> print $ f $ tf) [
--                --getTotalMass,
                getPosTotalCom,
                getLinearVel,
                getLinearAcc,
                getAngularVel,
                getAngularAcc,
                getLinearMomentum,
                getTotalLinearMomentum,
                getTotalLinearMomentum',
                getAngularMomentum,
                getAngularMomentum',
                getTotalAngularMomentum,
                getTotalAngularMomentum'
                ,getZMP
               ]
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
        getPath (('-':_):_:ws) = getPath ws
        getPath (path:_)
                | elem '/' path  = path
                | otherwise      = "/home/david/git/ode/ODE_01/Data/Animation/" ++ path
        getPath [] = "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-004_David.bvh"
        
        getPP ("-pp":nStr:_)
                | all (flip elem ['0'..'9']) nStr  = read nStr
                | otherwise                        = 0
        getPP (_:ws) = getPP ws
        getPP [] = 0


main :: IO ()
main = do

    (file, pp) <- getArguments
    
    md <- fmap (scaleAndTranslate 2 10) $ parseBVH file pp
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
    loop sim ti ti where
        loop sim ti tl = do
            tc <- getCurrentTime
            sim' <- mainLoop sim (realToFrac $ diffUTCTime tc ti) (realToFrac $ diffUTCTime tc tl)
            loop sim' ti tc

mainLoop :: Sim -> Double -> Double -> IO Sim
mainLoop sim t dt = do
    sim' <- step sim dt
--    let sim' = sim
    cSimFrame <- getSimSkel sim'
    let
        md = targetMotion sim
        ft = frameTime md
        fLength = length $ frames md
        maxFIndex = fLength -1
        cAniFrame = (frames md) !! (min maxFIndex ((floor $ t / ft) `mod` maxFIndex))
        
--        endSite0 = des cFrame where
--            des f = maybe f right (child0 f)
--            right f = maybe (des f) right (rightSib f)
--        c0 = fromJust $ child0 cFrame 
--        
--        com = getPosTotalCom cFrame
        
        drawLocalJointProps j = do
            if hasParent j
                then do
--                    drawVec3 (getPosCom j) ((getLinearVel j) ^* 0.5) 0.02
--                    drawVec3 (getPosCom j) ((getLinearAcc j) ^* 0.5) 0.02
--                    drawVec3 (getPosCom j) ((getAngularVel j) ^* 0.5) 0.05
--                    drawVec3 (getPosCom j) ((getAngularAcc j) ^* 0.5) 0.01
--                    drawVec3 (getPosCom j) ((getAngularMomentum j) ^* 10) 0.05
--                    drawVec3 (getPosCom j) ((getAngularMomentum j) ^* 1) 0.05
--                    print $ getAngularVel j
--                    print $ getAngularMomentum' j
                    return ()
                else
                    return ()
    -- sim visualization
    drawSkeleton $ cSimFrame
    
    -- Annimation
    let aniOffset =   zero --V3 (-2) 0 0
    drawFrame aniOffset cAniFrame
    -- ZMP
    drawPointC Green (aniOffset + (getZMP cAniFrame)) 0.04
    -- COM
--    drawPointC Yellow com 0.04
    -- COM floor
--    let V3 x _ z = com in drawPointC Yellow (V3 x 0 z) 0.04
    
    -- print output
--    putStrLn $ "tot lin momentum = " ++ (show $ norm $ getTotalLinearMomentum cFrame)
--    putStrLn $ "getTotalAngularMomentum' = " ++ (show $ getTotalAngularMomentum' cFrame)
    
    
    -- local props for all joits (not root)
--    treeMapM_ drawLocalJointProps cFrame
    
--    drawVec3C Black com ((getTotalAngularMomentum' cFrame) ^* 0.1) 0.015
--    drawVec3C Green com ((getTotalLinearMomentum' cFrame) ^* 0.1) 0.015
    --drawVec3 com ((getTotalLinearMomentum cFrame) ^* 0.2) 0.05
    --drawVec3 (getPosTotalCom endSite0) ((getTotalLinearMomentum endSite0) ^* 1) 0.03
    
    
    -- do render, sleep, then loop
    notDone <- doRender
    
    
    
    --threadDelay $ floor $ (frameTime md) * 1000000
--    if notDone
--        then mainLoop sim (t+0.03)
--        else return ()
    return sim'

   
   
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
            
     
