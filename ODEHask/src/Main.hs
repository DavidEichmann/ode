module Main where

import MotionData
import Constants
import Data.Maybe
import Data.TreeF
import Data.Color
import FFI
import Linear
import Util
import Control.Concurrent
import Data.Time.Clock


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


main :: IO ()
main = do
    let pp = True
    
    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-025_David.bvh" pp
--    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-011_David.bvh" pp

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
            initOgre
            mainLoopOut md
    
    
mainLoopOut :: MotionData -> IO ()
mainLoopOut md = do
    ti <- getCurrentTime
    loop ti where
        loop ti = do
            tc <- getCurrentTime
            mainLoop md (realToFrac $ diffUTCTime tc ti)
            loop ti

mainLoop :: MotionData -> Double -> IO ()
mainLoop md t = do

    let
        ft = frameTime md
        fLength = length $ frames md
        maxFIndex = fLength -1
        cFrame = (frames md) !! (min maxFIndex ((floor $ t / ft) `mod` maxFIndex))
        
        endSite0 = des cFrame where
            des f = maybe f right (child0 f)
            right f = maybe (des f) right (rightSib f)
        c0 = fromJust $ child0 cFrame 
        
        com = getPosTotalCom cFrame
        
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

    -- Annimation    
    drawSkeleton $ cFrame
    -- ZMP
    drawPointC Green ((getZMP cFrame)) 0.04
    -- COM
    drawPointC Yellow com 0.04
    -- COM floor
    let V3 x _ z = com in drawPointC Yellow (V3 x 0 z) 0.04
    
    -- print output
--    putStrLn $ "tot lin momentum = " ++ (show $ norm $ getTotalLinearMomentum cFrame)
--    putStrLn $ "getTotalAngularMomentum' = " ++ (show $ getTotalAngularMomentum' cFrame)
    
    
    -- local props for all joits (not root)
    treeMapM_ drawLocalJointProps cFrame
    
--    drawVec3C Black com ((getTotalAngularMomentum' cFrame) ^* 0.1) 0.015
--    drawVec3C Green com ((getTotalLinearMomentum' cFrame) ^* 0.1) 0.015
    --drawVec3 com ((getTotalLinearMomentum cFrame) ^* 0.2) 0.05
    --drawVec3 (getPosTotalCom endSite0) ((getTotalLinearMomentum endSite0) ^* 1) 0.03
    
    
    -- do render, sleep, then loop
    notDone <- doRender
    --threadDelay $ floor $ (frameTime md) * 1000000
    if notDone
        then mainLoop md (t+0.03)
        else return ()

   
drawSkeleton :: Frame -> IO ()
drawSkeleton = treeMapM_ drawBone' where
     drawBone' :: JointF -> IO ()
     drawBone' j
            | hasParent j   = drawBoneC (RedA 0.5) (getPosStart j) (getPosEnd j) boneRadiusDisplay
            | otherwise     = return ()
            




                
testF = toFocus (Tree nullJoint{name="root"} [Tree nullJoint{name="c1"} [Tree nullJoint{name="c11"} [],Tree nullJoint{name="c12"} []]])
            
     
