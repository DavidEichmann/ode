module Main where

import MotionData
import Constants
import Data.Maybe
import MotionData.BVHParser
import FFI
import Linear
import Util
import Control.Concurrent
import Data.Time.Clock

main :: IO ()
main = do
    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-005_David.bvh"
    --print $ setJoint ((getJoint testF){offset=V3 0 0 1}) testF
    --print $ jointMap (\f -> setJoint ((getJoint f){offset=V3 0 0 1}) f) testF
    --print $ getJoint $ fromJust . child0 $ (frames md)!!10
    
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
        maxFIndex = length $ frames md
    
    drawSkeleton $ (frames md) !! (min maxFIndex (floor $ t / ft))
    notDone <- doRender
    
    --threadDelay $ floor $ (frameTime md) * 1000000
    if notDone
        then mainLoop md (t+0.03)
        else return ()

   
drawSkeleton :: Frame -> IO ()
drawSkeleton = jointMapM_ drawBone' where
     drawBone' :: JointF -> IO ()
     drawBone' j
            | hasParent j   = drawBone (getPosStart j) (getPosEnd j) boneRadius
            | otherwise     = return ()
            
            

emptyJoint = Joint{
                name = "lol",
                offset = V3 0 0 0,
                rotation = identity,
                channels = [Xpos,Ypos]}
                
testF = toFocus (JointTree emptyJoint [JointTree emptyJoint [],JointTree emptyJoint []])
            
            