module Main where

import MotionData
import Constants
import Data.Maybe
import Data.TreeF
import FFI
import Linear
import Util
import Control.Concurrent
import Data.Time.Clock

main :: IO ()
main = do
    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-025_David.bvh"
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
    
    --drawSkeleton $ (frames md) !! 0
    drawSkeleton $ (frames md) !! (min maxFIndex ((floor $ t / ft) `mod` maxFIndex))
    notDone <- doRender
    
    --threadDelay $ floor $ (frameTime md) * 1000000
    if notDone
        then mainLoop md (t+0.03)
        else return ()

   
drawSkeleton :: Frame -> IO ()
drawSkeleton = treeMapM_ drawBone' where
     drawBone' :: JointF -> IO ()
     drawBone' j
            | hasParent j   = drawBone (getPosStart j) (getPosEnd j) boneRadius
            | otherwise     = return ()
            
            
{-

emptyJoint = Joint{
                name = "lol",
                offset = V3 0 0 0,
                rotation = identity,
                channels = [Xpos,Ypos]}
                
testF = toFocus (Tree emptyJoint [Tree emptyJoint [],Tree emptyJoint []])
            
  -}          