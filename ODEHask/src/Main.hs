{-# LANGUAGE BangPatterns #-}

module Main where

import Motion
import Constants
import Routines
import Data.TreeF
import Data.Color
import Simulation
import FFI

import Data.List
import Data.Array
import Linear hiding (_j)
import Util
import Data.Time.Clock
import System.Environment
import Control.DeepSeq
import System.Random
import Math.Tau
import Test.MyTest


getArguments :: IO (Bool, Bool, Bool, String, Integer)
getArguments = do
    ws <- getArgs
    return (dontDisplay ws, isTest ws, raw ws, getPath ws, getPP ws) where
        isTest = elem "-unitTest"
        raw = elem "-raw"
        dontDisplay = elem "-dontDisplay"
        getPath ws = case find (not . (isPrefixOf "-")) ws of
                Just path   -> if elem '/' path then path else defaultDataDirectory ++ path
                Nothing     -> "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/david-1-martialarts-004_David.bvh"

        getPP ws = case find (isPrefixOf "-pp") ws of
                Just w      -> read (drop 3 w)
                Nothing      -> 0


main :: IO ()
main = main'

main' :: IO ()
main' = do

    (dontDisplay, isTest, isRaw, file, pp) <- getArguments
    
    if isTest
        then
            runTests
        else do
            md <-  (if isRaw then id else fmap scaleAndTranslate) $ parseBVH file pp
            mainLoopOut dontDisplay md

mainLoop :: MainLoop
mainLoop =
--    viewAnimationLoopDefault
--    viewFlatFeet
    viewFlatFeetSim
--    viewSim
--    viewZmpCorrection

    -- setting kc to the data's framerate (usually 60Hz) and keeping kp=0 the origional target motion
    -- will be retreived.
--    coMFeedBackLoopExperiment [73] [0]

mainLoopOut :: Bool -> MotionData -> IO ()
mainLoopOut dontDisplay md = do
    let mdv = (setImpulseType (AutoPunch (V3 0 0 0)) $ getMotionDataVariablesFromMotionData md)
    putStrLn $ "total mass: " ++ (show (_mT mdv))
    displayTIOs <- mainLoop mdv

    let
        loopDisplay :: [(Double, IO ())] -> [(Double, IO ())] -> UTCTime -> IO ()
        loopDisplay allIOs ios tLastRender = do
            tc <- getCurrentTime
            let
                frameDT = playbackSpeed * (realToFrac $ tc `diffUTCTime` tLastRender)
                ios'@((_,io):_) = forward ios frameDT

                forward [] t = forward allIOs t
                forward ((dt,io):rest) t
                    | t < dt    = (dt-t,io):rest
                    | otherwise = forward rest (t - dt)

            io
            doRender
            loopDisplay allIOs ios' tc

    ti <- getCurrentTime
    if dontDisplay then return () else do
        initOgre
        loopDisplay displayTIOs displayTIOs ti

    return ()


