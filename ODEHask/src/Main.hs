{-# LANGUAGE BangPatterns #-}

module Main where

import MotionData
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


getArguments :: IO (String, Integer)
getArguments = do
    ws <- getArgs
    return (getPath ws, getPP ws) where
        getPath ws = case find (not . (isPrefixOf "-")) ws of
                Just path   -> if elem '/' path then path else "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/" ++ path
                Nothing     -> "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/david-1-martialarts-004_David.bvh"

        getPP ws = case find (isPrefixOf "-pp") ws of
                Just w      -> read (drop 3 w)
                Nothing      -> 0


main :: IO ()
main = main'

main' :: IO ()
main' = do

    (file, pp) <- getArguments

    md <- fmap (scaleAndTranslate 2 0) $ parseBVH file pp

    mainLoopOut md


mainLoop =
--    viewAnimationLoop

    -- setting kc to the data's framerate (usually 60Hz) and keeping kp=0 the origional target motion
    -- will be retreived.
    coMFeedBackLoopExperiment [73] [0]

mainLoopOut :: MotionData -> IO ()
mainLoopOut md = do

    displayTIOs <- mainLoop (getMotionDataVariablesFromMotionData md)

    let
        loopDisplay :: [(Double, IO ())] -> [(Double, IO ())] -> UTCTime -> IO ()
        loopDisplay allIOs ios tLastRender = do
            tc <- getCurrentTime
            let
                frameDT = (realToFrac $ tc `diffUTCTime` tLastRender)
                ios'@((_,io):_) = forward ios frameDT

                forward [] t = forward allIOs t
                forward ((dt,io):rest) t
                    | t < dt    = (dt-t,io):rest
                    | otherwise = forward rest (t - dt)

            io
            doRender
            loopDisplay allIOs ios' tc

    ti <- getCurrentTime
    initOgre
    loopDisplay displayTIOs displayTIOs ti

    return ()


