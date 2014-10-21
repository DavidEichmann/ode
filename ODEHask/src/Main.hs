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
import Data.Ix
import Data.Array
import Linear hiding (_j)
import Util
import Data.Time.Clock
import System.Environment
import Control.DeepSeq
import System.Random
import Math.Tau
import Test.MyTest
import System.IO


{-
mainLoop :: MainLoop
mainLoop =
--    viewAnimationLoopDefault
    viewFlatFeetSim
--    viewFlatFeet
--    viewSim
--    viewZmpCorrection

    -- setting kc to the data's framerate (usually 60Hz) and keeping kp=0 the origional target motion
    -- will be retreived.
--    coMFeedBackLoopExperiment [73] [0]
-}









getArguments :: IO (Bool, Bool, Bool, String, Integer, ImpulseType, Double, Maybe (Vec3, [Vec2], [Vec3]))
getArguments = do
    ws <- getArgs
    return (dontDisplay ws, isTest ws, raw ws, getPath ws, getPP ws, getImp ws, getDip ws, getExperiment ws) where
        isTest = elem "-unitTest"
        raw = elem "-raw"
        dontDisplay = elem "-dontDisplay"
        getPath ws = case find (not . (isPrefixOf "-")) ws of
                Just path   -> if elem '/' path then path else defaultDataDirectory ++ path
                Nothing     -> "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/david-1-martialarts-004_David.bvh"

        getPP ws = case find (isPrefixOf "-pp") ws of
                Just w      -> read (drop 3 w)
                Nothing      -> 0
        getDip ws = case find (isPrefixOf "-dip") ws of
                Just w      -> read (drop 4 w)
                Nothing      -> 0
        getImp ws = case elemIndex "-punch" ws of
                Just i  -> AutoPunch 
                    -- offset of impact position
                    (V2 (read $ ws!!(i+1)) (read $ ws!!(i+2)))
                    -- impulse vector
                    (V3  (read $ ws!!(i+3)) (read $ ws!!(i+4)) (read $ ws!!(i+5)))
                Nothing -> AutoPunch (V2 0 0) (V3 0 0 0)
        
        -- experiment options:
        --  V3 impulse direction
        --  min step max  impact translation range
        --  min step max  impact size
        getExperiment ws = case elemIndex "-experiment" ws of
                Just i  -> Just (impDir, ts, imps)
                    where
                        ds = map (read . (ws!!)) [i+1..]
                        impDir = normalize $ V3 (ds!!0) (ds!!1) (ds!!2)
                        tsSubRangeX = [(ds!!3), (ds!!3) + (ds!!4) .. (ds!!5)]
                        tsSubRangeZ = [(ds!!6), (ds!!6) + (ds!!7) .. (ds!!8)]
                        ts = [V2 x z | x <- tsSubRangeX, z <- tsSubRangeZ]
                        imps = map (impDir ^*) [(ds!!9), (ds!!9) + (ds!!10) .. (ds!!11)]
                Nothing -> Nothing
                




runExperiments :: Bool -> String -> Double -> MotionData -> (Vec3, [Vec2], [Vec3]) -> IO ()
runExperiments dontDisplay name dip mdRaw (impDir, ts, imps) = do

    
    if dontDisplay then return () else initOgre

    putStrLn $ "Running experiments on file " ++ name
    let
        mdv = officialFeedForwardControllerPreprocess dip (getMotionDataVariablesFromMotionData mdRaw)
        
        doExperiment' t imp = do
            let
                mdv' = officialFeedForwardControllerCorrections (setImpulseType (AutoPunch t imp) mdv)
            (isStanding, squareCoMerror) <- doExperiment dontDisplay mdv'
            
            -- print out the experiment results
            putStrLn $ "# experiment (t, imp,   isStanding, squareCoMerror) #   " ++ show t ++ ",\t" ++ show imp ++ ",\t" ++ show isStanding ++ ",\t" ++ show squareCoMerror
            hFlush stdout
            
            return ()
            
    sequence_ $ [doExperiment' t imp | t <- ts, imp <- imps]
    
    


main :: IO ()
main = main'

main' :: IO ()
main' = do

    initODE defaultTimeStep
    
    (dontDisplay, isTest, isRaw, file, pp, impulseType, dip, experiment) <- getArguments
    
    if isTest
        then
            runTests
        else do
            md <-  (if isRaw then id else fmap scaleAndTranslate) $ parseBVH file pp
            maybe 
                (mainLoopOut dontDisplay md impulseType dip)
                (runExperiments dontDisplay file dip md)
                experiment

mainLoopOut :: Bool -> MotionData -> ImpulseType -> Double -> IO ()
mainLoopOut dontDisplay md impulseType dip = do
    let mdv = (officialFeedForwardControllerCorrections $ setImpulseType impulseType $ officialFeedForwardControllerPreprocess dip $ getMotionDataVariablesFromMotionData md)
    putStrLn $ "total mass: " ++ (show (_mT mdv))
    
    displayTIOs <- simulateMainLoop mdv

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
--        doExperiment False mdv
        loopDisplay displayTIOs displayTIOs ti

    return ()


