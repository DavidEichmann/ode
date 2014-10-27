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









getArguments :: IO (Bool, Bool, Bool, String, Integer, ImpulseType, Double, Maybe (Vec3, [Vec2], [Vec3]), Maybe (Vec2, Vec3, [Double], [Double]), Maybe Double)
getArguments = do
    ws <- getArgs
    return (dontDisplay ws, isTest ws, raw ws, getPath ws, getPP ws, getImp ws, getDip ws, getExperiment ws, getExperimentRobust ws, getImpBox ws) where
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
                    -- expected impulse vector
                    (V3  (read $ ws!!(i+3)) (read $ ws!!(i+4)) (read $ ws!!(i+5)))
                    -- actual impulse vector
                    (V3  (read $ ws!!(i+6)) (read $ ws!!(i+7)) (read $ ws!!(i+8)))
                Nothing -> AutoPunch (V2 0 0) (V3 0 0 0) (V3 0 0 0)
        getImpBox ws = case elemIndex "-box" ws of
                Just i  -> Just (read $ ws!!(i+1))
                Nothing -> Nothing
        
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
                
        getExperimentRobust ws = case elemIndex "-experimentRobust" ws of
                Just i  -> Just (t, impDir, imps, errs)
                    where
                        ds = map (read . (ws!!)) [i+1..]
                        t = V2 (ds!!0) (ds!!1)
                        impDir = normalize $ V3 (ds!!2) (ds!!3) (ds!!4)
                        imps = [(ds!!5), (ds!!5) + (ds!!6) .. (ds!!7)]
                        errs = [(ds!!8), (ds!!8) + (ds!!9) .. (ds!!10)]
                Nothing -> Nothing
                




runExperiments :: Bool -> String -> Double -> Maybe Double -> MotionData -> [(Vec2, Vec3, Vec3)] -> [a] -> ((a,(Bool,Double)) -> IO ()) -> IO ()
runExperiments dontDisplay name dip box mdRaw tImps putterData putter = do

    
    if dontDisplay then return () else initOgre

    putStrLn $ "Running experiments on file " ++ name
    let
        mdv = officialFeedForwardControllerPreprocess dip (getMotionDataVariablesFromMotionData mdRaw)

        doExperiment' (dat, (t, imp, actualImp)) = do
            let
                mdv' = officialFeedForwardControllerCorrections (setImpulseType (AutoPunch t imp actualImp) mdv)
            (isStanding, squareCoMerror) <- doExperiment dontDisplay box mdv'
            
            putter (dat, (isStanding, squareCoMerror))
        
        
    mapM_ doExperiment' (zip putterData tImps)


runExperimentsMaxImp :: Bool -> String -> Double -> Maybe Double -> MotionData -> (Vec3, [Vec2], [Vec3]) -> IO ()
runExperimentsMaxImp dontDisplay name dip box mdRaw (impDir, ts, imps) =  do
    let
    
        putExp  ((t,_,actualImp), (isStanding, squareCoMerror)) = do
            -- print out the experiment results
            putStrLn $ "# experiment (t, imp,   isStanding, squareCoMerror) #   " ++ show t ++ ",\t" ++ show actualImp ++ ",\t" ++ show isStanding ++ ",\t" ++ show squareCoMerror
            hFlush stdout
            
        ops = [(t,imp,imp) | t <- ts, imp <- imps]
        
    runExperiments dontDisplay name dip box mdRaw ops ops putExp



runExperimentsRobust :: Bool -> String -> Double -> Maybe Double -> MotionData -> (Vec2, Vec3, [Double], [Double]) -> IO ()
runExperimentsRobust dontDisplay name dip box mdRaw (t, impDir, imps, errs) = do
    putStrLn "(impExMag errMag squareCoMerror ###)"
    let
    
        putExp  ((impExMag,errMag), (_, squareCoMerror)) = do
            -- print out the experiment results
            putStrLn $ (show impExMag) ++ " " ++ (show errMag) ++ " " ++ (show squareCoMerror) ++ " ###"
            hFlush stdout
            
        vars = [(impExMag,errMag) | impExMag <- imps, errMag <- errs]
        ops = map (\(impExMag,errMag) -> let impEx = impExMag *^ impDir in (t,impEx,impEx + (impDir ^* errMag))) vars
        
    runExperiments dontDisplay name dip box mdRaw ops vars putExp
    


main :: IO ()
main = main'

main' :: IO ()
main' = do

    initODE defaultTimeStep
    
    (dontDisplay, isTest, isRaw, file, pp, impulseType, dip, experiment, experimentRobust, box) <- getArguments
    
    if isTest
        then
            runTests
        else do
            md <-  (if isRaw then id else fmap scaleAndTranslate) $ parseBVH file pp
            maybe
                (maybe 
                    (mainLoopOut dontDisplay md impulseType dip box)
                    (runExperimentsMaxImp dontDisplay file dip box md)
                    experiment)
                (runExperimentsRobust dontDisplay file dip box md)
                experimentRobust

mainLoopOut :: Bool -> MotionData -> ImpulseType -> Double -> Maybe Double -> IO ()
mainLoopOut dontDisplay md impulseType dip box = do
    let mdv@MotionDataVars{_dt=dt,_bByName=bByName,_fN=fN,_fs=fs,_pT=pT,_l=l,_mT=mT,_zmp=zmp} = (officialFeedForwardControllerCorrections $ setImpulseType impulseType $ officialFeedForwardControllerPreprocess dip $ getMotionDataVariablesFromMotionData md)
    
    putStrLn $ "Motion Info"
    putStrLn $ "total mass: " ++ (show mT)
   -- mapM_ putStrLn [(show (dt*(fromIntegral fi))) ++ " " ++ (show pz) ++ " " ++ (show (vz $ zmp fI)) | fI@(F fi) <- (tail.init)fs, let pz = vz $ ((pT fI) + (pT (F (fi-1))) + (pT (F (fi+1)))) / 3]
    --let rhbI = bByName "RightWrist"
    --mapM_ putStrLn ["right wrist velocity (" ++ show fi ++ "/" ++ show (fN-1) ++ "): " ++ (show v) ++ (if v > 2.4 then "  *****" else "") | fI@(F fi) <-fs, let v = norm $ l fI (rhbI)]
    
    
    
    displayTIOs <-
        simulateMainLoop box mdv
        --viewAnimationLoopDefault (getMotionDataVariablesFromMotionData md)

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


