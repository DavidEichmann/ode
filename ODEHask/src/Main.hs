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

--    main'

main' :: IO ()
main' = do

    (file, pp) <- getArguments

    md <- fmap (scaleAndTranslate 2 0) $ parseBVH file pp
--    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/david-1-martialarts-025_David.bvh" pp
--    md <- fmap (scaleAndTranslate 2 10) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/david-1-martialarts-011_David.bvh" pp

--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/testDrop.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test_sin.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test_sinZ.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test_spin.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test_spin_acc.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test_spinZ_acc.bvh" pp
--    md <- fmap (scaleAndTranslate 2 0) $ parseBVH "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/test_arm_spin.bvh" pp

    mainLoopOut md


(dataInit, mainLoop) =

-- view animation
--    (return, viewAnimationLoop)

-- Default
    (defaultDataInit, defaultMainLoop)

-- test CoM controller
--    (testCoMControllerInit, testCoMControllerLoop)

-- takes input motion data and returns some data
type DataInit a = MotionDataVars -> IO a
-- takes data output by last loop, performs the loop and return the data for the next iteration
type MainLoop a = Double -> Double -> a -> IO (Maybe (a, IO ()))

mainLoopOut :: MotionData -> IO ()
mainLoopOut md = do
    let
        loopSimulate :: Double -> a -> MainLoop a -> IO [IO ()]
        loopSimulate t' d loopFn = do
            let t = t'+displayFDT
            m <- loopFn t displayFDT d
            maybe (return []) (\(d',io) -> do
                    rest <- loopSimulate t d' loopFn
                    return (io : rest)
                ) m


        loopDisplay :: [IO ()] -> UTCTime -> IO ()
        loopDisplay ios ti = do
            tc <- getCurrentTime
            let
                t = (realToFrac $ diffUTCTime tc ti) * playbackSpeed
                fi = (floor (t / displayFDT)) `mod` (length ios)
            _ <- (ios!!fi)
            notDone <- doRender
            loopDisplay ios ti

    d <- dataInit (getMotionDataVariablesFromMotionData md)

    displayIOs <- loopSimulate 0 d mainLoop
    initOgre
    ti <- getCurrentTime
    loopDisplay displayIOs ti

    return ()


defaultDataInit :: DataInit (Sim, MotionDataVars, MotionDataVars, Array Int Vec3)
defaultDataInit mdv = do
    let
        (mdvActual@MotionDataVars{_fN=fN,_fs=fs,_js=js,_j=j,_pT=pT,_l=l,_com=com,_zmp=zmp,_sp=sp,_zmpIsInSp=zmpIsInSp}) = mdv
        -- blend test
        --      blendIntoMotionData :: (Int -> Joint) -> MotionDataVars -> MotionDataVars
--        blended = blendIntoMotionData 1 (j 200) mdvActual


        comI  =  ((com (F 0)) * (V3 1 0 1)) + (V3 (-0.1) 0 0) --zmp (F 0)
        comIF = (((com (F 0)) * (V3 1 0 1)) + (V3 (0.1) 0 0)) -comI --(zmp (F (fN-1))) - comI
        bnds = (0,fN-1)
--        targetZmp = array bnds [(i, let t = ((fromIntegral i) / (fromIntegral (fN-1))) in comI + ((fromIntegral (round t))*^comIF) )  | i <- range bnds]
        centerOfSp = [let poly = (sp fI) in sum poly ^/ (fromIntegral $ length poly)  | fI <- fs]
        targetZmp = listArray bnds $
            -- target zmp as center of SP
            map xz2x0z centerOfSp
            -- target zmp as origional zmp projeted onto SP
            -- zipWith (\ proj zmp -> if null proj then zmp else xz2x0z . head $ proj) [polyEdgeLineSegIntersect (sp fI) (toXZ (zmp fI), spC) | (fI,spC) <- zip fs centerOfSp] (map zmp fs)


        mdvMod@MotionDataVars{_zmp=zmpMod,_fs=fsMod,_pT=pTMod} = fitMottionDataToZmp (shift (V3 0 0 0) mdvActual) targetZmp 0 0

    sim <- startSim mdvMod
    return (sim, mdv, mdvMod, targetZmp)

-- return an updated sim and display IO
defaultMainLoop :: MainLoop (Sim, MotionDataVars, MotionDataVars, Array Int Vec3)
defaultMainLoop
  t
  simdt
  (sim,
  mvdOrig@MotionDataVars{_zmpIsInSp=aniZmpIsInSp,_com=targetZmp'@aniCom,_zmp=aniZmp,_sp=aniSp},
  mdv@MotionDataVars{_zmpIsInSp=zmpIsInSp,_dt=dt,_fs=fs,_fN=fN,_bs=bs,_m=m,_l=l,_zmp=zmp},
  targetZmp) = do
    let
        maxFIndex = fN - 1
        frameix = 1 + ((floor $ t / dt))
        frameIx = F frameix

        yGRF = sum [(m b) * ((vy $ l frameIx b) + gravityAcc) | b <- bs]

    putStrLn $ "Simulating (" ++ (show t) ++ "/" ++ (show $ realToFrac fN * dt) ++")"
    if frameix > maxFIndex then return Nothing else do
        sim' <- step sim simdt (toXZ $ targetZmp!frameix) yGRF

        floorCOntacts <- getFloorContacts
        cSimFrame <- getSimSkel sim'

        let displayIO = do
            -- sim visualization
            drawSkeleton $ cSimFrame
            --mapM_ (\p -> drawPointC Yellow p 0.02) (map xz2x0z floorCOntacts)

            -- Annimation
            let aniOffset =   zero --V3 (-2) 0 0
        --    drawFrameIx (WhiteA 0.5) aniOffset mdv frameIx
            --drawFrameIx (BlackA 0.5) aniOffset mvdOrig frameIx
            drawFrameIx (YellowA 0.5) aniOffset (targetMotion sim) frameIx

            -- SP
        --    drawPolygonC (OrangeA 0.3) (map (\(V2 x z) -> V3 x 0 z) (aniSp frameIx))
            drawPolygonEdgesC Black (map (\(V2 x z) -> V3 x 0 z) (aniSp frameIx))
            --mapM_ ((\p -> drawPointC Blue p 0.02) . (\(V2 x z) ->V3 x 0 z)) (polyEdgeLineSegIntersect (aniSp frameIx) (toXZ (aniZmp frameIx), toXZ (aniCom frameIx)))

            -- ZMP
            --putStrLn $ "target zmp: " ++ (show $ targetZmp!(frameix))
            --putStrLn $ "actual zmp: " ++ (show (zmp frameIx))
            --drawPointC (Black) (aniZmp frameIx) 0.02

        --    drawPointC (Red) (zmp frameIx) 0.02
        --    drawPointC (WhiteA 0.4) (targetZmp!(frameix)) 0.04

        --    print $ (zmp frameIx) - (targetZmp!(frameix))
        --    drawPointC (if zmpIsInSp frameIx then White else Red) (aniOffset + (zmp frameIx)) 0.02
        --    drawPointC (if aniZmpIsInSp frameIx then (WhiteA 0.5) else (OrangeA 0.5)) (aniOffset + (aniZmp frameIx)) 0.04
        --    drawPointC Red (aniOffset + (targetZmp frameIx)) 0.04
            -- COM
            --drawPointC Yellow ((* (V3 1 0 1)) $ aniCom frameIx) 0.02


        --threadDelay $ floor $ (frameTime md) * 1000000
    --    if notDone
    --        then mainLoop sim (t+0.03)
    --        else return ()
        return $ Just ((sim',
                  mvdOrig,
                  mdv,
                  targetZmp), displayIO)

-- return an updated sim and display IO
viewAnimationLoop :: MainLoop MotionDataVars
viewAnimationLoop
  t
  simdt
  mdv@MotionDataVars{_dt=dt,_fN=fN} = do
    let
        maxFIndex = fN - 1
        frameix = 1 + ((floor $ t / dt))
        frameIx = F frameix

    if frameix > maxFIndex then return Nothing else return $ Just (mdv, do
            -- Annimation
            drawFrameIx (BlackA 0.5) zero mdv frameIx
        )



-- Test how well the applyCoMVelFeedbackControler function moves noisy data back to the original data
-- score is squared distance of the new motions ZMP to the origional ZMP
testCoMControllerInit :: MotionDataVars -> IO (MotionDataVars,MotionDataVars)
testCoMControllerInit targetMotion@MotionDataVars{_dt=dt,_fN=fN,_fs=fs,_com=comTarget,_zmp=zmpTarget} = do
    let
        resultTable :: [[Double]]
        resultTable = [ let (_,(errCoM,errZMP)) = runTest kc kp in [kc, kp, errCoM, errZMP] |  kp <- [0,0.0005..0.009], kc <- [0,5..120]]

        -- create noisy data (this is the error of the CoM)
        --rands = map (\(d,a) -> d *^ (V3 (sin a) 0 (cos a))) (zip (randomRs (0,0.1) (mkStdGen 1045902832)) (randomRs (0,tau) (mkStdGen 498205208)))
        rands = repeat (V3 0.00001 0 0)
        shiftError = listArray (0,fN-1) (zero : zero : (take (fN-3) rands) ++ [zero])

        runTest :: Double -> Double -> (MotionDataVars, (Double,Double))
        runTest kc kp = (controlOutput,(errCoM,errZMP))where

            -- apply the applyCoMVelFeedbackControler function
            controlOutput@MotionDataVars{_com=comOut,_zmp=zmpOut} = applyCoMVelFeedbackControler kc kp targetMotion shiftError

            -- square distance of ZMP of the controlOutput to the targetMotion
            errCoM = sum $ map norm2 $ [(comTarget fI) - (comOut fI) | fI <- fs]
            errZMP = sum $ map norm2 $ [(zmpTarget fI) - (zmpOut fI) | fI <- fs]

    putStrLn "# kc kp ErrCoM ErrZMP"
    putStrLn $ intercalate "\n" $ map ((intercalate "\t") . (map show)) resultTable
    return (targetMotion, fst $ runTest 0 0.01)



testCoMControllerLoop :: MainLoop (MotionDataVars,MotionDataVars)
testCoMControllerLoop
  t
  _
  d@(
    target@MotionDataVars{            _com=comT,_zmp=zmpT},
    out@MotionDataVars{_dt=dt,_fN=fN, _com=comO,_zmp=zmpO}
  ) =
            if (fromIntegral fN) * dt <= t then return Nothing else do
                let fI = F (floor (t/dt))
                return (Just (d, do

                    drawFrameIx (BlackA 0.5) zero target fI
                    drawPointC  (BlackA 0.5) (zmpT fI) 0.02

                    drawFrameIx (YellowA 0.5) zero out fI
                    drawPointC  (YellowA 0.5) (zmpO fI) 0.02
                 ))





--
-- Drawing Functions
--


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
    color = RedA 0.2
    drawBone' :: Bone -> IO ()
    drawBone' (Long start end) = drawBoneC color start end boneRadiusDisplay
    drawBone' (Box lengths center rot) = drawBoxC color lengths center rot



testF = toFocus (Tree nullJoint{name="root"} [Tree nullJoint{name="c1"} [Tree nullJoint{name="c11"} [],Tree nullJoint{name="c12"} []]])


