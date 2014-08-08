module Routines where

import Simulation
import MotionData
import Constants
import Draw
import Data.Color
import Util
import FFI

import Data.Array
import Linear hiding (_j)

-- takes input motion data and returns some data
type DataInit a = MotionDataVars -> IO a
-- takes data output by last loop, performs the loop and return the data for the next iteration
type MainLoop = MotionDataVars -> IO [(Double,IO ())]
type TimedLoop a = a -> Double -> Double -> IO (Maybe (a, IO ()))




-- return an updated sim and display IO
viewAnimationLoop :: MainLoop
viewAnimationLoop mdv@MotionDataVars{_dt=dt,_fs=fs} = return $ zip (repeat dt) (map display fs) where
    display fI = drawFrameIx (BlackA 0.5) zero mdv fI



coMFeedBackLoopExperiment :: [Double] -> [Double] -> MainLoop
coMFeedBackLoopExperiment kcs kps targetRaw@MotionDataVars{_fs=fs,_fN=fN,_sp=spRaw,_dt=dt} = (fmap head) $ sequence [coMFeedBackLoopRaw kc kp target | kc <- kcs, kp <-kps] where

        bnds = (0,fN-1)
        centerOfSp = [let poly = (spRaw fI) in sum poly ^/ (fromIntegral $ length poly)  | fI <- fs]
        targetZmp = listArray bnds $
            -- target zmp as center of SP
            map xz2x0z centerOfSp
            -- target zmp as origional zmp projeted onto SP
            -- zipWith (\ proj zmp -> if null proj then zmp else xz2x0z . head $ proj) [polyEdgeLineSegIntersect (sp fI) (toXZ (zmp fI), spC) | (fI,spC) <- zip fs centerOfSp] (map zmp fs)
        target = fitMottionDataToZmp targetRaw targetZmp 0 10

-- setting kc to the data's framerate (usually 60Hz) and keeping kp=0 the origional target motion
-- will be retreived.
coMFeedBackLoopRaw :: Double -> Double -> MainLoop
coMFeedBackLoopRaw kc kp target@MotionDataVars{_fs=fs,_fN=fN,_sp=spRaw,_dt=dt,_js=js,_j=jT,_com=comT,_zmp=zmpT} = do


    -- the sim target motion starts as the first frame of the true target motion. In each iteration:
    --   The current simulated state is used to replace the current targetMotion frame (fi)
    --   The targetMotion gets appended with a new frame (fiNext) generated from the feedback controller (this frame has IK applied to it).
    --   The simulator is steped one frameBoundingBox. in the step method
    --      The AMotors are set to reach the pose of the generated frame (fiNext)
    --   repeat
    simI <- startSim (modifyMotionDataVarsFN target 1 [])
    coMFeedBackLoop' simI 0 where

        -- @params (simulation) (time)
        coMFeedBackLoop' sim fi = do
            let

                t = fromIntegral fi * dt

                fI = F fi
                fiNext = fi+1
                fINext = F fiNext


            -- get the state and update the mdvActual
            jointsActual <- getSimJoints sim
            let
                mdvA@MotionDataVars{_j=jA,_com=comA,_zmp=zmpA} = modifyMotionDataVars (targetMotion sim) [((fi,ji), jointsActual ji) | J ji <- js]

            if fi == fN-1
                then do
                    let
                        -- square distance of ZMP of the controlOutput to the targetMotion
                        errCoM = sum $ map norm2 $ [(comT fI) - (comA fI) | fI <- fs]
                        errZMP = sum $ map norm2 $ [(zmpT fI) - (zmpA fI) | fI <- fs]

                    putStrLn $ "kc kp errCoM errZMP: " ++ show kc ++ "\t" ++ show kp ++ "\t" ++ show errCoM ++ "\t" ++ show errZMP
                    return []
                else do
                    -- for each frame

                        -- Create draw IO
                        cSimFrame <- getSimSkel sim
                        floorCOntacts <- fmap (map xz2x0z) getFloorContacts
                        cop <- getCoP
                        let
                            displayIO = do
                                -- draw sim with contact points
                                drawSkeleton (RedA 0.5) cSimFrame
                                mapM_ ((flip (drawPointC Black)) 0.01) floorCOntacts
                                --   CoP
                                drawPointC Green cop 0.03
                                -- draw target
                                drawFrameIx (YellowA 0.3) zero (updatedMdvA) fINext
                                -- draw target
                                drawFrameIx (BlueA 0.3) zero target fI


                        --
                        -- do feedback control, updating the target motion
                        --

                            -- errors (ZMP and CoM) comparing frame fI in target and actual
                            ep = (zmpT fI) - (zmpA fI)
                            ec = (comT fI) - (comA fI)

                            -- current desired CoM velocity (use current and next frame only)
                            lT = ((comT fINext) - (comT fI)) ^/ dt

                            -- adjusted CoM velocity (based on equation (44))
                            ld = lT - (kp *^ ep) + (kc *^ ec)

                            -- now we adjust the next frame (fi + 1) to reflect the adjusted CoM velocity (ld')
                            -- in this case we consider only frames fi and fi+1 when considering velocity. This
                            -- means we only calculate the offset over 1 frame given the velocity and apply that
                            -- to frame fi+1 (we don't consider a 3rd frame as is done in finite difference)
                            --
                            -- here we convert shift in CoM to shift in Root node (they are not the same thing)
                            rootJoint = (jA fi 0)
                            nextFrameRootJoint = (jT fiNext 0)
                            targetCoM = (comA fI) + (ld ^* dt)
                            --     here is the resulting offset
        --                    nextOffset = offset nextFrameRootJoint
                            nextOffset = targetCoM + ((offset nextFrameRootJoint) - (comT fINext))

                            -- modify fiNext frame according to feedback controller
                            feedbackOffestMdvA = modifyMotionDataVarsFN mdvA (fi+2) [((fiNext,0), nextFrameRootJoint{
                                    offset = nextOffset
                                })]
                            updatedMdvA =
                                -- use foot IK to get feet to the true target positions
                                copyFrames [fiNext] (correctFeet target feedbackOffestMdvA) feedbackOffestMdvA


                        -- simulate
        --                putStrLn $ "Simulating (with feadback control) " ++ show fi ++ " / " ++ show fN
                        sim' <- step sim{targetMotion = updatedMdvA} dt zero 0

                        -- return
                        rest <- coMFeedBackLoop' sim' (fi + 1)
                        return $ (dt, displayIO) : rest








{-

timedLoopToMainLoop :: TimedLoop a -> a -> MainLoop a
timedLoopToMainLoop tl a = return $ timedLoopToMainLoop' 0 a where
    timedLoopToMainLoop' t d = do
            let t' = t+displayFDT
            m <- tl d t' displayFDT
            maybe (return []) (\(d',io) -> do
                    rest <- timedLoopToMainLoop' t d'
                    return ((displayFDT,io) : rest)
                ) m



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
defaultMainLoop = fmap
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



testCoMControllerLoop :: MainLoop
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
-}
