module Simulation (
    Sim(..),
    Bone(..),
    startSim,
    step,
    getSimSkel
) where

import FFI
import Util
import Linear hiding (slerp)
import Constants
import MotionData
import Data.Maybe
import Data.Array
import Data.TreeF
import Data.Color
import Data.Bone
import Control.Applicative
import Math.Tau


-- This record, Sim, represents an entire simulation. It should be created via the startSim function
data Sim = Sim {
    wid             :: DWorldID,
    -- Each body corresponds to the bone between the current joint and parent joint
    --  root has a (almost) weightless sphere as a body
    odeBodies       :: TreeF DBodyID,
    -- Each motor corresponds to a motor controlling the PARENT joint (if one exists and is not the root).
    odeMotors       :: TreeF (Maybe DJointID),
    targetMotion    :: MotionDataVars,
    simTime         :: Double,
    simTimeExtra    :: Double,
    timeDelta       :: Double
}

startSim :: MotionDataVars -> IO Sim
startSim md@MotionDataVars{_baseSkeleton=skel,_js=js,_jb=jb,_xj=xj,_xb=xb,_rj=rj,_pj=pj,_jHasParent=jHasParent} = do
    newWid <- initODE defaultTimeStep
    bodies <- realizeBodies
    moveToFrame0 bodies
    aMotors <- realizeAMotors bodies
    putStrLn $ "ode world id = " ++ (show $ newWid)
    return Sim {
        wid             = newWid,
        odeBodies       = bodies,
        odeMotors       = aMotors,
        targetMotion    = md,
        simTime         = 0,
        simTimeExtra    = 0,
        timeDelta       = defaultTimeStep }

    where
        realizeAMotors bodies = treeMapM maybeCreateAMotor bodies
        maybeCreateAMotor :: TreeF DBodyID -> IO (Maybe DJointID)
        maybeCreateAMotor jf
                    | hasParent jf && (hasParent (justParent jf))  = fmap Just (createAMotor (view jf) (view (justParent jf)))
                    | otherwise = return Nothing

        realizeBodies = do
            bodyTree <- treeMapM createBody skel
            treeMapM_ view (treeZipWith createJoint skel bodyTree)
            return bodyTree

        createBody :: JointF -> IO DBodyID
        createBody jf
            | hasParent jf = do
                let
                    (m, mIM) = massInertiaM jf
                    footStart@(V3 _ fy1 fz1) = getPosStart (if isToeJoint jf then justParent jf else jf)
                    footEnd@  (V3 _ fy2 fz2)  = getPosEnd (if isToeJoint jf then jf else fromJust (child0 jf))
                    (V3 _ _ boxZSize) = fmap abs ((getPosEnd jf) - (getPosStart jf))
                    boxYSize = let (V3 _ comY _) = getPosCom jf in 2 * (abs $ comY - fy2)
                if isFootJoint jf
                    then
                        appendFootBody
                             -- position CoM
                            (getPosCom jf)
                            -- global quaternion rotation
                            (getRot $ justParent jf)
                            -- dimensions
                            (V3 footWidth boxYSize boxZSize)
                            -- mass and Inertia matrix (about CoM)
                            m mIM
                    else
                        appendCapsuleBody
                             -- position CoM
                            (getPosCom jf)
                            -- local rotation for the bone from Z aligned to offset orientation
                            (zToDirQuat (offset $- jf))
                            -- global quaternion rotation
                            (getRot $ justParent jf)
                            -- dimensions, mass
                            boneRadius (norm (offset $- jf)) m
                            -- Inertia matrix (about CoM)
                            mIM
            | otherwise     =
                appendCapsuleBody
                     -- position CoM
                    (getPosCom jf)
                    -- local and global rotation
                    identity identity
                    -- dimensions, mass
                    boneRadius 0 0.01
                    -- Inertia matrix (about CoM)
                    (eye3 !!* 0.01)

        f0I = F 0
        moveToFrame0 :: TreeF DBodyID -> IO ()
        moveToFrame0 bidf = sequence_ $ zipWith moveBody (flatten bidf) js

        moveBody :: DBodyID -> JointIx -> IO ()
        moveBody bid jI
            | jHasParent jI  = setBodyPosRot bid (xb f0I $ jb jI) (rj f0I $ pj jI)
            | otherwise     = setBodyPosRot bid (xj f0I jI) identity

        createJoint jf bf
            | hasParent jf  = createBallJoint (view $ fromJust $ parent bf) (view bf) (getPosStart jf)
            | otherwise     = mapM_ (createFixedJoint (view bf)) (map view (getChildFs bf))


step :: Sim -> Double -> Vec2 -> Double -> IO Sim
step isim idt targetCoP yGRF = step' isim{simTimeExtra = 0} (idt + (simTimeExtra isim)) where
    ddt = timeDelta isim
    w = wid isim
    step' sim dt
        | dt >= ddt     = do
                            stepODE w targetCoP yGRF

                            -- per step instructions go here
                            matchMotionDataVars sim ddt


                            simF <- step' sim{
                                simTimeExtra = 0,
                                simTime      = (simTime sim) + ddt
                            } (dt - ddt)
                            return simF
        | otherwise     = return sim{simTimeExtra = dt}

-- Sim:     the simulation to match against
-- Double:  the time delta in which to match the data (as the siumlation is only changed via manipulating velocities)
matchMotionDataVars :: Sim -> Double -> IO ()
matchMotionDataVars sim dt = do
    (_, rootSimQuat) <- getBodyPosRot $- (odeBodies sim)
    -- TODO: calculate joint angular velocities in global coordinates relative to the simulated root orientation and apply to AMotors
    let
        MotionDataVars{
                _dt=dtF,
                _jHasParent=jHasParent,
                _js=js,
                _pj=pj,
                _rjL=rjL
            } = targetMotion sim
        t = simTime sim
        fu = (t + dt) / dtF
        u = --traceShowVM "matchMotionDataVars: u = " $
                1 - (fu - (fromIntegral fai))
        fai = floor fu
        faI = F fai
        fbi = ceiling fu
        fbI = F fbi
        setAMotor :: (DBodyID,DBodyID) -> JointIx -> Maybe DJointID -> IO ()
        setAMotor bids jI mam = maybe (return ()) (setAMotor' bids jI) mam
        setAMotor' :: (DBodyID,DBodyID) -> JointIx -> DJointID -> IO ()
        setAMotor' (bidP,bid) jI am
            | jHasParent jI && (jHasParent $ pj jI) = do
                (_,ppRotGSim) <- getBodyPosRot bidP
                (_, pRotGSim) <- getBodyPosRot bid
                let
                    simRotG     = pRotGSim
                    targetRotG  = ppRotGSim * (slerp (rjL faI $ pj jI) (rjL fbI $ pj jI) u)

                    angularVelG = toAngularVel simRotG targetRotG dt

                setAMotorVelocity am angularVelG
            | otherwise = error $ "joint doesn't have 2 parent's yet has an aMotor..... That should not be!!!"
    sequence_ $ zipWith3 setAMotor (flatten $ treeMap' (\ f -> (view (justParent f), view f)) (odeBodies sim)) js (flatten $ odeMotors sim)


getSimSkel :: Sim -> IO [Bone]
getSimSkel sim = fmap allButRoot frame where
    allButRoot = tail . flatten
    frame = tMapM getBodyBone (odeBodies sim)

