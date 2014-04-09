module Simulation (
    Sim(..),
    Bone,
    startSim,
    step,
    getSimSkel
) where

import FFI
import Util
import Linear
import Constants
import MotionData
import Data.Maybe
import Data.TreeF
import Data.Color
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
    targetMotion    :: MotionData,
    simTime         :: Double,
    simTimeExtra    :: Double,
    timeDelta       :: Double
}

startSim :: MotionData -> IO Sim
startSim md = do
    newWid <- initODE
    bodies <- realizeBodies
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
        skel = frames md !! 0
        realizeAMotors bodies = treeMapM maybeCreateAMotor bodies
        maybeCreateAMotor :: TreeF DBodyID -> IO (Maybe DJointID)
        maybeCreateAMotor jf
                    | hasParent jf && (hasParent (justParent jf))  = fmap Just (createAMotor (view jf) (view (justParent jf)))
                    | otherwise  = return Nothing
                            
        realizeBodies = do
            bodyTree <- treeMapM createBody skel
            treeMapM_ view (treeZipWith createJoint skel bodyTree)
            return bodyTree
            
        createBody :: JointF -> IO DBodyID
        createBody jf
            | hasParent jf = do
                let (m, mIM) = massInertiaM jf
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

        createJoint jf bf
            | hasParent jf  = createBallJoint (view $ fromJust $ parent bf) (view bf) (getPosStart jf)
            | otherwise     = mapM_ (createFixedJoint (view bf)) (map view (getChildFs bf))
                                    

step :: Sim -> Double -> IO Sim
step isim idt = step' isim{simTimeExtra = 0} (idt + (simTimeExtra isim)) where
    ddt = timeDelta isim
    w = wid isim
    step' sim dt
        | dt >= ddt     = do
                            stepODE w ddt
                            
                            -- per step instructions go here
                            matchMotionData sim ddt
                            
                            
                            simF <- step' sim{
                                simTimeExtra = 0,
                                simTime      = (simTime sim) + ddt
                            } (dt - ddt)
                            return simF
        | otherwise     = return sim{simTimeExtra = dt}
        
matchMotionData :: Sim -> Double -> IO ()
matchMotionData sim dt = do
    (_, rootSimQuat) <- getBodyPosRot $- (odeBodies sim)
    -- TODO: calculate joint angular velocities in global coordinates relative to the simulated root orientation and apply to AMotors
    let
        md = targetMotion sim
        frame = getInterpolatedFrame (simTime sim) md
        setAMotor :: TreeF (DBodyID, Joint) -> TreeF (Maybe DJointID) -> IO ()
        setAMotor bidFjF mamF = maybe (return ()) (setAMotor' bidF jF) (view mamF)
           where
            bidF = fmap fst bidFjF
            jF   = fmap snd bidFjF
        setAMotor' :: TreeF DBodyID -> JointF -> DJointID -> IO ()
        setAMotor' bidF jF am
            | hasParent jF && (hasParent $ justParent jF) = do
                (_,ppRotGSim) <- getBodyPosRot $- (justParent bidF)
                (_, pRotGSim) <- getBodyPosRot $- bidF
                let
                    pjF = justParent jF

                    simRotG     = pRotGSim
                    targetRotG  = ppRotGSim * (rotationL $- pjF)
                    
                    angularVelG = toAngularVel simRotG targetRotG dt
                
                setAMotorVelocity am angularVelG
            | otherwise = error $ "jF doesn't have 2 parent's..... That doesn't make sense!!!"
    treeSequence_ $ treeZipWith setAMotor (treeZip (odeBodies sim) frame) (odeMotors sim)
--    tMapM_ (maybe (return ()) (\jid -> setAMotorVelocity jid (V3 0 0 (2*pi)))) (odeMotors sim)
    

type Bone = (Vec3, Vec3) -- start and end in global coordinates 
getSimSkel :: Sim -> IO [Bone]
getSimSkel sim = fmap allButRoot frame where
    allButRoot = tail . flatten
    frame = tMapM getBodyStartEnd (odeBodies sim)
