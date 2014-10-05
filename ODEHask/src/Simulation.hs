module Simulation (
    Sim(..),
    Bone(..),
    FeedBackController,
    startSim,
    step,
    getSimSkel,
    getSimJoints,
    setAMotorsToMDVJointVels
) where

import FFI
import Util
import Linear hiding (slerp,_i,_j,trace)
import Constants
import Motion
import Data.Maybe
import Data.Array
import Data.TreeF
import Data.Color
import Data.Bone
import Control.Applicative
import Math.Tau
import Debug.Trace


-- This record, Sim, represents an entire simulation. It should be created via the startSim function
data Sim = Sim {
    wid                 :: DWorldID,
    -- List of Bones and their corresponding ode body
    --  root has a (almost) weightless sphere as a body with special BoneIx = B -1 
    odeBodies           :: [(BoneIx, DBodyID)],
    -- List of parent bone, joint, child bone, and their corresponding AMotor
    odeMotors           :: [(BoneIx, JointIx, BoneIx, DBodyID, DBodyID, DJointID)],
    targetMotion        :: MotionDataVars,
    feedBackController  :: FeedBackController,
    simTime             :: Double,
    simTimeExtra        :: Double,
    timeDelta           :: Double
}
type FeedBackController = Sim -> Double -> IO (Sim)

startSim :: MotionDataVars -> IO Sim
startSim md@MotionDataVars{_xj=xjMotion,_xb=xbMotion,_rb=rbMotion,_rj=rjMotion,_baseSkeletonMDV=skel@MotionDataVars{_footBs=((leftFoot,leftToe),(rightFoot,rightToe)),_j=j,_m=m,_i=i,_jN=jN,_bN=bN,_js=js,_bs=bs,_jb=jb,_xj=xj,_xb=xb,_xsb=xsb,_xeb=xeb,_rb=rb,_pj=pj,_cjs=cjs,_pb=pb,_jHasParent=jHasParent,_jHasChild=jHasChild}} = do
    newWid <- initODE defaultTimeStep
    bodies <- realizeBodies
    aMotors <- realizeJoints bodies
    moveToFrame0 bodies
--    putStrLn $ "ode world id = " ++ (show $ newWid)
    return Sim {
            wid             = newWid,
            odeBodies       = bodies,
            odeMotors       = aMotors,
            targetMotion    = md,
            feedBackController = highGainsController,
            --feedBackController = highGainsFlatFeetController,
            simTime         = 0,
            simTimeExtra    = 0,
            timeDelta       = defaultTimeStep
        }

    where
        realizeJoints :: [(BoneIx, DBodyID)] -> IO [(BoneIx, JointIx, BoneIx, DBodyID, DBodyID, DJointID)]
        realizeJoints boneBodys = sequence $ concat $ map realizeJoint js where
            realizeJoint :: JointIx -> [IO (BoneIx, JointIx, BoneIx, DBodyID, DBodyID, DJointID)]
            realizeJoint jI
                | not (jHasChild jI) = []
                | otherwise = map realizeJointForChild cbIs where
                    -- parent bone index
                    pbI = jb jI
                    -- child bone indexes
                    cbIs = map jb (cjs jI)
                    
                    realizeJointForChild cbI = do
                        let
                            dParentBody = (fromJust $ lookup pbI boneBodys)
                            dChildBody = (fromJust $ lookup cbI boneBodys)
                        
                        -- Ball joint (unless root then fixed)
                        if jHasParent jI
                            then
                                createBallJoint dParentBody dChildBody (xj f0I jI)
                            else
                                createFixedJoint dParentBody dChildBody
                        
                        -- A Motor
                        motorID <- createAMotor dChildBody dParentBody
                        
                        return (pbI, jI, cbI, dParentBody, dChildBody, motorID)
                                

        
        realizeBodies :: IO [(BoneIx,DBodyID)]
        realizeBodies = mapM createBody js

        createBody :: JointIx -> IO (BoneIx, DBodyID)
        createBody jI@(J ji)
            -- If not the root
            | jHasParent jI = do
                let
                    mass = m bI
                    mIM = i bI
                    leftFootBs = [leftFoot,leftToe]
                    footBs = leftFootBs ++ [rightFoot,rightToe]
                    footStart@(V3 _ fy1 fz1) = (xsb fI (if elem bI leftFootBs then leftFoot else rightFoot)) + (V3 0 0 (-0.05))
                    footEnd@  (V3 _ fy2 fz2)  = xeb fI (if elem bI leftFootBs then leftToe else rightToe)
                    (V3 _ _ boxZSize) = (fmap abs ((xeb fI bI) - (xsb fI bI))) + (V3 0 0 0.05)
                    boxYSize = 2 * (abs $ (vy (xb fI bI)) - fy2)
                    
                bid <- (
                    
                    if elem bI footBs
                        then
                            -- Feet get box body
                            appendFootBody
                                 -- position CoM
                                (xb fI bI)
                                -- global quaternion rotation
                                (rb fI bI)
                                -- dimensions
                                (V3 footWidth boxYSize boxZSize)
                                -- mass and Inertia matrix (about CoM)
                                mass mIM
                        else
                            -- other bones get capsule body
                            appendCapsuleBody
                                 -- position CoM
                                (xb fI bI)
                                -- local rotation for the bone from Z aligned to offset orientation
                                (zToDirQuat (offset $ j fi ji))
                                -- global quaternion rotation
                                (rb fI bI)
                                -- dimensions, mass
                                boneRadius (norm (offset $ j fi ji)) mass
                                -- Inertia matrix (about CoM)
                                mIM
                    )
                return (bI, bid)
            | otherwise     = do 
                -- root bone gets a massless capsule (actually a sphere) body
                bid <- appendCapsuleBody
                     -- position CoM
                    (xj fI jI)
                    -- local and global rotation
                    identity identity
                    -- dimensions, mass
                    boneRadius 0.0000001 0.0000001
                    -- Inertia matrix (about CoM)
                    (eye3 !!* 0.000000001)
                return (jb jI, bid)
            where
                fi = 0
                fI = F fi
                bI = jb jI

        f0I = F 0
        moveToFrame0 :: [(BoneIx, DBodyID)] -> IO ()
        moveToFrame0 boneBodies = mapM_ moveBody boneBodies

        moveBody :: (BoneIx, DBodyID) -> IO ()
        moveBody (B (-1), body) = setBodyPosRot body (xjMotion f0I (J 0)) (rjMotion f0I (J 0))
        moveBody (bI, body)     = setBodyPosRot body (xbMotion f0I bI)    (rbMotion f0I bI)


getSimJoints :: Sim -> IO (Int -> Joint)
getSimJoints sim = do
    let
        tm@MotionDataVars{
            _j = j,
            _js=js,
            _jHasChild=jHasChild,
            _cjs=cjs,
            _pj = pj
        } = targetMotion sim
        
    posRotGs <- mapM getBodyPosRot (map snd (odeBodies sim))
    let
        posGs = map fst posRotGs
        rotGs = map snd posRotGs
        joints :: Int -> Joint
        joints 0 = (j 0 0) {
                rotationL = rotGs !! 1,
                offset = posGs !! 0
            }
        joints ji = let origJoint = (j 0 ji) in
            if jHasChild jI
                then
                    origJoint{
                        rotationL = (conjugate $ rotGs!!ji) * (rotGs!!cji)
                    }

                else
                    origJoint
                where
                    jI = (J ji)
                    (J cji) = head $ cjs jI
    return joints

step :: Sim -> Double -> Vec2 -> Double -> IO Sim
step isim idt targetCoP yGRF = step' isim{simTimeExtra = 0} (idt + (simTimeExtra isim)) where
    ddt = timeDelta isim
    w = wid isim
    -- dt  => time left to simulate
    -- ddt => time delta for each simulation step
    step' sim@Sim{odeBodies=odeBodies,targetMotion=(mdv@MotionDataVars{_dt=frameDt,_impulse=impulse})} dt
        | dt >= ddt     = do
                            -- calculate impulse for this step, should be distributed over one MDV frame
                            let (fI,_,_) = getFrameInterpVals mdv (simTime sim)
                            maybe (return ()) (\(impPoint,frameImp,impBI) -> do
                                    let
                                        frameToStepImpulseRatio = ddt / frameDt
                                        stepImpulse = frameToStepImpulseRatio *^ frameImp
                                        
                                    -- apply the impulse
                                    addImpulse (fromJust $ lookup impBI odeBodies) impPoint stepImpulse
                                ) (impulse fI)
        
                            -- step the world
                            stepODE w targetCoP yGRF
                            
                            -- feed back control
                            sim' <- (feedBackController sim) sim ddt

                            -- update the sim record and loop
                            simF <- step' sim'{
                                simTimeExtra  = 0,
                                simTime       = (simTime sim) + ddt
                            } (dt - ddt)
                            return simF
        | otherwise     = return sim{simTimeExtra = dt}


highGainsController :: FeedBackController
highGainsController sim@Sim{targetMotion=tm} dt =  setAMotorsToMDVJointVels sim tm dt

highGainsFlatFeetController :: FeedBackController
highGainsFlatFeetController sim@Sim{targetMotion=tm,odeMotors=motors} dt = do
        -- match the motion data as usual
        setAMotorsToMDVJointVels sim tm dt
        -- overwrite changes to the ankle (make the foot flat)
        mapM_ flattenFeet motors
        return sim
    where
        MotionDataVars{
            _footJs=((lajI,_,_),(rajI,_,_))
        } = tm
        flattenFeet m@(pbI, jI, cbI, _, _, am)
            | jI `elem` [lajI, rajI]  = setAMotorToAchieveTargetGlobalRot sim m identity dt
            | otherwise = return ()


-- Sim:     the simulation to match against
-- Double:  the time delta in which to match the data (as the siumlation is only changed via manipulating velocities)
setAMotorsToMDVJointVels :: Sim -> MotionDataVars -> Double -> IO (Sim)
setAMotorsToMDVJointVels sim@Sim{odeBodies=bodies,odeMotors=motors} target dt = do
    (_, rootSimQuat) <- getBodyPosRot (fromJust $ lookup (B (-1)) bodies)
    -- calculate joint angular velocities in global coordinates relative to the simulated root orientation and apply to AMotors
    let
        MotionDataVars{
                _rjL=rjL
            } = target
        (faI,fbI,u) = getFrameInterpVals target ((simTime sim)+dt)
        
        setAMotor :: (BoneIx, JointIx, BoneIx, DBodyID, DBodyID, DJointID) -> IO ()
        setAMotor m@(_,jI,_,_,_,_) = setAMotorToAchieveTargetLocalRot sim m (slerp (rjL faI jI) (rjL fbI jI) u) dt
        
    mapM_ setAMotor motors
    return sim


setAMotorToAchieveTargetLocalRot :: Sim -> (BoneIx, JointIx, BoneIx, DBodyID, DBodyID, DJointID) -> Quat -> Double -> IO ()
setAMotorToAchieveTargetLocalRot sim@Sim{odeBodies=bodies,odeMotors=motors} m@(pbI, jI, cbI, _, _, am) targetRotL dt = do
                (_,pRotGSim) <- getBodyPosRot (fromJust $ lookup pbI bodies)
                let
                    targetRotG  = pRotGSim * targetRotL
                    
                setAMotorToAchieveTargetGlobalRot sim m targetRotG dt

setAMotorToAchieveTargetGlobalRot :: Sim -> (BoneIx, JointIx, BoneIx, DBodyID, DBodyID, DJointID) -> Quat -> Double -> IO ()
setAMotorToAchieveTargetGlobalRot Sim{odeBodies=bodies} (pbI, jI, cbI, _, _, am) targetRotG dt = do
                (_,cRotGSim) <- getBodyPosRot (fromJust $ lookup cbI bodies)
                let
                    simRotG     = cRotGSim
                    angularVelG = toAngularVel simRotG targetRotG dt
                    
                setAMotorVelocity am angularVelG

getSimSkel :: Sim -> IO [Bone]
getSimSkel sim = mapM (getBodyBone . snd) (tail (odeBodies sim))

