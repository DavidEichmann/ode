module Simulation (
    Sim(..),
    Bone(..),
    FeedBackController,
    startSim,
    step,
    getSimSkel,
    getSimJoints
) where

import FFI
import Util
import Linear hiding (slerp, _j,trace)
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
    odeMotors           :: [(BoneIx, JointIx, BoneIx, DJointID)],
    targetMotion        :: MotionDataVars,
    feedBackController  :: FeedBackController,
    simTime             :: Double,
    simTimeExtra        :: Double,
    timeDelta           :: Double
}
type FeedBackController = Sim -> Double -> IO (Sim)

startSim :: MotionDataVars -> IO Sim
startSim md@MotionDataVars{_baseSkeleton=skel,_jN=jN,_bN=bN,_js=js,_bs=bs,_jb=jb,_xj=xj,_xb=xb,_rb=rb,_pj=pj,_cjs=cjs,_pb=pb,_jHasParent=jHasParent,_jHasChild=jHasChild} = do
    newWid <- initODE defaultTimeStep
    bodies <- realizeBodies
    aMotors <- realizeAMotors bodies
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
        realizeAMotors :: [(BoneIx, DBodyID)] -> IO [(BoneIx, JointIx, BoneIx, DJointID)]
        realizeAMotors boneBody = sequence $ concat $ map createAMotorForJoint js where
            createAMotorForJoint :: JointIx -> [IO (BoneIx, JointIx, BoneIx, DJointID)]
            createAMotorForJoint jI
                | not (jHasParent jI && jHasChild jI) = []
                | otherwise = map createAMotorForChild cbIs where
                    -- parent bone index
                    pbI = jb jI
                    -- child bone indexes
                    cbIs = map jb (cjs jI)
                    
                    createAMotorForChild cbI = do
                        motorID <- createAMotor
                                    (fromJust $ lookup cbI boneBody)
                                    (fromJust $ lookup pbI boneBody)
                        return (pbI, jI, cbI, motorID)
                                

        
        realizeBodies = do
            bodyTree <- treeMapM createBody skel
            treeMapM_ view (treeZipWith createJoint skel bodyTree)
            -- Note that MotionDataVars does not include a root bone, so its index is added here as (B -1) 
            return $ zip ((B (-1)) : bs) (flatten bodyTree)

        createBody :: JointF -> IO DBodyID
        createBody jf
            -- If not the root
            | hasParent jf = do
                let
                    (m, mIM) = massInertiaM jf
                    footStart@(V3 _ fy1 fz1) = getPosStart (if isToeJoint jf then justParent jf else jf)
                    footEnd@  (V3 _ fy2 fz2)  = getPosEnd (if isToeJoint jf then jf else fromJust (child0 jf))
                    (V3 _ _ boxZSize) = fmap abs ((getPosEnd jf) - (getPosStart jf))
                    boxYSize = let (V3 _ comY _) = getPosCom jf in 2 * (abs $ comY - fy2)
                if isFootJoint jf
                    then
                        -- Feet get box body
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
                        -- other bones get capsule body
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
                -- root bone gets a massless capsule body
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
        moveToFrame0 :: [(BoneIx, DBodyID)] -> IO ()
        moveToFrame0 boneBodies = mapM_ moveBody boneBodies

        moveBody :: (BoneIx, DBodyID) -> IO ()
        moveBody (B (-1), body) = setBodyPosRot body (xj f0I (J 0)) identity
        moveBody (bI, body)     = setBodyPosRot body (xb f0I bI)    (rb f0I bI)

        createJoint jf bf
            | hasParent jf  = createBallJoint (view $ fromJust $ parent bf) (view bf) (getPosStart jf)
            | otherwise     = mapM_ (createFixedJoint (view bf)) (map view (getChildFs bf))


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
                rotationL = rotGs !! 0,
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
    step' sim dt
        | dt >= ddt     = do
                            stepODE w targetCoP yGRF
                            
                            (feedBackController sim) sim ddt

                            simF <- step' sim{
                                simTimeExtra  = 0,
                                simTime       = (simTime sim) + ddt
                            } (dt - ddt)
                            return simF
        | otherwise     = return sim{simTimeExtra = dt}


highGainsController :: FeedBackController
highGainsController sim@Sim{targetMotion=tm} dt =  matchMotionDataVars sim tm dt

highGainsFlatFeetController :: FeedBackController
highGainsFlatFeetController sim@Sim{targetMotion=tm,odeMotors=motors} dt = do
        -- match the motion data as usual
        matchMotionDataVars sim tm dt
        -- overwrite changes to the ankle (make the foot flat)
        mapM_ flattenFeet motors
        return sim
    where
        MotionDataVars{
            _footJs=((lajI,_,_),(rajI,_,_))
        } = tm
        flattenFeet m@(pbI, jI, cbI, am)
            | jI `elem` [lajI, rajI]  = setAMotorToAchieveTargetGlobalRot sim m identity dt
            | otherwise = return ()


-- Sim:     the simulation to match against
-- Double:  the time delta in which to match the data (as the siumlation is only changed via manipulating velocities)
matchMotionDataVars :: Sim -> MotionDataVars -> Double -> IO (Sim)
matchMotionDataVars sim@Sim{odeBodies=bodies,odeMotors=motors} target dt = do
    (_, rootSimQuat) <- getBodyPosRot (fromJust $ lookup (B (-1)) bodies)
    -- calculate joint angular velocities in global coordinates relative to the simulated root orientation and apply to AMotors
    let
        MotionDataVars{
                _rjL=rjL
            } = target
        (faI,fbI,u) = getFrameInterpVals target ((simTime sim)+dt)
        
        setAMotor :: (BoneIx, JointIx, BoneIx, DJointID) -> IO ()
        setAMotor m@(_,jI,_,_) = setAMotorToAchieveTargetLocalRot sim m (slerp (rjL faI jI) (rjL fbI jI) u) dt
        
    mapM_ setAMotor motors
    return sim


setAMotorToAchieveTargetLocalRot :: Sim -> (BoneIx, JointIx, BoneIx, DJointID) -> Quat -> Double -> IO ()
setAMotorToAchieveTargetLocalRot sim@Sim{odeBodies=bodies,odeMotors=motors} m@(pbI, jI, cbI, am) targetRotL dt = do
                (_,pRotGSim) <- getBodyPosRot (fromJust $ lookup pbI bodies)
                let
                    targetRotG  = pRotGSim * targetRotL
                    
                setAMotorToAchieveTargetGlobalRot sim m targetRotG dt

setAMotorToAchieveTargetGlobalRot :: Sim -> (BoneIx, JointIx, BoneIx, DJointID) -> Quat -> Double -> IO ()
setAMotorToAchieveTargetGlobalRot Sim{odeBodies=bodies} (pbI, jI, cbI, am) targetRotG dt = do
                (_,cRotGSim) <- getBodyPosRot (fromJust $ lookup cbI bodies)
                let
                    simRotG     = cRotGSim
                    angularVelG = toAngularVel simRotG targetRotG dt
                    
                setAMotorVelocity am angularVelG

getSimSkel :: Sim -> IO [Bone]
getSimSkel sim = mapM (getBodyBone . snd) (tail (odeBodies sim))

