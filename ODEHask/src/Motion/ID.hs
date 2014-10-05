{-
  This code is based off of Roy Featherstone and David E. Orin book part A chapter 2 "Dynamics"
-}

module Motion.ID (
    inverseDynamics,
    unitTestsID,
    inverseDynamicsInternals
) where

import Prelude hiding (length,concat,map,foldl,zip,take,drop,foldr,replicate, (++))
import qualified Data.List as L
import Data.List.Split
import qualified Data.Vector as V
import Data.Vector hiding (and,sequence_)
import Linear hiding (_w,_j,_i,cross,trace)
import qualified Linear as L 
import Util
import Motion.MotionDataVars
import Motion.Joint
import Debug.Trace
import Constants hiding (gravityAcc)
import qualified Constants as Constants
import Test.HUnit
import Test.HUnitX
import Test.TestData
import Control.Applicative
import FFI


-- (2.2) Plueker forces and motion. angular then linear

type P = Vector Double
type MotionV = P
type ForceV = P

zeroP = replicate 6 0

-- identity matrix
m0 ::  Int -> Int -> Matrix
m0 r c = replicate r (replicate c 0)
mid :: Int -> Int -> Matrix
mid r c = generate r (\r -> generate c (\c -> if r == c then 1 else 0))

split6 = V.splitAt 3


-- (2.2.7)
-- 3x3 cross product matrix
s3 :: Num a => Vector a -> Vector (Vector a)
s3 v = fromList2 [
            [    0,     -(v!2),     v!1   ],
            [   v!2,      0,      -(v!0)  ],
            [ -(v!1),    v!0,        0    ]
        ]
{-
smm :: P -> Matrix
smm vm = fromList2Vector2 [[s3m,    zero33],
                           [s3 mo,  s3m]] where
                                            (m,mo) = split vm
                                            s3m = s3 m

smf :: P -> Matrix
smf vm = (vectorT $ smm vm) !!* (-1)

    -}
mxf :: MotionV -> ForceV -> ForceV
vm `mxf` vf = ((m `cross` fo) ^+^ (mo `cross` f)) ++ (m `cross`f) where
    (m,mo) = split6 vm
    (fo,f) = split6 vf
    
    
mxm :: MotionV -> MotionV -> MotionV
vm1 `mxm` vm2 = (m1 `cross` m2) ++ ((m1 `cross` m2o) ^+^ (m1o `cross` m2)) where
    (m1,m1o) = split6 vm1
    (m2,m2o) = split6 vm2
    
-- Inverse Dynamics algorithm. as described by table 2.6
-- some differences: use B(-1) for the base body (the floor)
-- input a motion data and frame index, then output Joint indexed function of joint torques 
inverseDynamics :: MotionDataVars -> FrameIx -> (JointIx -> Vec3, JointIx -> Vec3)
inverseDynamics mdv fI = fst $ inverseDynamicsInternals True Constants.gravityAcc mdv fI

--inverseDynamicsInternals :: Bool -> MotionDataVars -> FrameIx -> ((JointIx -> Vec3), a)
inverseDynamicsInternals useGRF gravityAcc md fI@(F fi) = -- trace (L.intercalate "\n" $ fmap show [("comGRF",show comGRF),("fc'",show fc'),("contactPoints",show contactPoints)])
 (
    
        ((\v -> V3 (v!0) (v!1) (v!2)) . jointTorques, (\v -> V3 (v!3) (v!4) (v!5)) . jointTorques),
        
        (axb,(xfOj,(ixpi,(inert,(ixpiF,(v,(a,(f',(fNet,(fOe,()))))))))))
        
 ) where
    -- motion data
    MotionDataVars{
        _j=jf,
        _bN=bN,
        _jN=jN,
        _bs=bs,
        _bj=bje,
        _jb=ejb,
        _pj=pj_,
        _pb=pb_,
        _xb=xbf,
        _xj=xjf,
        _rjL=rjLf,
        _rj=rjf,
        _cjs=cjs_,
        _jHasChild=jHasChild_,
        _q'=q'f,
        _q''=q''f,
        _m=massB,
        _mT=mT,
        _l'T=l'TF,
        _i=inertCB,
        _com=comF,
        _w=wF,
        _h=hF,
        _h'=h'F,
        _footBs=footBs,
        _footBCorners=footBCornersF
    } = md
    
    -- apply frame index to some of these functions
    com = v32Vector $ comF fI
    w bI = v32Vector $ wF fI bI
    h bI = v32Vector $ hF fI bI
    h' bI = v32Vector $ h'F fI bI
    l'T = v32Vector $ l'TF fI
    j=jf fi
    cjs = cjs_
    xb = v32Vector . (xbf fI)
    xj = v32Vector . (xjf fI)
    rjL = (rjLf fI)
    rj = (rjf fI)
    q'  = (\(a,l) -> (v32Vector a) ++ (v32Vector l)) . (q'f  fI)
    q'' = (\(a,l) -> (v32Vector a) ++ (v32Vector l)) . (q''f fI)
    footBCorners :: [[P]]
    footBCorners = let ((lf,lt),(rf,rt)) = footBCornersF fI in (fmap . fmap) v32Vector [lf,lt,rf,rt]
    
    jHasChild = jHasChild_
    mass bi = massB bi
    inertC bi = inertCB bi
    
    -- modification of bj to convert to the bone's starting joint instead of end joint (this fits the theory better)
    bjp :: BoneIx -> JointIx
    bjp = pj_ . bje
    p :: BoneIx -> BoneIx
    p = pb_
    
    
    
    --------------------------------------
    -- Calculate Ground Reaction Forces --
    --------------------------------------
    
    -- Net Spatial force on CoM
    comFnet :: ForceV
    comFnet =   -- angular
                ((com `cross` (mT *^ l'T)) ^+^ (sumV [  ((w bI) `cross` (h bI)) ^+^ (h' bI)  | bI <- bs  ])) ++
                -- linear
                (mT *^ l'T)
    
    -- Net GRF = Net force - Gravity force
    comGRF =
        -- ensure the GRF is in the positive y direction (else the feet would be "sticking to the floor" and the CoM would accelerate fater than gravity)
        (\f -> if f!4 < 0 then trace "Negative GRF!!!" (f//[(4,0)]) else f)
        -- Net GRF = Net force - Gravity force
        (comFnet ^-^ (fromList [0,0,0, 0,-(mT * gravityAcc),0]))
    
    -- Distribute forces to contact points
    
    
    --   Find the contact points
    filterForContacts = L.filter (\v -> v!1 <= floorContactThreshold)
    contactPointsBs = fmap filterForContacts footBCorners
    contactPoints = L.concat contactPointsBs
    --   build constrained linear programming problem
    --     This is in the form of:   arg min x (c dot x)   where M x = f
    setXZTorque0 matrix66 = map (\v -> (singleton (v!1)) ++ (drop 3 v)) matrix66 
    lpMatrix = L.foldl1 (|||) [ setXZTorque0 (axbF (com ^-^ p) identity) | p <- contactPoints ]
    
    {-  This is in an attempt to add constraints like force must be away from floor, but maybe a simpler method (psudo inverse) is sufficient 
    constraintIxs = [ (5+rC, (4*rC)-2) | rC <- [1..cN]]
    lpMatrixNZ =
        -- non zero lpMatrix values
        [ ((r,c), v) | r <- [0..5], c <- [0..(4*cN)-1], let v = lpMatrix!r!c, v /= 0 ]
    -- Y forces for each contact force >= 0 (these are GLPK "column constraints")
    collumnLo = [  |  <- [] ]
    -- solve for contact point forces (each force is relative to the contact position)
    fcs = linearProgramOpt Minimize lpMatrixNZ constraintsLower []
    
    -}
    
    --    using psudo-inverse instead of LP (simplex/constrained lp/GLPK) method
    fc' :: P
    fc' = psudoInverseMult lpMatrix comGRF
    --    extract the forces
    fc :: [P]
    fc = let zeroV1 = singleton 0 in
            -- clip y forces to be >= 0
            --fmap (\f -> if f!4 < 0 then f//[(4,0)] else f) $
            -- add back in the x and y torques = 0
            fmap (\f -> zeroV1 ++ (fromList $ L.take 1 f) ++ zeroV1 ++ (fromList $ L.drop 1 f)) (chunksOf 4 $ toList fc')
    -- convert to global frame
    f0c :: [P]
    f0c = fmap (\(c, f) -> let oxc = axbF ((-1) *^ c) identity in oxc !* f) (L.zip contactPoints fc)
    -- split the feet, calculating net GRF on each foot
    sumF = L.foldl (^+^) zeroP
    boneForce0Lookup =
        L.zip (let ((lf,lt),(rf,rt)) = footBs in [lf,lt,rf,rt]) $   -- zip bone index onto bone forces to make this a lookup list
        fmap sumF $                                                 -- sum forces on each bone to get net forces on each bone
        (splitPlaces (fmap L.length contactPointsBs) f0c)           -- f0c grouped by bone


    
    ------------------------------------------
    -- ID: Recursive Newton–Euler Algorithm --
    ------------------------------------------
    
    -- spatial motion base changing matrix to change from one joint coordinate system to another
    -- input is the i bone index, but the basies are the starting joint of that bone (see bjs)
    -- bone index -> transformation matrix from parent joint to joint i
    ixpi :: BoneIx -> Matrix
    ixpi = ixpij . bjp
    ixpij :: JointIx -> Matrix
    ixpij jI@(J ji) = axb (v32Vector $ offset (j ji)) (conjugate $ rjL jI)
    
    ixpiF = ixpijF . bjp
    ixpijF jI@(J ji) = axbF (v32Vector $ offset (j ji)) (conjugate $ rjL jI)
    
    -- basis conversion from global basis for spatial forces to joint basis
    xfO :: BoneIx -> Matrix 
    xfO = xfOj . bjp
    xfOj :: JointIx -> Matrix 
    xfOj jI = axbF (xj jI) (conjugate $ rj jI)
    
    -- Force basis change axbF = (axb)^-T = (bxa)^T   (derived by swapping a and b. see the input definitions for axb)
    -- ba = vector b to a in b frame
    -- br = quaternion rotation of b frame relative to a frame
    axbF ba br = vectorT $ axb ((-1) *^ ((quat2Matrix br) !* ba)) (conjugate br)
    
    -- helper function for base change
    -- ba = vector b to a in b frame
    -- br = quaternion rotation of b frame relative to a frame
    axb ba br =             (irpi ||| (m0 3 3))
                                  -|-
            (((s3 ippi) !*! irpi) ||| irpi)
        where
            -- translation from i to pi in i frame (note that i is rotated according to (rjL i))
            ippi = irpi !* ((-1) *^ ba)
            -- rotation
            irpi = quat2Matrix br
            
    
    -- spatial inertia (2.2.11)
    -- this must be relative to the start joint of the input bone
    -- Note that the MDV's inertia matrix (from inertC) is 3x3 and relative to the center of mass of the bone
    inert :: BoneIx -> Matrix
    inert i =   ((icm !+! (msc !*! (vectorT sc)))     |||        msc)
                                                      -|-
                         ((vectorT msc)               |||   (m *!! (mid 3 3)))
     where
        m = mass i
        -- position of CoM in joint coordinate system
        c = (v32Vector $ offset $ j $ unJ (bje i)) ^/ 2
        sc = s3 c
        msc = m *!! sc
        -- inertia about the CoM
        icm =  m332Matrix $ inertC i
       
    -- spatial external force in global coordinates
    -- includes gravity unless added to the root acceleration
    fOe :: BoneIx -> ForceV  
    fOe bI = -- zeroP
        -- gravity. convert a pure force from the center of mass to a spatial force in base frame
        (oxc !* (mass bI *^ g)) ^+^
        -- GRF
        (if useGRF then (maybe zeroP id (lookup bI boneForce0Lookup)) else zeroP)
        where
            -- Note that the gravity is specified in global coordinates (not the frame of the bone even though the point of application is the center of the bone)
            oxc = axbF ((-1) *^ (xb bI)) identity
    
    -- joint type matrix Φ. All joints are the same type (ball and socket) other than the rrot which is just 6 DOF
    oI :: JointIx -> Matrix
    oI (J ji) = if ji == 0 then mid 6 6 else mid 6 3
    
    -- spatial velocity by bone index (-1 is base)
    v :: BoneIx -> MotionV
    v (B bi) = if bi == -1 then zeroP else v_!bi where
        v_ = generate bN (\bi -> let (bI,jI) = (B bi, bjp bI) in
                ((ixpi bI) !* (v (p bI))) ^+^ ((oI jI) !* (q' jI))
            )
    
    -- gravity vector
    g = fromList [0,0,0, 0,-gravityAcc,0]
    
    -- spatial acceleration
    a :: BoneIx -> MotionV
    a (B bi) = if bi == -1
        then
            -- gravity compensation can be achieved by setting the acceleration of the base joint to -gravity, or by using eternal forces
            -- NOTE the algorithm uses "a_0 = - a_g" but as the direction of gravity is in the negative
            -- y direction, we have a double negative, so can just use (0,0,0, 0,g,0) where g is the magnatude
            -- of gravitaional acceleration
            --(-1) *^ g    -- using base accel
            zeroP -- NOT using base accel
        else 
            a_!bi where
                a_ = generate bN (\bi -> let (bI,jI) = (B bi, bjp bI) in
                        ((ixpi bI) !* (a (p bI))) ^+^ ((oI jI) !* (q'' jI)) ^+^
                            {- assume ̇Φ' term is 0 -} ((v bI) `mxm` ((oI jI) !* q' jI))
                    )
                    
    fNet :: BoneIx -> ForceV
    fNet (B bi) = fNet_ ! bi where
        fNet_ = generate bN (\bi -> let bI = (B bi) in
                ((inert bI) !* (a bI)) ^+^ ((v bI) `mxf` ((inert bI) !* (v bI)))
            )
    
    -- spatial forces of each bodies's parent joint (joint attaching it to the parent bone)
    --   f' = net force - external forces
    f' :: BoneIx -> ForceV
    f' (B bi) = f'_ ! bi where
        f'_ = generate bN (\bi -> let bI = (B bi) in
                (fNet bI) ^-^ ((xfO bI) !* (fOe bI))
            )
    
    --   f = joint force = f' + child joint forces
    f :: BoneIx -> ForceV
    f (B bi) = f_ ! bi where
        f_ = generate bN (\bi -> let bI = B bi in
                -- net force - external forces = f'
                (f' bI) ^+^
                -- plus sum of child joint forces
                (L.foldr (^+^) zeroP [(vectorT (ixpi cbI)) !* (f cbI) | cjI <- cjs (bje bI), let cbI = ejb cjI])
            )
    
    -- Finally! the torques
    t :: BoneIx -> ForceV
    -- use this version to only get actuated DoFs
    --t (B bi) = t_ ! bi where t_ = generate bN (\i -> let (bI,jI) = (B bi, bjp bI) in (vectorT (oI jI)) !* (f bI))
    t (B bi) = t_ ! bi where t_ = generate bN (\i -> let (bI,jI) = (B bi, bjp bI) in f bI)
        
    -- some joints have multiple children (the root joitn is connected to the spine and 2 hip joints) accumulate these torques
    jointTorques :: JointIx -> ForceV
    jointTorques (J ji) = jointTorques'!ji where jointTorques' = accum (^+^) (replicate jN zeroP) (L.map (\bI -> (unJ (bjp bI), t bI)) bs)
    


cross :: Num a => Vector a -> Vector a -> Vector a
cross a b = (s3 a) !* b





--
-- Unit Tests
--

unitTestsID = test [

        "Convert Vec3 to Vector (v32Vector)" ~: test [
                (fromList [1,2,3]) ~=? (v32Vector (V3 1 2 3)),
                (fromList [(-3),5,10]) ~=? (v32Vector (V3 (-3) 5 10))
            ]
    
        ,"Test that the cross product matrix s3 (actually the local cross function) works as expected (compared to Linear.cross) " ~:
            sequence_ [   ((fromList [x1,y1,z1]) `cross` (fromList [x2,y2,z2])) @?~= (v32Vector (v1 `L.cross` v2))     |      v1@(V3 x1 y1 z1) <- testDataVec3, v2@(V3 x2 y2 z2) <- testDataVec3   ]
        
        
    ]


