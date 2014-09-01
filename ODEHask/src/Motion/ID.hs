{-
  This code is based off of Roy Featherstone and David E. Orin book part A chapter 2 "Dynamics"
-}

module Motion.ID (
    inverseDynamics
) where

import Prelude hiding (map,foldr,replicate, (++))
import qualified Data.List as L
import qualified Data.Vector as V
import Data.Vector (Vector, generate, map, fromList, toList, replicate, (!), (++), imap)
import qualified Data.Array as A
import Data.Array hiding ((!))
import Linear hiding (_j,_i,cross,trace)
import Util
import Motion.MotionDataVars
import Motion.Joint
import Debug.Trace


-- (2.2) Plueker forces and motion. angular then linear

type P = Vector Double
type MotionV = P
type ForceV = P
type Matrix = Vector (Vector Double)

zeroP = replicate 6 0

-- identity matrix
m0 ::  Int -> Int -> Matrix
m0 r c = replicate r (replicate c 0)
mid :: Int -> Int -> Matrix
mid r c = generate r (\r -> generate c (\c -> if r == c then 1 else 0))

split = V.splitAt 3


-- (2.2.7)
-- 3x3 cross product matrix
s3 :: Num a => Vector a -> Vector (Vector a)
s3 v = fromList2 [
            [    0,     -(v!2),     v!1   ],
            [   v!2,      0,      -(v!0)  ],
            [ -(v!1),    v!0,        0    ]
        ]

smm :: P -> Matrix
smm vm = fromList2Vector2 [[s3m,    zero33],
                           [s3 mo,  s3m]] where
                                            (m,mo) = split vm
                                            s3m = s3 m

smf :: P -> Matrix
smf vm = (vectorT $ smm vm) !!* (-1)


mxm :: MotionV -> MotionV -> MotionV
vm1 `mxm` vm2 = (m1 `cross` m2) ++ ((m1 `cross` m2o) ^+^ (m1o `cross` m2)) where
    (m1,m1o) = split vm1
    (m2,m2o) = split vm2

mxf :: MotionV -> ForceV -> ForceV
vm `mxf` vf = ((m `cross` fo) ^+^ (mo `cross` f)) ++ (m `cross`f) where
    (m,mo) = split vm
    (fo,f) = split vf
    
    
    
-- Inverse Dynamics algorithm. as described by table 2.6
-- some differences: use B(-1) for the base body (the floor)
-- input a motion data and frame index, then output Joint indexed array of joint torques 
inverseDynamics :: MotionDataVars -> FrameIx -> Array Int Vec3
inverseDynamics md fI@(F fi) = fmap (\v -> V3 (v!0) (v!1) (v!2)) jointTorques where
    -- motion data
    MotionDataVars{
        _j=jf,
        _bN=bN,
        _jN=jN,
        _bj=bj_,
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
        _i=inertCB
    } = md
    
    j=jf fi
    cjs = (L.map unJ) . cjs_ . J
    jHasChild = jHasChild_ . J
    xj = v32Vector . (xjf fI) . J
    rjL = (rjLf fI) . J
    rj = (rjf fI) . J
    xb = v32Vector . (xbf fI) . B
    mass bi = massB (B bi)
    inertC bi = inertCB (B bi)
    q'  = v32Vector . (q'f  fI) . J
    q'' = v32Vector . (q''f fI) . J
    
    -- modification of bj to convert to the bone's starting joint instead of end joint (this fits the theory better)
    bjp :: Int -> Int
    bjp = unJ . pj_ . bj_ . B
    pjb = unJ . pj_ . bj_ . B
    p :: Int -> Int
    p = unB . pb_ . B
    
    -- ID
    
    -- spatial motion base changing matrix to change from one joint coordinate system to another
    -- inputs are bone indexies, but the basis are the starting joint of those bones (see bjs)
    -- bone index -> transformation matrix from parent joint to joint i
    ixpi :: Int -> Matrix
    ixpi bi = axb (v32Vector $ offset (j i)) (rjL i)
     where
        i = bjp bi
    
    -- basis conversion from global basis for spatial forces to joint basis
    xfO :: Int -> Matrix 
    xfO bi = axb (xj i) (rj i)
     where
        i = bjp bi
    
    -- helper function for base change
    -- ba = vector b to a in b frame
    -- ar = quaternion rotation from a to b
    axb ba ar = (irpi ||| (m0 3 3))
                                  -|-
            (((s3 ippi) !*! irpi) ||| irpi)
        where
            -- translation from i to pi in i frame (note that i is ratates according to (rjL i))
            ippi = irpi !* ((-1) *^ ba)
            -- rotation
            irpi = quat2Matrix (conjugate ar)      -- ??? TODO is this right???
            
    
    -- spatial inertia (2.2.11)
    -- this must be relative to the joint i (note input is bone i)
    -- Note that the MDV's inertia matrix is 3x3 and relative to the center of mass of the bone
    inert :: Int -> Matrix
    inert i =   ((icm !+! (msc !*! (vectorT sc)))     |||        msc)
                                                      -|-
                         ((vectorT msc)               |||   (m *!! (mid 3 3)))
     where
        m = mass i
        -- position of CoM in joint coordinate system
        c = (v32Vector $ offset $ j (bjp i)) ^/ 2
        sc = s3 c
        msc = m *!! sc
        -- inertia about the CoM
        icm =  m332Matrix $ inertC i
       
    -- spatial external force in global coordinates
    fOe :: Vector ForceV  
    fOe = replicate bN zeroP
    
    -- joint type matrix Φ. All joints are the same type (ball and socket)
    oI :: Int -> Matrix
    oI _ = mid 6 3
    
    -- spatial velocity by bone index (-1 is base)
    v :: Int -> MotionV
    v = (A.!) v_ where
        v_ = buildArray (-1,bN-1) (\i ->
                if i == -1
                    then
                        zeroP
                    else
                        ((ixpi i) !* (v (p i))) ^+^ ((oI i) !* (q' i))
            )
    
    -- spatial acceleration
    a :: Int -> MotionV
    a = (A.!) a_ where
        a_ = buildArray (-1,bN-1) (\i ->
                if i == -1
                    then
                        zeroP
                    else
                        ((ixpi i) !* (a (p i))) ^+^ ((oI i) !* (q'' i)) ^+^
                            {- assume ̇Φ' term is 0 -} ((v i) `mxm` ((oI i) !* q' i))
            )
    
    -- spacial forces of each joint
    --   f' = net force - external forces
    --   f = joint force = f' + child joint forces
    f' :: Vector ForceV
    f' = generate (bN-1) (\i ->
                ((inert i) !* (a i)) ^+^ ((v i) `mxm` ((inert i) !* (v i))) ^-^ ((xfO i) !* (fOe!i)) 
        )
    
    f :: Vector ForceV
    f = generate (bN-1) (\i ->
                -- net force - external forces = f'
                (f'!i) ^+^
                -- plus sum of child joint forces
                (L.foldr (^+^) zeroP (L.map (\ci -> trace ((show ci) L.++ "  " L.++ (name (j ci))) $ (vectorT (ixpi ci)) !* (f!ci)) (map pjb $ filter jHasChild (cjs i))))
        )
    
    -- Finally! the torques
    t :: Vector ForceV
    t = generate (bN-1) (\i ->
                (vectorT (oI i)) !* (f!i)
        )
        
    jointTorques :: Array Int ForceV
    jointTorques = A.accum (^+^) (listArray (0,jN-1) (L.repeat zeroP)) (toList $ imap (\i t -> (bjp i, t)) t)
    

quat2Matrix :: Quat -> Matrix
quat2Matrix q = m332Matrix $ (fromQuaternion q)

m332Matrix :: M33 Double -> Matrix
m332Matrix = v32Vector . (fmap v32Vector) 

v32Vector :: V3 a -> Vector a
v32Vector (V3 a b c) = fromList [a,b,c] 

cross :: Num a => Vector a -> Vector a -> Vector a
cross a b = (s3 a) !* b


