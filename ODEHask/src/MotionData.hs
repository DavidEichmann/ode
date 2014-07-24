module MotionData where

import Control.Applicative (liftA2)
import Data.List
import Data.Array
import Data.Maybe
import Data.Functor
import Data.TreeF as T
import Data.Geometry.Point
import Data.Geometry.Polygon
import Data.MemoTrie
import Linear hiding (_i,_j,_k,_w,trace)
import Util
import Constants
import FFI

import Debug.Trace
import Math.Tau

import Data.Char
import Text.Parsec
import Text.Parsec.Token hiding (dot)
import Text.Parsec.Language





data Joint = Joint {
            name       :: String,
            offset     :: Vec3,
            rotationL  :: Quat,
            channels   :: [Channel]
    } deriving (Show, Eq)


type JointF = (TreeF Joint)
type Frame = JointF
data MotionData = MotionData {
        frames        :: Array Int Frame,
        frameTime     :: Double,
        baseSkeleton  :: JointF
} deriving Show

type FJIndexedFrames = Int -> Int -> Joint

newtype FrameIx = F Int deriving (Show,Eq)
newtype JointIx = J Int deriving (Show,Eq)
newtype BoneIx = B Int deriving (Show,Eq)





data MotionDataVars = MotionDataVars {
    -- raw data
    _dt     :: Double,
    _j      :: FJIndexedFrames,
    _jBase  :: JointIx -> Joint,
    _baseSkeleton :: Frame,

    -- derived data
    _g      :: Double,
    _fN     :: Int,
    _jN     :: Int,
    _bN     :: Int,
    _js     :: [JointIx],
    _bs     :: [BoneIx],
    _fs     :: [FrameIx],
    _bj     :: BoneIx -> JointIx,
    _jb     :: JointIx -> BoneIx,
    _jHasParent :: JointIx -> Bool,
    _pj     :: JointIx -> JointIx,
    _pb     :: BoneIx -> BoneIx,
    _d      :: BoneIx -> Double,
    _m      :: BoneIx -> Double,
    _i      :: BoneIx -> M3,
    _rjL    :: FrameIx -> JointIx -> Quat,
    _rj     :: FrameIx -> JointIx -> Quat,
    _xj     :: FrameIx -> JointIx -> Vec3,
    _rb     :: FrameIx -> BoneIx -> Quat,
    _xb     :: FrameIx -> BoneIx -> Vec3,
    _xsb    :: FrameIx -> BoneIx -> Vec3,
    _xeb    :: FrameIx -> BoneIx -> Vec3,
    _l      :: FrameIx -> BoneIx -> Vec3,
    _w      :: FrameIx -> BoneIx -> Vec3,
    _l'     :: FrameIx -> BoneIx -> Vec3,
    _w'     :: FrameIx -> BoneIx -> Vec3,
    _p      :: FrameIx -> BoneIx -> Vec3,
    _h      :: FrameIx -> BoneIx -> Vec3,
    _p'     :: FrameIx -> BoneIx -> Vec3,
    _h'     :: FrameIx -> BoneIx -> Vec3,
    _mT     :: Double,
    _com    :: FrameIx -> Vec3,
    _pT     :: FrameIx -> Vec3,
    _hT     :: FrameIx -> Vec3,
    _pT'    :: FrameIx -> Vec3,
    _hT'    :: FrameIx -> Vec3,
    _zmp    :: FrameIx -> Vec3,
    _sp     :: FrameIx -> [Vec2],
    _zmpIsInSp :: FrameIx -> Bool,

    _bByName :: String -> BoneIx,
    _footBs :: ((BoneIx,BoneIx),(BoneIx,BoneIx)),
    _footJs :: ((JointIx,JointIx,JointIx),(JointIx,JointIx,JointIx)),
    _isFootBone  :: BoneIx -> Bool
}

seqMDV :: MotionDataVars -> a -> a
seqMDV mdv@MotionDataVars{
    _fs = fs,
    _js = js,
    _zmp = zmp,
    _sp = sp
} a =
    (seqF (show . sp))
    (seqF zmp) a
    where
        seqF fn = seq $ foldl1' seq [fn fI | fI <- fs]

getMotionDataVariablesFromMotionData :: MotionData -> MotionDataVars
getMotionDataVariablesFromMotionData md = getMotionDataVariables (frameTime md) j (baseSkeleton md) fN where

    fN = arraySize $ frames md
    jN = length $ flatten $ baseSkeleton md

    -- joints organized by frame
    j :: FJIndexedFrames
    j = (\ f j -> (flatten $ (frames md) ! f) !! j) where
        jArr = array bnd [((fi,ji), (flatten $ (frames md) ! fi) !! ji) | (fi,ji) <- range bnd]
        bnd = ((0,0),(fN-1,jN-1))


modifyMotionDataVars :: MotionDataVars -> [((Int,Int), Joint)] -> MotionDataVars
modifyMotionDataVars (MotionDataVars{_dt=dt,_baseSkeleton=bSkel,_fN=fN,_j=jOrig}) jUpdates = getMotionDataVariables dt jNew bSkel fN where
    jNew ji fi = fromMaybe (jOrig ji fi) (jUpdatesMaybe ji fi)
    jUpdatesMaybe = memo2 (\ j f -> lookup (j,f) jUpdates)


-- arguments: frameTimeDelta  (motion data by FrameIx and JointIx) (base skeletion)
-- Note that Joint index is from 0 and coresponds to flettening the base skeletoin
getMotionDataVariables :: Double -> FJIndexedFrames -> Frame -> Int -> MotionDataVars
getMotionDataVariables dt j bskel fN = MotionDataVars {
    _dt     = dt,
    _j      = j,
    _jBase  = jBase,
    _baseSkeleton = bskel,

    _g      = g,
    _fN     = fN,
    _jN     = jN,
    _bN     = bN,
    _js     = js,
    _bs     = bs,
    _fs     = fs,
    _bj     = bj,
    _jb     = jb,
    _pj     = pj,
    _jHasParent = jHasParent,
    _pb     = pb,
    _d      = d,
    _m      = m,
    _i      = i,
    _rjL    = rjL,
    _rj     = rj,
    _xj     = xj,
    _rb     = rb,
    _xb     = xb,
    _xsb    = xsb,
    _xeb    = xeb,
    _l      = l,
    _w      = w,
    _l'     = l',
    _w'     = w',
    _p      = p,
    _h      = h,
    _p'     = p',
    _h'     = h',
    _mT     = mT,
    _com    = com,
    _pT     = pT,
    _hT     = hT,
    _pT'    = pT',
    _hT'    = hT',
    _zmp    = zmp,
    _sp     = sp,
    _zmpIsInSp = zmpIsInSp,

    _bByName = bByName,
    _footBs = footBs,
    _footJs = footJs,
    _isFootBone = isFootBone
 } where


    memoizeJ :: (JointIx->b) -> (JointIx->b)
    memoizeJ fn = (\(J a) -> intMemo a) where
        intMemo = memo (\a -> fn (J a))

    memoizeB :: (BoneIx->b) -> (BoneIx->b)
    memoizeB fn = (\(B a) -> intMemo a) where
        intMemo = memo (\a -> fn (B a))

    memoizeF :: (FrameIx->b) -> (FrameIx->b)
    memoizeF fn = (\(F a) -> intMemo a) where
        intMemo = memo (\a -> fn (F a))

    memoizeFJ :: (FrameIx->JointIx->b) -> (FrameIx->JointIx->b)
    memoizeFJ fn = (\(F a) (J b) -> intMemo a b) where
        intMemo = memo2 (\a b -> fn (F a) (J b))

    memoizeFB :: (FrameIx->BoneIx->b) -> (FrameIx->BoneIx->b)
    memoizeFB fn = (\(F a) (B b) -> intMemo a b) where
        intMemo = memo2 (\a b -> fn (F a) (B b))



    --
    -- Data
    --

    -- joint count
    jN = length $ flatten $ bskel

    -- bone count
    bN = jN - 1

    bndFB = ((0,0),(fN-1,bN-1))
    bndF = (0,fN-1)
    bndJ = (0,jN-1)
    bndB = (0,bN-1)

    js :: [JointIx]
    js = map J (range bndJ)

    bs :: [BoneIx]
    bs = map B (range bndB)

    fs :: [FrameIx]
    fs = map F (range bndF)

    bj :: BoneIx -> JointIx
    bj (B a) = J (a+1)

    jb :: JointIx -> BoneIx
    jb (J a) = B (a-1)

    toJIx :: (FrameIx, JointIx) -> (Int, Int)
    toJIx ((F f),(J j)) = (f - (fst bndF),j - (fst bndJ))

    -- joints of baseSkeleton
    jBase :: JointIx -> Joint
    jBase = memoizeJ (\(J jI) -> (flatten $ bskel) !! (jI - (fst bndJ)))

    -- bones (a really just the set of all joints that have parents)
--    b :: FrameIx -> BoneIx -> Bone
--    b = j . bj

    g = gravityAcc

    -- total mass
    mT :: Double
    mT = sum (map m bs)

    --
    -- Joint (uses on baseSkeleton)
    --

    -- parent joint
    jHasParent :: JointIx -> Bool
    jHasParent = memoizeJ (\(J jI) -> jI > fst bndJ) -- only root (at first index) has no parent
    pj :: JointIx -> JointIx
    pj = memoizeJ pj_ where
        pj_ (J ji) = J pjIx where
            pJoint = (flatten $ T.treeMap' ((fmap T.view) . T.parent) (bskel)) !! (ji - (fst bndJ))
            pjIx = maybe (error "trying to access root's parent") (+ (fst bndJ)) (elemIndex pJoint (map (Just . jBase . J) (range bndJ)))

    --
    -- Bone (uses on baseSkeleton)
    --
--    bHasParent :: BoneIx -> Bool
--    bHasParent = memoizeB (\bI -> let jI = bj bI in jHasParent jI && jHasParent (pj jI) ) -- only root (at first index) has no parent
    pb :: BoneIx -> BoneIx
    pb = jb . pj . bj

    -- bone density
    d :: BoneIx -> Double
    d _ = boneDensity

    -- bone mass and local Inertia Matrix
    m :: BoneIx -> Double
    m = fst . mi
    i :: BoneIx -> M3
    i = snd . mi
    mi :: BoneIx -> (Double, M3)
    mi = memoizeB mi_ where
            mi_ bI = getCapsulMassInertiaM (d bI) boneRadius (offset (jBase $ bj bI))

    --
    -- Frame Joint
    --

    rjL :: FrameIx -> JointIx -> Quat
    rjL (F fi) (J ji) = rotationL $ j fi ji

    -- joint global rotation
    rj :: FrameIx -> JointIx -> Quat
    rj = memoizeFJ rj_ where
        rj_ fI jI
            | jHasParent jI = (rj fI (pj jI)) * rotL
            | otherwise     = rotL
            where
                rotL = rjL fI jI

    xj :: FrameIx -> JointIx -> Vec3
    xj = memoizeFJ xj_ where
        xj_ fI@(F fi) jI@(J ji)
            | jHasParent jI = (xj fI $ pj jI) + ((rj fI $ pj jI) `rotate` (offset $ jBase jI))
            | otherwise     = offset $ j fi ji

    --
    -- Frame Bone
    --

    -- bone orientation
    rb :: FrameIx -> BoneIx -> Quat
    rb fI bI = ((rj fI) . pj . bj) bI

    -- bone start
    xsb :: FrameIx -> BoneIx -> Vec3
    xsb fI bI = ((xj fI) . pj . bj) bI
    -- bone end
    xeb :: FrameIx -> BoneIx -> Vec3
    xeb fI bI = ((xj fI) . bj) bI

    -- bone CoM position
    xb :: FrameIx -> BoneIx -> Vec3
    xb = memoizeFB xb_ where
        xb_ fI bI = ((x1) + (x2)) ^/ 2 where
            x1 = ((xj fI) . pj . bj) bI
            x2 = ((xj fI) . bj) bI

    -- helper function to measure change over time (using current and previous frame)
    -- first frame simply copies the result from the second
    deriveFB :: (FrameIx -> BoneIx -> Vec3) -> (FrameIx -> BoneIx -> Vec3)
    deriveFB fn = derive__ where
        derive_ = memoizeFB derive__
        derive__ (fI@(F fi)) bI
            | fi > fst bndF     = ((fn fI bI) - (fn (F (fi-1)) bI)) ^/ dt
            | otherwise         = derive_ (F (fi+1)) bI
    derive2FB :: (FrameIx -> BoneIx -> Vec3) -> (FrameIx -> BoneIx -> Vec3)
    derive2FB fn = derive__ where
        derive_ = memoizeFB derive__
        derive__ (fI@(F fi)) bI
            | fi == fst bndF = derive_ (F (fi+1)) bI
            | fi == snd bndF = derive_ (F (fi-1)) bI
            | otherwise      = ((fn (F (fi-1)) bI) - (2 * (fn fI bI)) + (fn (F (fi+1)) bI)) ^/ (dt**2)
    deriveF :: (FrameIx -> Vec3) -> (FrameIx -> Vec3)
    deriveF fn = derive__ where
        derive_ = memoizeF derive__
        derive__ (fI@(F fi))
            | fi > fst bndF     = ((fn fI) - (fn (F (fi-1)))) ^/ dt
            | otherwise         = derive_ (F (fi+1))

    -- bone linear velocity
    l :: FrameIx -> BoneIx -> Vec3
    l = deriveFB xb



    -- bone angular velocity
    w :: FrameIx -> BoneIx -> Vec3
    w = memoizeFB w_ where
        w_ (fI@(F fi)) bI
            | fi > fst bndF     = toAngularVel (rb (F (fi-1)) bI) (rb fI bI)  dt
            | otherwise         = w (F (fi+1)) bI

    -- bone linear acc
    l' :: FrameIx -> BoneIx -> Vec3
    l' = derive2FB xb

    -- bone angular acc
    w' :: FrameIx -> BoneIx -> Vec3
    w' = memoizeFB w'_ where
        w'_ (fI@(F fi)) bI
            | fi > fst bndF     = ((toAngularVel (rb (F (fi-1)) bI) (rb fI bI)  dt) - (toAngularVel (rb fI bI) (rb (F (fi+1)) bI) dt)) ^/ dt
            | otherwise         = w' (F (fi+1)) bI

    -- bone linear momentum
    p :: FrameIx -> BoneIx -> Vec3
    p fI bI = (l fI bI) ^* (m bI)

    -- bone angular momentum
    h :: FrameIx -> BoneIx -> Vec3
    h fI bI = rot `rotate` ((i bI) !* (rotInv `rotate` (w fI bI))) where
        rot = rb fI bI
        rotInv = conjugate rot

    -- bone linear momentum rate
    p' :: FrameIx -> BoneIx -> Vec3
    p' fI bI = (l' fI bI) ^* (m bI)

    -- bone angular momentum rate
    h' :: FrameIx -> BoneIx -> Vec3
    h' fI bI = rot `rotate` ((i bI) !* (rotInv `rotate` (w' fI bI))) where
        rot = rb fI bI
        rotInv = conjugate rot

    --
    -- Frame
    --

    -- total center of mass
    com = memoizeF com_ where
        com_ fI = (sum (map weightedCom bs)) ^/ mT where
            weightedCom bI = (m bI) *^ (xb fI bI)

    -- total linear momentum
    pT :: FrameIx -> Vec3
    pT = memoizeF pT_ where
        pT_ fI = sum (map (p fI) bs)

    -- total angular momentum about the origin
    hT :: FrameIx -> Vec3
    hT = memoizeF hT_ where
        hT_ fI = sum (map hComp bs) where
            hComp bI = ((xb fI bI) `cross` (p fI bI)) + (h fI bI)

    -- total linear momentum
    pT' :: FrameIx -> Vec3
    pT' =  memoizeF pT'_ where
        pT'_ fI = sum (map (p' fI) bs)

    -- total angular momentum about the origin
    hT' :: FrameIx -> Vec3
    hT' = memoizeF hT'_ where
        hT'_ fI = sum (map hComp bs) where
            hComp bI = ((l fI bI) `cross` (p fI bI)) + ((xb fI bI) `cross` (p' fI bI)) + (h' fI bI) + ((w fI bI) `cross` (h fI bI))

    -- ZMP
    -- assume that ZMP is on the floor (y component is 0)
    -- to preserve right-handedness we must swap (ZMP paper axis -> my code axis): z -> y, y -> x, x -> y)
    zmp :: FrameIx -> Vec3
    zmp = memoizeF zmp_ where
        zmp_ fI = V3
            -- definition from "Online Generation of Humanoid Walking Motion based on a Fast Generation Method of Motion Pattern that Follows Desired ZMP"
            ((sum $ map (\bI -> ((m bI) * (vy (xb fI bI)) * (vx (l' fI bI)) - ((m bI) * (vy (l' fI bI) + g) * (vx (xb fI bI))) + (vz (h fI bI))) ) bs) /
                (negate $ sum $ map (\bI -> (m bI) * ((vy (l' fI bI)) + g)) bs))
            0
            ((sum $ map (\bI -> ((m bI) * (vy (xb fI bI)) * (vz (l' fI bI)) - ((m bI) * (vy (l' fI bI) + g) * (vz (xb fI bI))) + (vx (h fI bI))) ) bs) /
                (negate $ sum $ map (\bI -> (m bI) * ((vy (l' fI bI)) + g)) bs))
--            -- definition from http://www.mate.tue.nl/mate/pdfs/10796.pdf
--            (((mT * g * comX) + h'Z) / ((mT * g) + p'Y))
--            0
--            (((mT * g * comZ) - h'X) / ((mT * g) + p'Y))
--            where
--                (V3 comX _ comZ) = com fI
--                (V3 _ p'Y _) = pT' fI
--                (V3 h'X _ h'Z) = hT' fI

    isFootBone :: BoneIx -> Bool
    isFootBone bI = (bName bI) `elem` footJoints where
        footJoints = ["LeftFoot", "LeftToe", "RightFoot", "RightToe"]

    footBs :: ((BoneIx,BoneIx),(BoneIx,BoneIx))
    footBs = (
            (
                bByName "LeftAnkle",
                bByName "LeftToe"
            ),
            (
                bByName "RightAnkle",
                bByName "RightToe"
            )
        )

    footJs :: ((JointIx,JointIx,JointIx),(JointIx,JointIx,JointIx))
    footJs = (
            (
                pj . bj $ lfI,
                bj lfI,
                bj ltI
            ),
            (
                pj . bj $ rfI,
                bj rfI,
                bj rtI
            )
        ) where
            ((lfI,ltI),(rfI,rtI)) = footBs

    bName :: BoneIx -> String
    bName = name . jBase . pj . bj

    bByName :: String -> BoneIx
    bByName bn = fromMaybe (error $ "Bone with name\"" ++ bn ++ "\" could not be found") (lookup bn (zip (map bName bs) bs))

    soleCorners :: FrameIx -> [Vec3]
    soleCorners fI = corners where
        -- hack to convert base skeleton to motiondatavars (use only one frame consisting of the base skeleton)
        MotionDataVars{_xb=base_xb',_xsb=base_xsb',_xeb=base_xeb'} = getMotionDataVariablesFromMotionData (MotionData {
                    frames = listArray (0,0) [bskel],
                    frameTime = dt,
                    baseSkeleton = bskel
            })
        f0 = F 0
        base_xb  = base_xb'  f0
        base_xsb = base_xsb' f0
        base_xeb = base_xeb' f0
        -- base measurements off of left base foot (assume it it pointing towards +Z and that offsets are the same for the right foot)
        ti = bByName "LeftToe"
        fi = pb ti
        halfFootWidth = footWidth / 2
        -- floor is at tip of toe
        (V3 _ floorY _) = base_xeb ti
        -- main foot sole corners
        (V3 _ _ fHalfLength) = (fmap abs ((base_xeb fi) - (base_xsb fi))) ^/ 2
        fSoleYOffset = floorY - (vy (base_xb fi))
        footCorners = [
                V3   halfFootWidth  fSoleYOffset fHalfLength,
                V3 (-halfFootWidth) fSoleYOffset fHalfLength,
                V3   halfFootWidth  fSoleYOffset (-fHalfLength),
                V3 (-halfFootWidth) fSoleYOffset (-fHalfLength)
            ]
        -- toe corners
        (V3 _ _ tHalfLength) = (fmap abs ((base_xeb ti) - (base_xsb ti))) ^/ 2
        tSoleYOffset = floorY - (vy (base_xb ti))
        toeCorners = [
                V3   halfFootWidth  tSoleYOffset tHalfLength,
                V3 (-halfFootWidth) tSoleYOffset tHalfLength,
                V3   halfFootWidth  tSoleYOffset (-tHalfLength),
                V3 (-halfFootWidth) tSoleYOffset (-tHalfLength)
            ]

        -- get all corners
        ((lfI,ltI),(rfI,rtI)) = footBs
        corners =
            (map (l2g lfI) footCorners) ++
            (map (l2g ltI) toeCorners)  ++
            (map (l2g rfI) footCorners) ++
            (map (l2g rtI) toeCorners)

        l2g :: BoneIx -> Vec3 -> Vec3
        l2g bI offset = (xb fI bI) + (rb fI bI `rotate` offset)

    -- support Polygon (must be touching the ground else Nothing)
    sp :: FrameIx -> [Vec2]
    sp fI
        | length contactPoints <= 1 = []
        | otherwise             = hull
        where
            hull = convexHull contactPoints
            contactPoints = map (\(V3 x _ z) -> V2 x z) (filter (\(V3 _ y _) -> y <= floorContactThreshold) (soleCorners fI))

    zmpIsInSp :: FrameIx -> Bool
    zmpIsInSp fi' = polyContainsPoint (sp fi') (toXZ (zmp fi'))





data Channel = Xpos | Ypos | Zpos | Xrot | Yrot | Zrot deriving (Eq, Show)
posChans :: [Channel]
posChans = [Xpos,Ypos,Zpos]
rotChans :: [Channel]
rotChans = [Xrot,Yrot,Zrot]


getCapsulMassInertiaM :: Double -> Double -> Vec3 -> (Double,M3)
getCapsulMassInertiaM d r boneVec = --(mass, rotM !*! xm !*! rotInvM) where
    (mass, rotM !*! xm !*! rotInvM) where
        (mass, xm) = getXCapsulMassInertiaM d r (norm boneVec)
        rot     = xToDirQuat boneVec
        rotInv  = conjugate rot
        rotM    = fromQuaternion rot
        rotInvM = fromQuaternion rotInv

getXCapsulMassInertiaM :: Double -> Double -> Double -> (Double,M3)
getXCapsulMassInertiaM d r l = (m, im) where
    r2 = r ** 2
    l2 = l ** 2
    m1 = pi * l * r2 * d
    m = m1 {- +m2

    For now just use a cylinder
    m2 = (4/3) * pi * r2 * r * d
    ia = m1 * (0.25 * r2 + (1/12) * l2) +
         m2 * (0.4  * r2 + 0.375  * r * l + 0.25 * l2)
    ib = ((m1 * 0.5) + (m2 * 0.4)) * r2;

    im = V3 (V3  ib  0   0)
            (V3  0   ia  0)
            (V3  0   0   ia)
            -}

    iX  = 0.5 * m1 * r2
    iZY = (1/12) * m1 * ((3 * r2) + (l2))
    im = V3 (V3  iX  0   0)
            (V3  0   iZY  0)
            (V3  0   0   iZY)


toAngularVel :: Quat -> Quat -> Double -> Vec3
toAngularVel qi qf dt = if angleHalf == 0 then zero else ((2 * angleHalf) *^ rotAxis) ^/ dt where
                        Quaternion qlnW qlnV = qf * (conjugate qi)
                        angleHalf = acos $ max (-1) (min qlnW 1)
                        rotAxis = qlnV ^/ (sin angleHalf)





-- Legacy code
-- inertia matrix is computed as that of a capsule represented by the offset vector
-- as the root joint does not represent a bone, it has no valid inertia matrix, so zero is used
massInertiaM :: JointF -> (Double, M3)
massInertiaM jF
        | hasParent jF   = getCapsulMassInertiaM (density jF) boneRadius (offset $- jF)
        | otherwise     = (0,zero)


-- legacy code
density :: JointF -> Double
density jF
        | hasParent jF   = boneDensity
        | otherwise     = 0





--
-- Parsing
--


nullJoint :: Joint
nullJoint = Joint {
        name = "",
        offset = zero,
        rotationL = identity,
        channels = []
}

numChannels :: Joint -> Int
numChannels = length . channels

hasPosChan :: Joint -> Bool
hasPosChan j = not $ null (intersect posChans (channels j))

hasRotChan :: Joint -> Bool
hasRotChan j = not $ null (intersect rotChans (channels j))


scale :: Double -> MotionData -> MotionData
scale s md = md{
                frames = fmap (jointScale s) (frames md),
                baseSkeleton = jointScale s (baseSkeleton md)
        }

translate :: Vec3 -> MotionData -> MotionData
translate t md = md{
                frames = fmap (jointTranslate t) (frames md),
                baseSkeleton = jointTranslate t (baseSkeleton md)
        }

jointScale :: Double -> JointF -> JointF
jointScale s = treeMap (map1 (\j@Joint{offset=o} -> j{offset=o ^* s}))

jointTranslate :: Vec3 -> JointF -> JointF
jointTranslate t = map1 (\r@Joint{offset=o} -> r{offset=o+t})

fromChanVals :: JointF -> [Double] -> JointF
fromChanVals jf cv = (\(d,r) -> if d == [] then r else error $ "chan vals left over: " ++ (show d)) $ treeMapCon applyChans cv jf where
         applyChans cv jf = (cv', set (sk{ offset = offset', rotationL = rotationL'}) jf) where
                sk = view jf
                baseOffset = offset sk
                baseRot    = rotationL sk
                (offset', rotationL', cv') = consume baseOffset baseRot (channels sk) cv
                consume ost rot [] vs = (ost, rot, vs)
                consume _ _ _ [] = error("Not enough values to fill channels")
                consume (V3 _ py pz) rot (Xpos:cs) (v:vs) = consume (V3 v py pz) rot cs vs
                consume (V3 px _ pz) rot (Ypos:cs) (v:vs) = consume (V3 px v pz) rot cs vs
                consume (V3 px py _) rot (Zpos:cs) (v:vs) = consume (V3 px py v) rot cs vs


               {- consume pos rot (Yrot:Xrot:Zrot:cs) (y:x:z:vs) = consume pos (
                        rot *
                        (axisAngle (unitY) (degreeToRadian y)) *
                        (axisAngle (unitX) (degreeToRadian x)) *
                        (axisAngle (unitZ) (degreeToRadian z))
                    ) cs vs -}

                consume pos rot (Xrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitX) (degreeToRadian v))) cs vs
                consume pos rot (Yrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitY) (degreeToRadian v))) cs vs
                consume pos rot (Zrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitZ) (degreeToRadian v))) cs vs


parseBVH :: String -> Integer -> IO (MotionData)
parseBVH filePath ppFilterWindow = do
        bvh <- raw_parseBVH filePath
        return $ (if ppFilterWindow > 0 then preProcess ppFilterWindow else id) bvh

-- currently this just uses moving average for joint angles and positions
preProcess :: Integer -> MotionData -> MotionData
preProcess windowSize md
    | newBounds /= bounds (frames md)   = error "preprocessing: new bounds not equal to old bounds!!!"
    | otherwise = mdF where
        n = fromInteger windowSize
        newFrames = map avg (windows (elems $ frames md))
        newBounds = (0, (length newFrames) - 1)
        mdF = md{
                 frames = listArray newBounds newFrames
            }
        windows fs = lfsn : if length lfsn == 1 then [] else windows (tail fs)
            where
                lfsn = take n fs
        avg fs = fmap avgJ (foldl1 (treeZipWith (\a b -> (view a) ++ (view b))) (map (fmap (:[])) fs))
        avgJ :: [Joint] -> Joint
        avgJ js = (head js) {
            offset   = avgPos,
            rotationL = avgRotL
        }  where
            avgPos = (foldl1 (+) (map offset js)) ^/ (fromIntegral (length js))
            avgRotL = avgRot (map rotationL js)

-- addapted from "Online Generation of Humanoid Walking Motion based on a Fast Generation Method of Motion Pattern that Follows Desired ZMP"
-- attempts to move zmp of each frame such that it is in the support polygon
fitMottionDataToSP :: MotionDataVars -> (Array Int (Double, Double),MotionDataVars)
fitMottionDataToSP mdvOrig@MotionDataVars{_fN=fN,_g=g,_dt=dt,_bs=bs,_fs=fs,_zmp=zmpO,_sp=spO} = newMDV where

    newMDV = calculateOffsetMap mdvOrig (listArray (0,fN-1) (map (\_ -> (0,0)) fs)) 0

    -- take:    current mdv, current offset map, current frame index
    -- return:  new motion data vars
    calculateOffsetMap :: MotionDataVars -> Array Int (Double,Double) -> Int -> (Array Int (Double, Double),MotionDataVars)
    calculateOffsetMap (mdv@MotionDataVars{_zmpIsInSp=zmpIsInSp,_j=j,_m=m,_xb=xb,_l'=l',_com=com,_zmp=zmp,_sp=sp}) e fi
        -- end of Motion data.. then we are finished
        | fi > (snd $ bounds e)     = (e,mdv)
        -- ZMP already in the SP, move to the next frame
        | zmpIsInSp (F fi)          = calculateOffsetMap mdv e (fi + 1)
        -- ZMP outside of the SP, modify the frame and move to the next frame
        | otherwise                 = calculateOffsetMap shiftedMdv e' (fi + 1)
        where
            fI = F fi

            -- MDV with frame fi shifted by the new shift for the current frame (ce)
            shiftedMdv = modifyMotionDataVars mdvOrig ([((fi,0), fiRootJ{offset = (offset fiRootJ) + (V3 cex 0 cez)})])
            fiRootJ = j fi 0

            e' = e // [(fi,ce)]


            -- target ZMP position offset: via projection of zmp onto the SP towards the CoM
            -- TODO handle case where number of intersection points is 0 or more than 1 (take closest point to zmp -> smallest offset)
            (V2 zmpex zmpez)
                | spis == []  = V2 0 0
                | otherwise   = let zmpe = head $ sortBy (\a b -> compare (dot a a) (dot b b)) spis in trace ((show fi ++ " : zmpe : " ++ (show zmpe))) zmpe
            spis = map  (\x -> x-(toXZ (zmpO fI))) (polyEdgeLineSegIntersect poly (let zmp2Com = (toXZ (zmp fI), toXZ (com fI)) in trace (show $ zmp2Com) zmp2Com))

            poly = sp fI
--            poly = spO fI
--            expandFactor = 2
--            expandedSPO = map (\v -> ((v-centroidSPO)^*expandFactor) + centroidSPO ) poly
--            centroidSPO = (foldr1 (+) poly) ^/ (fromIntegral (length poly))

            -- current shift to get to the target ZMP
            ce@(cex,cez) = (
                    (zmpex - (b*(xe_2 - (2*xe_1)))) / (b), -- x
                    (zmpez - (b*(ze_2 - (2*ze_1)))) / (b)  -- z
                )
            (xe_2,ze_2) = if fi-2 >= fst (bounds e) then e!(fi-2) else (0,0)
            (xe_1,ze_1) = if fi-1 >= fst (bounds e) then e!(fi-1) else (0,0)

            b = negate $ sum [(m bi) * (vy (xb fI bi)) | bi <- bs]
                                        /
                     (sum [(m bi) * (vy (l' fI bi) + g) | bi <- bs] * dt2)


    dt2 = dt*dt


shiftPerFrame :: Array Int Vec3 -> MotionDataVars -> MotionDataVars
shiftPerFrame v md@MotionDataVars{_fs=fs,_j=j} = modifyMotionDataVars md [let joint = j fi 0 in ((fi,0), joint{offset = (offset joint) + v!fi}) | (F fi) <- fs]

shift :: Vec3 -> MotionDataVars -> MotionDataVars
shift v md@MotionDataVars{_fs=fs,_j=j} = modifyMotionDataVars md [let joint = j fi 0 in ((fi,0), joint{offset = (offset joint) + v}) | (F fi) <- fs]

dip :: Double -> MotionDataVars -> MotionDataVars
dip amount = shift (V3 0 (-amount) 0)

-- based off of "Online Generation of Humanoid Walking Motion based on a Fast Generation Method of Motion Pattern that Follows Desired ZMP"
-- origional motion data
-- target ZMP trajectory
-- dip (lower the body to extend leg reach)
-- number of itterations
fitMottionDataToZmp :: MotionDataVars -> Array Int Vec3 -> Double -> Int -> MotionDataVars
fitMottionDataToZmp mdvOrig zmpX dipHeight its = fitMottionDataToZmp' mdvOrig (listArray (bounds zmpX) [1,1..]) its where
    fitMottionDataToZmp' :: MotionDataVars -> Array Int Double -> Int -> MotionDataVars
    fitMottionDataToZmp' mdv _ 0 = mdv
    fitMottionDataToZmp' mdv weights itsI = finalModifiedMdv where
        MotionDataVars {
            _j       = j,
            _g       = g,
            _m       = m,
            _dt      = dt,
            _fN      = fN,
            _fs      = fs,
            _bs      = bs,
            _xb      = posb,
            _l'      = l',
            _zmp     = zmp
        } = mdv

        x f i = vx (posb (F f) i)
        y f i = vy (posb (F f) i)
        z f i = vz (posb (F f) i)

        -- here we setup:    M xe = xep
        --           and:    M ze = zep
        -- M is a matrix
        -- xe is the vector of shifts in body position (to be solved with a matrix solver)
        -- xep is the shift is ZMP poisitons through time (note that first and last elements are 0)

        -- generate the matrix M (list of position and values... sparse matrix)
        constraintFramesRix = zip [0,fN-1,1] [0,fN-1,fN]
        mrn = fN - 2 + (length constraintFramesRix)
        _M :: [((Int,Int),Double)]
        _M =    -- fN-2 rows are the zmp = target zmp constraint for all frames other than the first and last
                (concat  [zip [(r,r-1), (r,r), (r,r+1)] (fmap (*(weights!r)) [f r, diag r, f r]) | r <- [1..fN-2]]) ++
                -- remaining rows ensure that there is no shift for the constraintFrames
                (map (\(f,rix) -> ((rix,f), 10000000000)) constraintFramesRix)
            where
                dt2         = dt ** 2
                f :: Int -> Double
                f           = memoize (0,fN-1) f' where
                                f' :: Int -> Double
                                f' r =  (negate (sum (map (\bI -> (m bI) * (y r bI)) bs))) /
                                            (dt2 * (sum (fmap (\bI -> (m bI) * ((vy $ l' fI bI) + g)) bs))) where
                                           fI = F r
                diag r = 1 - (2 * (f r))

        -- ep
        ep :: Array Int Vec3
        ep = array (0,mrn-1) (
                            -- first fN-2 rows is the zmp = target zmp constraint for all frames other than the first and last
                            [(fi, (weights!fi) *^ ((zmpX!fi) - (zmp (F fi)))) | fi <- range (1,fN-2)] ++
                            -- remaining rows ensure that ther is no shift for the constraintFrames
                            [(rix,V3 0 0 0) | (_,rix) <- constraintFramesRix])
        -- xep
        xep = fmap vx ep
        zep = fmap vz ep

        -- use matrix solver to get xe
        --  sparseMatrixSolve :: [((Int,Int),Double)] -> [Array Int Double] -> [Array Int Double]
--        (ze:xe:_) = leastSquareSparseMatrixSolve mrn fN _M [zep,xep]
        xe:_ = leastSquareSparseMatrixSolve mrn fN _M [xep]
        ze:_ = leastSquareSparseMatrixSolve mrn fN _M [zep]
        --(xe:ze:_) = traceShow ep (sparseMatrixSolve _M  [xep,zep])

        shiftedFrames = [ let joint@Joint{offset=offset} = j fi 0 in ((fi, 0), joint{ offset = offset + (V3 (xe!fi) 0 (ze!fi)) })  | fi <- [0..fN-1] ]
        modifiedMdv@MotionDataVars{_zmp=zmpActualResult} = modifyMotionDataVars mdv shiftedFrames

        weightsLearnRateA = 30
        weightsLearnRateB = 2
        newWeightsRaw = array (bounds weights) [ (fi, (weights!fi) + ((weightsLearnRateA * (norm $ (zmpX!fi) - (zmpActualResult (F fi)))) ** weightsLearnRateB)) | fi <- indices weights]
        windowHalf = 2
        newWeights = let x = listArray (bounds weights) [ (sum (map (\fi -> if inRange (bounds weights) fi then newWeightsRaw!fi else 0) [fi-windowHalf..fi+windowHalf])) / (1 + (2*(fromIntegral windowHalf))) | fi <- indices weights] in traceShow (elems x) x

        --modifyMotionDataVars :: MotionDataVars -> FJIndexedFrames -> MotionDataVars
        doFeetIK = False
        finalModifiedMdv = fitMottionDataToZmp' ((if doFeetIK then (correctFeet mdvOrig) else id) $ (if itsI == its then dip dipHeight else id) $ if itsI == 1 then modifiedMdv else mdv) newWeights (itsI-1)

        -- Move (via IK) feet in the second MDV argument to the position of the feet in the first
        -- update the motion data. Each frame is updated independantly (individual updates are collected from "correctFrame")
        correctFeet :: MotionDataVars -> MotionDataVars -> MotionDataVars
        correctFeet target@MotionDataVars{_xj=xjT} md@MotionDataVars{_pj=pj,_bj=bj,_fN=fN,_fs=fs,_j=j,_rj=rj,_xj=xj,_bByName=bByName,_footBs=((lfbI,_),(rfbI,_))} =
            trace
             "Done correcting feet with IK"
             (modifyMotionDataVars md (concat [correctFrame fI | fI <- fs])) where

                -- returns the updates to apply to a given frame
                correctFrame :: FrameIx -> [((Int,Int), Joint)]
                correctFrame fI@(F fi) = (correctLeg lajI) ++ (correctLeg rajI) where

                    correctLeg ajI@(J aji) = [
                             ((fi, hji), jF fi hji) -- jointH{ rotationL = (rotationL jointH) * dRotHL})
                            ,((fi, kji), jF fi kji) -- jointK{ rotationL = (rotationL jointK) * dRotKL})
                            ,((fi, aji), jF fi aji) -- jointA{ rotationL = (rotationL jointA) * dRotAL})
                        ] where

                            kjI@(J kji) = pj ajI
                            hjI@(J hji) = pj kjI

                            -- joints

                            jointH = j fi hji
                            jointK = j fi kji
                            jointA = j fi aji


                            -- clamped asin/acos

                            asinc v = asin $ max (-1) (min v 1)
                            acosc v = acos $ max (-1) (min v 1)

                            -- target ankel position (normalized for max extention)

                            maxExtention = (norm ((xj fI kjI) - (xj fI hjI))) + (norm ((xj fI ajI) - (xj fI kjI)))

                            aT = h' + ((min (norm hat') maxExtention) *^ (normalize hat')) where
                                h' = (xj fI hjI)
                                hat' = (xjT fI ajI) - h'

                            -- setp 1   Rotate hip to point ankel toward target ankel

                            ha1  = (xj fI ajI)  - (xj fI hjI)
                            hat1 = aT - (xj fI hjI)

                            axisH = normalize $ ha1 `cross` hat1
                            angleDH2 = acosc $ (ha1 `dot` hat1) / ((norm ha1) * (norm hat1))

                            md2@MotionDataVars{_j=j2,_xj=xj2,_rj=rj2} = modifyMotionDataVars md [
                                    ((fi,hji), jointH{rotationL = (rotationL jointH) * (axisAngle ((conjugate $ rj fI hjI) `rotate` axisH) angleDH2)})
                                ]
                            jointH2 = j2 fi hji
                            jointK2 = j2 fi kji

                            -- step 2   adjust knee/hip (rota aroun knee asxis) to be reach the correct distance (hipt to ankel)

                            -- some vectors
                            haT  = aT  - (xj2 fI hjI)
                            hatNorm = norm haT
                            ha  = (xj2 fI ajI)  - (xj2 fI hjI)
                            kh  = (xj2 fI hjI)  - (xj2 fI kjI)
                            hk  = (xj2 fI kjI)  - (xj2 fI hjI)
                            ka  = (xj2 fI ajI)  - (xj2 fI kjI)

                            -- original angles
                            angleK = acosc $ (kh `dot` ka) / ((norm kh) * (norm ka))
                            angleH = acosc $ (ha `dot` hk) / ((norm ha) * (norm hk))
                            -- new angles
                            angleKT = acosc $ ((hatNorm ** 2) - (norm2 hk) - (norm2 ka)) / (-2 * (norm hk) * (norm ka))
                            angleHT = asinc $ ((norm ka) * (sin angleKT)) / (hatNorm)

                            -- 2.1  extend
                            axisKExtend = normalize $ kh `cross` ka
                            md22@MotionDataVars{_j=j22,_xj=xj22,_rj=rj22} = modifyMotionDataVars md2 [
                                    ((fi,hji), jointH2{rotationL = (rotationL jointH2) * (axisAngle ((conjugate $ rj2 fI hjI) `rotate` axisKExtend) (negate angleH))}),
                                    ((fi,kji), jointK2{rotationL = (rotationL jointK2) * (axisAngle ((conjugate $ rj2 fI kjI) `rotate` axisKExtend) (pi - angleK))})
                                ]
                            jointH22 = j22 fi hji
                            jointK22 = j22 fi kji
                            jointA22 = j22 fi aji

                            -- 2.2 retract and correct foot
                            axisKRetract = (rj22 fI hjI) `rotate` (negate unitX)

                            md3@MotionDataVars{_rj=rj3} = modifyMotionDataVars md22 [
                                     ((fi,hji), jointH22{rotationL = (rotationL jointH22) * (axisAngle ((conjugate $ rj22 fI hjI) `rotate` axisKRetract) angleHT)})
                                    ,((fi,kji), jointK22{rotationL = (rotationL jointK22) * (axisAngle ((conjugate $ rj22 fI kjI) `rotate` axisKRetract) (angleKT - pi))})
                                ]

                            -- 3 correct foot
                            mdF@MotionDataVars{_j=jF} = modifyMotionDataVars md3 [
                                    -- target global ankel rotation is (rj fI ajI) we want this equal to the current global rotation (rj3 fI ajI = rj3 fI kjI * rotaionL ajI) hence
                                    --      rj3 fI kjI * rotaionL ajI = rj fI ajI
                                    --                    rotaionL ajI = (conjugate $ rj3 fI kjI) * (rj fI ajI)
                                    ((fi,aji), jointA22{rotationL = (conjugate $ rj3 fI kjI) * (rj fI ajI)})
                                ]


                lajI = pj . bj $ lfbI   -- left ankel joint
                lkjI = pj lajI          -- left knee joint
                lhjI = pj lkjI          -- left hip joint
                rajI = pj . bj $ rfbI   -- right ankel joint
                rkjI = pj rajI          -- right knee joint
                rhjI = pj rkjI          -- right hip joint








{-------------------------

        HIGH LEVEL
        FUNCTIONS

--------------------------}

getRot :: JointF -> Quat
getRot j = case parent j of
        Nothing  ->  rotationL $- j
        Just p   ->  (getRot p) * (rotationL $- j)

getPosEnd :: JointF -> Vec3
getPosEnd j = getPosEnd' (parent j) where
        getPosEnd' Nothing = offset $- j
        getPosEnd' (Just p) = (getPosEnd p) + ((getRot p) `rotate` (offset $- j))

getPosStart :: JointF -> Vec3
getPosStart j = maybe zero getPosEnd (parent j)

getPosCom :: JointF -> Vec3
getPosCom jf = case parent jf of
        Nothing  ->  getPosEnd jf
        Just _   ->  ((getPosStart jf) + (getPosEnd jf)) / 2 where


-- get the (min, max) coordinates of the system
frameBoundingBox :: Frame -> (Vec3, Vec3)
frameBoundingBox f = treeFoldNR minMax initMinMax f where
        initMinMax = let o = offset $- f in (o, o)
        minMax :: (Vec3, Vec3) -> JointF -> (Vec3, Vec3)
        minMax (cMin,cMax) v = let vPos = getPosEnd v in ((liftA2 min) cMin vPos, (liftA2 max) cMax vPos)



scaleAndTranslate :: Double -> Int -> MotionData -> MotionData
scaleAndTranslate targetHeight sampleFrameIndex md = mdTS where
    bb = frameBoundingBox $ (frames md)!sampleFrameIndex
    bbDiag = uncurry (+) bb
    bbC@(V3 _ cHeight _) = bbDiag / 2
    targetC = (V3 0 (cHeight + boneRadius) 0)
    translation = targetC - bbC
    scaleFactor = let V3 _ height _ = bbDiag in targetHeight / height

    mdT  = translate translation md
    mdTS = scale scaleFactor mdT


getInterpolatedFrame :: Double -> MotionData -> Frame
getInterpolatedFrame t md
    | i > snd (bounds (frames md))  = error "getInterpolatedFrame: i out of bounds"
    | j > snd (bounds (frames md))  = error "getInterpolatedFrame: j out of bounds"
    | otherwise = f where
        n = arraySize $ frames md
        i' = doMod (t / (frameTime md)) + 1 where
            nd = fromIntegral $ n
            doMod x
                | x < nd = x
                | otherwise = doMod (x - nd)
        i = min (n-1) (floor i')
        j = min (n-1) (i+1)
        a = (frames md) ! i
        b = (frames md) ! j
        f = set ((view f'){offset = ((offset $- a) + (offset $- b)) ^/ 2}) f'
        f' = treeZipWith interpJoint a b
        interp = i' - (fromIntegral i)
        interpJoint af bf = (view af){
                                rotationL  = Util.slerp (rotationL $- af) (rotationL $- bf) interp
                            }


isFootJoint :: JointF -> Bool
isFootJoint jf = (name $- jf) `elem` footJoints || (name $- (justParent jf)) `elem` footJoints where
    footJoints = ["LeftFoot", "LeftToe", "RightFoot", "RightToe"]
isToeJoint :: JointF -> Bool
isToeJoint jf = hasParent jf && (name $- (justParent jf)) `elem` footJoints where
    footJoints = ["LeftToe", "RightToe"]

-- Parsing code

hsToken = makeTokenParser haskellStyle

wordP = do
        spaces
        many1 (satisfy (not. isSpace))

floatP = do
        num <- wordP
        return (read num :: Double)

intP = do
        num <- wordP
        return (read num :: Int)

vec3 = do
        x <- floatP
        y <- floatP
        z <- floatP
        return (V3 x y z)

hierarchy = do
        string "HIERARCHY"
        spaces
        skel <- fmap head joints
        return skel
        where
                joints = many (joint <|> endSite)
                joint = do
                        string "ROOT" <|> string "JOINT"
                        spaces
                        name <- wordP
                        spaces
                        char '{'
                        spaces
                        string "OFFSET"
                        spaces
                        offset <- vec3
                        spaces
                        string "CHANNELS"
                        spaces
                        numC <- intP
                        spaces
                        channels <- channels numC
                        spaces
                        children <- joints
                        spaces
                        char '}'
                        spaces
                        return (Tree nullJoint{
                                name = name,
                                offset = offset,
                                channels = channels
                        } children)
                endSite = do
                        string endSiteName
                        spaces
                        char '{'
                        spaces
                        string "OFFSET"
                        spaces
                        offset <- vec3
                        spaces
                        char '}'
                        return (Tree nullJoint{
                                name = endSiteName,
                                offset = offset
                        } [])
                        where endSiteName = "End Site"
                channels 0 = return []
                channels numC = do
                        spaces
                        c <- wordP
                        cs <- channels (numC-1)
                        return ((toChan c) : cs) where
                                toChan "Xposition" = Xpos
                                toChan "Yposition" = Ypos
                                toChan "Zposition" = Zpos
                                toChan "Xrotation" = Xrot
                                toChan "Yrotation" = Yrot
                                toChan "Zrotation" = Zrot

newline' = try (string "\r\n") <|> (string "\n")

motion skel = do
        spaces
        string "MOTION"
        spaces
        string "Frames:"
        spaces
        numF <- intP
        spaces
        string "Frame Time:"
        spaces
        frameTime <- floatP
        spaces
        -- As most motion data has the first frame as a rest pose, we prune that frame
        frames <- fmap tail getFrames
        spaces
        return (frames, frameTime) where
                getFrames = endBy line newline' <?> "Frames"
                line = do
                        vs <- sepBy floatP (many1 (char ' '))
                        return (fromChanVals skel vs)
                      <?> "Line"


bvh = do
        spaces
        skel <- fmap toFocus hierarchy
        (frames,frameTime) <- motion skel
        return MotionData {
                frames = listArray (0,(length frames)-1) frames,
                baseSkeleton = skel,
                frameTime = frameTime
        }

raw_parseBVH :: String -> IO(MotionData)
raw_parseBVH filePath = do
        content <- readFile filePath
        out <- either (error $ "Failed to parse bvh file: " ++ filePath) (return) (parse bvh "" content)
        print "done Parsing"
        return out
