module Motion.MotionDataVars where

import Data.Array
import Data.List
import Data.Functor
import Linear hiding (_i,_j,_k,_w,trace)
import Data.Maybe

import Util
import Motion.Util
import Constants
import Data.TreeF as T
import Motion.Joint
import Motion.MotionData






type FJIndexedFrames = Int -> Int -> Joint

newtype FrameIx = F Int deriving (Show,Eq)
newtype JointIx = J Int deriving (Show,Eq)
newtype BoneIx = B Int deriving (Show,Eq)


data MotionDataVars = MotionDataVars {
    -- raw data
    _dt     :: Double,
    _jRaw   :: Array Int (Array Int Joint),
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
    _jHasChild :: JointIx -> Bool,
    _cjs     :: JointIx -> [JointIx],
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
    _lT     :: FrameIx -> Vec3,
    _pT     :: FrameIx -> Vec3,
    _hT     :: FrameIx -> Vec3,
    _pT'    :: FrameIx -> Vec3,
    _hT'    :: FrameIx -> Vec3,
    _zmp    :: FrameIx -> Vec3,
    _sp     :: FrameIx -> [Vec2],
    _zmpIsInSp :: FrameIx -> Bool,

    _soleCorners :: FrameIx -> ([Vec3],[Vec3]),
    _bByName :: String -> BoneIx,
    _jByName :: String -> JointIx,
    _footBs :: ((BoneIx,BoneIx),(BoneIx,BoneIx)),
    _footJs :: ((JointIx,JointIx,JointIx),(JointIx,JointIx,JointIx)),
    _isFootBone  :: BoneIx -> Bool
}

seqMDV :: MotionDataVars -> a -> a
seqMDV mdv@MotionDataVars{
    _fs = fs,
    _js = js,
    _j = j,
    _zmp = zmp,
    _sp = sp
} a =
    (seqF (show . sp))
    (seqF zmp)
    (seqFJ (\fi ji -> show $ j fi ji))
    a
    where
        seqF fn = seq $ foldl1' seq [fn fI | fI <- fs]
        seqFJ fn = seq $ foldl1' seq [fn fi ji | F fi <- fs, J ji <- js]

seqJMDV :: MotionDataVars -> a -> a
seqJMDV mdv@MotionDataVars{
    _fs = fs,
    _js = js,
    _j = j
} a = (seqFJ (\fi ji -> let joint = j fi ji in joint == joint)) a
    where
        seqF fn = seq $ foldl1' seq [fn fI | fI <- fs]
        seqFJ fn = seq $ foldl1' seq [fn fi ji | F fi <- fs, J ji <- js]

getMotionDataVariablesFromMotionData :: MotionData -> MotionDataVars
getMotionDataVariablesFromMotionData md = getMotionDataVariables (frameTime md) jRaw (baseSkeleton md) fN where

    fN = arraySize $ frames md
    jN = length $ flatten $ baseSkeleton md

    -- joints organized by frame then Joint
    jRaw :: Array Int (Array Int Joint)
    jRaw = array bndF [(fi, array bndJ [(ji, (flatten $ (frames md) ! fi) !! ji) | ji <- range bndJ]) | fi <- range bndF] where
        bndF = (0,fN-1)
        bndJ = (0,jN-1)


modifyMotionDataVars :: MotionDataVars -> [((Int,Int), Joint)] -> MotionDataVars
modifyMotionDataVars (mdv@MotionDataVars{_fN=fN}) = modifyMotionDataVarsFN mdv fN

modifyMotionDataVarsFN :: MotionDataVars -> Int -> [((Int,Int), Joint)] -> MotionDataVars
modifyMotionDataVarsFN (MotionDataVars{_dt=dt,_jN=jN,_baseSkeleton=bSkel,_jRaw=jRaw}) newfN jUpdates = getMotionDataVariables dt (jRaw // jUpdates') bSkel newfN where
    (_,uBoundFRaw) = bounds jRaw
    bndF = (0,uBoundFRaw)
    bndJ = (0,jN-1)
    jUpdatesPerFrameAssocs = accumArray (flip (:)) [] bndF (map (\((f,j),v) -> (f,(j,v))) jUpdates)
    jUpdates' = [ (fi, (jRaw!fi) // (jUpdatesPerFrameAssocs!fi)) | fi <- range bndF, jUpdatesPerFrameAssocs!fi /= [] ]

-- copy specific frames from one motion to the other
copyFrames :: [Int] -> MotionDataVars -> MotionDataVars -> MotionDataVars
copyFrames frames from@MotionDataVars{_js=js,_j=jFrom} to = modifyMotionDataVars to [((fi,ji), jFrom fi ji) | fi <- frames, (J ji) <- js]


--type JDataAndChanges = (Array (Int,Int) Joint, )

-- arguments: frameTimeDelta  (motion data by FrameIx and JointIx) (base skeletion)
-- Note that Joint index is from 0 and coresponds to flettening the base skeletoin
getMotionDataVariables :: Double -> Array Int (Array Int Joint) -> Frame -> Int -> MotionDataVars
getMotionDataVariables dt jRaw bskel fN = MotionDataVars {
    _dt     = dt,
    _jRaw   = jRaw,
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
    _jHasChild = jHasChild,
    _cjs    = cjs,
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
    _lT     = lT,
    _pT     = pT,
    _hT     = hT,
    _pT'    = pT',
    _hT'    = hT',
    _zmp    = zmp,
    _sp     = sp,
    _zmpIsInSp = zmpIsInSp,

    _soleCorners = soleCorners,
    _bByName = bByName,
    _jByName = jByName,
    _footBs = footBs,
    _footJs = footJs,
    _isFootBone = isFootBone
 } where

    memoizeJ :: (JointIx->b) -> (JointIx->b)
    memoizeJ fn = (\(J a) -> arrJ ! a) where
        arrJ = listArray bndJ (fn <$> js)

    memoizeB :: (BoneIx->b) -> (BoneIx->b)
    memoizeB fn = (\(B a) -> arrB ! a) where
        arrB = listArray bndB (fn <$> bs)

    memoizeF :: (FrameIx->b) -> (FrameIx->b)
    memoizeF fn = (\(F a) -> arrF ! a) where
        arrF = listArray bndF (fn <$> fs)

    memoizeFJ :: (FrameIx->JointIx->b) -> (FrameIx->JointIx->b)
    memoizeFJ fn = (\(F a) b -> (arrFmemJ ! a) b) where
        arrFmemJ = listArray bndF ((memoizeJ . fn) <$> fs)

    memoizeFB :: (FrameIx->BoneIx->b) -> (FrameIx->BoneIx->b)
    memoizeFB fn = (\(F a) b -> (arrFmemB ! a) b) where
        arrFmemB = listArray bndF ((memoizeB . fn) <$> fs)

    j a b = (jRaw!a)!b

    -- helper function to measure change over time (using current and previous frame)
    -- first frame simply copies the result from the second
    deriveFB :: (FrameIx -> BoneIx -> Vec3) -> (FrameIx -> BoneIx -> Vec3)
    deriveFB fn = derive_ where
        derive_ = memoizeFB derive__
        derive__ (fI@(F fi)) bI
            | fi > fst bndF     = ((fn fI bI) - (fn (F (fi-1)) bI)) ^/ dt
            | fN <= 1           = zero
            | otherwise         = derive_ (F (fi+1)) bI
    derive2FB :: (FrameIx -> BoneIx -> Vec3) -> (FrameIx -> BoneIx -> Vec3)
    derive2FB fn = derive_ where
        (lo,hi) = bndF
        derive_ = memoizeFB derive__
        derive__ (fI@(F fi)) bI
            | lo < fi && fi < hi    = ((fn (F (fi-1)) bI) - (2 * (fn fI bI)) + (fn (F (fi+1)) bI)) ^/ (dt**2)
            | fN <= 2               = zero
            | fi == lo              = derive__ (F (fi+1)) bI
            | fi == hi              = derive__ (F (fi-1)) bI
            | otherwise             = error $ "accessing frame " ++ show fi ++ " with n = " ++ show fN
--    deriveF :: (FrameIx -> Vec3) -> (FrameIx -> Vec3)
--    deriveF fn = derive_ where
--        derive_ = memoizeF derive__
--        derive__ (fI@(F fi))
--            | fi > fst bndF     = ((fn fI) - (fn (F (fi-1)))) ^/ dt
--            | otherwise         = derive_ (F (fi+1))



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
    pj (J ji) = pjArr ! ji
    pjArr = array bndJ [(ji, pj_ ji) | ji <- range bndJ] where
        pj_ ji = J pjIx where
            pJoint = (flatten $ T.treeMap' ((fmap T.view) . T.parent) (bskel)) !! (ji - (fst bndJ))
            pjIx = maybe (error "trying to access root's parent") (+ (fst bndJ)) (elemIndex pJoint (map (Just . jBase . J) (range bndJ)))

    -- child joints
    jHasChild :: JointIx -> Bool
    jHasChild = memoizeJ (\jI -> [] /= (cjs jI))
    cjs :: JointIx -> [JointIx]
    cjs = memoizeJ cj_ where
        cj_ jI =[ ujI | ujI <- tail js, jI == pj ujI ]

    --
    -- Bone (uses on baseSkeleton)
    --
--    bHasParent :: BoneIx -> Bool
--    bHasParent = memoizeB (\bI -> let jI = bj bI in jHasParent jI && jHasParent (pj jI) ) -- only root (at first index) has no parent
    pb :: BoneIx -> BoneIx
    pb = jb . pj . bj

    -- bone density
    d :: BoneIx -> Double
    -- make limbs lighter
    d bI = boneDensity / (depth (bj bI))  where
        -- depth from root
        depth jI
            | jHasParent jI = 1 + (depth $ pj jI)
            | otherwise     = 1

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

    -- bone linear velocity
    l :: FrameIx -> BoneIx -> Vec3
    l = deriveFB xb

    -- CoM linear Velocity
    lT :: FrameIx -> Vec3
    lT fI = (sum [(m bI) *^ (l fI bI) | bI <- bs]) ^/ mT

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

    jByName :: String -> JointIx
    jByName jn = fromMaybe (error $ "Bone with name\"" ++ jn ++ "\" could not be found") (lookup jn (zip (map (name . jBase) js) js))

    soleCorners :: FrameIx -> ([Vec3],[Vec3])
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
            ((map (l2g lfI) footCorners) ++ (map (l2g ltI) toeCorners),
             (map (l2g rfI) footCorners) ++ (map (l2g rtI) toeCorners))

        l2g :: BoneIx -> Vec3 -> Vec3
        l2g bI offset = (xb fI bI) + (rb fI bI `rotate` offset)

    -- support Polygon (must be touching the ground else Nothing)
    sp :: FrameIx -> [Vec2]
    sp fI
        | length contactPoints <= 1 = []
        | otherwise             = hull
        where
            hull = convexHull contactPoints
            contactPoints = map (\(V3 x _ z) -> V2 x z) (filter (\(V3 _ y _) -> y <= floorContactThreshold) (uncurry (++) (soleCorners fI)))

    zmpIsInSp :: FrameIx -> Bool
    zmpIsInSp fi' = polyContainsPoint (sp fi') (toXZ (zmp fi'))

