module MotionData where

import Control.Applicative (liftA2)
import Data.List
import Data.Array
import Data.TreeF as T
import Linear
import Util
import Constants


import Data.Char
import Text.Parsec
import Text.Parsec.Token
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


newtype FrameIx = F Int
newtype JointIx = J Int
newtype BoneIx = B Int

data MotionDataVars = MotionDataVars {
    dt     :: Double,
    fN     :: Int,
    jN     :: Int,
    bN     :: Int,
    js     :: [JointIx],
    bs     :: [BoneIx],
    fs     :: [FrameIx],
    j      :: FrameIx -> JointIx -> Joint,
    pj     :: JointIx -> JointIx,
    pb     :: BoneIx -> BoneIx,
    d      :: BoneIx -> Double,
    m      :: BoneIx -> Double,
    i      :: BoneIx -> M3,
    rj     :: FrameIx -> JointIx -> Quat,
    xj     :: FrameIx -> JointIx -> Vec3,
    rb     :: FrameIx -> BoneIx -> Quat,
    xb     :: FrameIx -> BoneIx -> Vec3,
    xsb    :: FrameIx -> BoneIx -> Vec3,
    xeb    :: FrameIx -> BoneIx -> Vec3,
    l      :: FrameIx -> BoneIx -> Vec3,
    w      :: FrameIx -> BoneIx -> Vec3,
    l'     :: FrameIx -> BoneIx -> Vec3,
    w'     :: FrameIx -> BoneIx -> Vec3,
    p      :: FrameIx -> BoneIx -> Vec3,
    h      :: FrameIx -> BoneIx -> Vec3,
    p'     :: FrameIx -> BoneIx -> Vec3,
    h'     :: FrameIx -> BoneIx -> Vec3,
    mT     :: Double,
    comT   :: FrameIx -> Vec3,
    pT     :: FrameIx -> Vec3,
    hT     :: FrameIx -> Vec3,
    pT'    :: FrameIx -> Vec3,
    hT'    :: FrameIx -> Vec3,
    zmp    :: FrameIx -> Vec3,
    
    isFootBone  :: BoneIx -> Bool
};

getMotionDataVariables :: MotionData -> MotionDataVars
getMotionDataVariables md = MotionDataVars {
    dt     = dt,
    fN     = fN,
    jN     = jN,
    bN     = bN,
    js     = js,
    bs     = bs,
    fs     = fs,
    j      = j,
    pj     = pj,
    pb     = pb,
    d      = d,
    m      = m,
    i      = i,
    rj     = rj,
    xj     = xj,
    rb     = rb,
    xb     = xb,
    xsb    = xsb,
    xeb    = xeb,
    l      = l,
    w      = w,
    l'     = l',
    w'     = w',
    p      = p,
    h      = h,
    p'     = p',
    h'     = h',
    mT     = mT,
    comT   = comT,
    pT     = pT,
    hT     = hT,
    pT'    = pT',
    hT'    = hT',
    zmp    = zmp,
    
    isFootBone = isFootBone
} where
    

    memoizeJ :: (JointIx->b) -> (JointIx->b)
    memoizeJ fn = (\(J a) -> table ! a) where
        table = (array bndJ [ (u, fn (J u)) | u <- range bndJ ])

    memoizeB :: (BoneIx->b) -> (BoneIx->b)
    memoizeB fn = (\(B a) -> table ! a) where
        table = (array bndB [ (u, fn (B u)) | u <- range bndB ])

    memoizeF :: (FrameIx->b) -> (FrameIx->b)
    memoizeF fn = (\(F a) -> table ! a) where
        table = (array bndF [ (u, fn (F u)) | u <- range bndF ])
        
    memoizeFJ :: (FrameIx->JointIx->b) -> (FrameIx->JointIx->b)
    memoizeFJ fn = (\(F f) (J a) -> table ! (f,a)) where
        table = (array bndFJ [ ((f,u), fn (F f) (J u)) | (f,u) <- range bndFJ ])

    memoizeFB :: (FrameIx->BoneIx->b) -> (FrameIx->BoneIx->b)
    memoizeFB fn = (\(F f) (B a) -> table ! (f,a)) where
        table = (array bndFB [ ((f,u), fn (F f) (B u)) | (f,u) <- range bndFB ])
    
    
    
    --
    -- Data
    --
    
    dt = frameTime md 
    
    bndFJ = ((1,1),(fN,jN))
    bndFB = ((1,1),(fN,bN))
    bndF = (1,fN)
    bndJ = (1,jN)
    bndB = (1,bN)
    
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
    
    -- frame count
    fN = arraySize $ frames md
    
    -- joint count
    jN = length $ flatten $ baseSkeleton md
    
    -- bone count
    bN = jN - 1
    
    -- joints organized by frame
    j :: FrameIx -> JointIx -> Joint 
    j = memoizeFJ (\(F fI) (J jI) -> (flatten $ frames md ! fI) !! (jI - (fst bndJ)))
    
    -- joints of baseSkeleton
    jBase :: JointIx -> Joint 
    jBase = memoizeJ (\(J jI) -> (flatten $ baseSkeleton md) !! (jI - (fst bndJ)))
    
    -- bones (a really just the set of all joints that have parents)
--    b :: FrameIx -> BoneIx -> Bone
--    b = j . bj
    
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
            pJoint = (flatten $ T.treeMap' ((fmap T.view) . T.parent) (baseSkeleton md)) !! (ji - (fst bndJ))
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
    
    -- joint global rotation
    rj :: FrameIx -> JointIx -> Quat
    rj = memoizeFJ rj_ where
        rj_ fI jI
            | jHasParent jI = (rj fI $ pj jI) * (rotationL $ jBase jI)
            | otherwise     = rotationL $ j fI jI
            
    xj :: FrameIx -> JointIx -> Vec3
    xj = memoizeFJ xj_ where
        xj_ fI jI
            | jHasParent jI = (xj fI $ pj jI) + ((rj fI $ pj jI) `rotate` (offset $ jBase jI))
            | otherwise     = offset $ j fI jI
        
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
            | fi > fst bndF     = ((fn (F (fi-1)) bI) - (fn fI bI)) ^/ dt
            | otherwise         = derive_ (F (fi+1)) bI
    deriveF :: (FrameIx -> Vec3) -> (FrameIx -> Vec3)
    deriveF fn = derive__ where
        derive_ = memoizeF derive__
        derive__ (fI@(F fi))
            | fi > fst bndF     = ((fn (F (fi-1))) - (fn fI)) ^/ dt
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
    l' = deriveFB l
    
    -- bone angular acc
    w' :: FrameIx -> BoneIx -> Vec3
    w' = deriveFB w
    
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
    p' = deriveFB p
    
    -- bone angular momentum rate
    h' :: FrameIx -> BoneIx -> Vec3
    h' = deriveFB h
    
    --
    -- Frame
    --
    
    -- total center of mass
    comT = memoizeF comT_ where
        comT_ fI = (sum (map weightedCom bs)) ^/ mT where
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
    pT' = deriveF pT
    
    -- total angular momentum about the origin
    hT' :: FrameIx -> Vec3
    hT' = deriveF hT
            
    -- ZMP
    -- assume that ZMP is on the floor (y component is 0)
    -- to preserve right-handedness we must swap (ZMP paper axis -> my code axis): z -> y, y -> x, x -> y)
    zmp :: FrameIx -> Vec3
    zmp = memoizeF zmp_ where
        zmp_ fI = V3
            (((mT * g * comX) + h'Z) / ((mT * g) + p'Y))
            0
            (((mT * g * comZ) - h'X) / ((mT * g) + p'Y))
            where
                g = gravityAcc
                (V3 comX _ comZ) = comT fI
                (V3 _ p'Y _) = pT' fI
                (V3 h'X _ h'Z) = hT' fI
                
    isFootBone :: BoneIx -> Bool
    isFootBone bI = ((name . jBase . pj . bj) bI) `elem` footJoints where
        footJoints = ["LeftFoot", "LeftToe", "RightFoot", "RightToe"]
    


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
preProcess windowSize md = mdF where
    n = fromInteger windowSize
    newFrames = map avg (windows (elems $ frames md))
    mdF = md{
             frames = listArray (1, length newFrames) newFrames 
        }
    windows fs
        | length lfsn == n  = lfsn : windows (tail fs)
        | otherwise         = []
        where
            lfsn = take n fs
    avg fs = fmap avgJ (foldl1 (treeZipWith (\a b -> (view a) ++ (view b))) (map (fmap (:[])) fs))
    avgJ :: [Joint] -> Joint
    avgJ js = (head js) {
        offset   = avgPos,
        rotationL = avgRotL
    }  where
        avgPos = (foldl1 (+) (map offset js)) ^/ (fromIntegral n)
        avgRotL = avgRot (map rotationL js)


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
getInterpolatedFrame t md = f where
    n = arraySize $ frames md
    i = doMod (t / (frameTime md)) + 1 where
        nd = fromIntegral $ n
        doMod x
            | x <= nd = x
            | otherwise = doMod (x - nd)
    
    a = (frames md) ! (floor i)
    b = (frames md) ! if ceiling i > n then 0 else (ceiling i)
    f = treeZipWith interpJoint a b
    interp = i - (fromIntegral (floor i))
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
                frames = listArray (1,length frames) frames,
                baseSkeleton = skel,
                frameTime = frameTime
        }

raw_parseBVH :: String -> IO(MotionData)
raw_parseBVH filePath = do
        content <- readFile filePath
        out <- either (error $ "Failed to parse bvh file: " ++ filePath) (return) (parse bvh "" content)
        print "done Parsing"
        return out