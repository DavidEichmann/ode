module Util where

import Linear hiding (slerp,trace)
import Linear.Matrix hiding (trace)
import Data.Array
import Data.Maybe
import Data.List (sort)

import Debug.Trace

type Vec2 = V2 Double
type Vec3 = V3 Double
type M3 = M33 Double
type Quat = Quaternion Double

unitX :: Vec3
unitX = V3 1 0 0

unitY :: Vec3
unitY = V3 0 1 0

unitZ :: Vec3
unitZ = V3 0 0 1

identity :: Quat
identity = axisAngle unitZ 0

degreeToRadian :: Fractional a => a -> a
degreeToRadian d = 0.01745329251 * d

zToDirQuat :: Vec3 -> Quat
zToDirQuat = dirToDirQuat unitZ

xToDirQuat :: Vec3 -> Quat
xToDirQuat = dirToDirQuat unitX
{-xToDirQuat (V3 x 0 0)
            | x < 0     = axisAngle unitY pi
            | otherwise = identity
xToDirQuat dir = axisAngle axis angle where
    dirU = normalize dir
    axis = normalize $ unitX `cross` dirU
    angle = acos (dirU `dot` unitX) -}

dirToDirQuat :: Vec3 -> Vec3 -> Quat
dirToDirQuat a b
            | cross a b == 0    =  axisAngle unitY pi --if dot a b > 0 then identity else axisAngle unitY pi
            | otherwise         = axisAngle axis angle where
                                    aU = normalize a
                                    bU = normalize b
                                    axis = normalize $ aU `cross` bU
                                    angle = acos (bU `dot` aU)

-- average a number of rotations, this is done by repeated application of Slerp
avgRot :: [Quat] -> Quat
avgRot [] = identity
avgRot qs = fst . head . avgRot' $ map (flip (,) 1) qs where
    avgRot' :: [(Quat,Integer)] -> [(Quat,Integer)]
    avgRot' [] = []
    avgRot' (qw:[]) = [qw]
    avgRot' (qw1:qw2:qws) = avgRot' $ (merge qw1 qw2) : (avgRot' qws)
    merge (q1,w1) (q2,w2) = (slerp q1 q2 (fromInteger w2 / fromInteger (w1+w2)), w1+w2)

slerp :: RealFloat a => Quaternion a -> Quaternion a -> a -> Quaternion a
slerp q p t
  | 1.0 - cosphi < 1e-8 = q
  | otherwise           = ((sin ((1-t)*phi) *^ q) + (sin (t*phi)) *^ (f p)) ^/ (sin phi)
  where
    dqp = dot q p
    (cosphi, f) = if dqp < 0 then (-dqp, negate) else (dqp, id)
    phi = acos cosphi


arraySize :: (Ix i) => Array i e -> Int
arraySize = rangeSize . bounds

memoize :: (Ix i) => (i,i) -> (i->b) -> (i->b)
memoize bnd fn = (\a -> table ! a) where
    table = (array bnd [ (u, fn u) | u <- range bnd ])

vx :: V3 a -> a
vx (V3 x _ _) = x

vy :: V3 a -> a
vy (V3 _ y _) = y

vz :: V3 a -> a
vz (V3 _ _ z) = z

toXZ :: Vec3 -> Vec2
toXZ (V3 x _ z) =  V2 x z

xz2x0z :: Vec2 -> Vec3
xz2x0z (V2 x z) = V3 x 0 z

v2tov3 :: Num a => V2 a -> V3 a
v2tov3 (V2 x y) = V3 x y 0

v3tov2 :: V3 a -> V2 a
v3tov2 (V3 x y _) = V2 x y

norm2 :: (Num a, Metric f) => f a -> a
norm2 x = dot x x


-- point intersects simple polygon
-- http://en.wikipedia.org/wiki/Point_in_polygon   Ray casting algorithm
polyContainsPoint :: [Vec2] -> Vec2 -> Bool
polyContainsPoint [] _ = False
polyContainsPoint (pp:[]) p = pp == p
polyContainsPoint poly p@(V2 px py) = odd $ length rayIntersects where
    rayIntersects = catMaybes $ map (intersectLineSegs ray) edges
    edges = zip poly ((tail poly) ++ [head poly])
    ray = (V2 (minX-1) py, p)
    minX = minimum (map (\(V2 x _) -> x) poly)


-- points where a polygon's border intersects a line segment
polyEdgeLineSegIntersect :: [Vec2] -> (Vec2, Vec2) -> [Vec2]
polyEdgeLineSegIntersect [] _ = []
polyEdgeLineSegIntersect (_:[]) _ = []  -- ignore degenerate polygon intersections
polyEdgeLineSegIntersect poly ls = catMaybes $ map (intersectLineSegs ls) edges where
    edges = zip poly ((tail poly) ++ [head poly])

-- linesegment intersects linesegment
-- find intersect.... A solution is only returned if there is exactly 1 intersection point
intersectLineSegs ::  (Vec2, Vec2) -> (Vec2, Vec2) -> Maybe Vec2
intersectLineSegs (p',pr') (q',qs')
    | rxs /= 0 &&
        0 <= t && t <= 1 &&
        0 <= u && u <= 1       = Just (v3tov2 (p + (t *^ r)))
    | otherwise = Nothing
    where
        p:pr:q:qs:_ = map v2tov3 [p',pr',q',qs']
        r = pr - p
        s = qs - q
        rxs = r `cross` s
        rxsZ = vz rxs
        t = (vz ((q - p) `cross` s)) / rxsZ
        u = (vz ((q - p) `cross` r)) / rxsZ

-- sort/filter an array of points into convex hull order (first and last elements are NOT the same) copied from wikibooks
--  http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
-- Implements the monotone chain algorithm
convexHull :: [Vec2] -> [Vec2]
convexHull [] = []
convexHull [p] = [p]
convexHull points = lower ++ upper where
    sorted = sort points
    lower = chain sorted
    upper = chain (reverse sorted)

    -- Checks if it's shortest to rotate from the OA to the OB vector in a clockwise
    -- direction.
    clockwise :: Vec2 -> Vec2 -> Vec2 -> Bool
    clockwise o a b = (a - o) `cross` (b - o) <= 0

    -- 2D cross product.
    cross :: Vec2 -> Vec2 -> Double
    cross (V2 x1 y1) (V2 x2 y2) = x1 * y2 - x2 * y1


    chain :: [Vec2] -> [Vec2]
    chain = go []
      where
        -- The first parameter accumulates a monotone chain where the most recently
        -- added element is at the front of the list.
        go :: [Vec2] -> [Vec2] -> [Vec2]
        go acc@(r1:r2:rs) (x:xs) =
          if clockwise r2 r1 x
            -- Made a clockwise turn - remove the most recent part of the chain.
            then go (r2:rs) (x:xs)
            -- Made a counter-clockwise turn - append to the chain.
            else go (x:acc) xs
        -- If there's only one point in the chain, just add the next visited point.
        go acc (x:xs) = go (x:acc) xs
        -- No more points to consume - finished!  Note: the reverse here causes the
        -- result to be consistent with the other examples (a ccw hull), but
        -- removing that and using (upper ++ lower) above will make it cw.
        go acc [] = reverse $ tail acc

buildArray :: Ix a => (a,a) -> (a -> b) -> Array a b
buildArray bnd fn = array bnd [( ix,
                fn ix
            ) | ix <- range bnd]

matrixMult :: Array (Int,Int) Double -> Array (Int,Int) Double -> Array (Int,Int) Double
matrixMult a b
    | m /= mb   = error ("Trying to multiply matriceis of incompatible sizes:" ++ (show $ bounds a) ++ (show $ bounds b))
    | otherwise = ab
    where
        ((ari,aci),(arf,acf)) = bounds a
        ((bri,bci),(brf,bcf)) = bounds b
        n = 1 + (arf-ari)
        m = 1 + (acf-aci)
        mb = 1 + (brf-bri)
        p = 1 + (bcf-bci)

        ab = buildArray ((0,0),(n-1,p-1)) (\(r,c) ->
            sum [ (a!(r+ari,i+aci)) * (b!(i+bri,c+bci)) | i <- [0..m-1]]
         )

matrixTranspose ::  Array (Int,Int) a ->  Array (Int,Int) a
matrixTranspose m = array newBnds [((r,c), m!(rI+c,cI+r))  | (r,c) <- range newBnds] where
    newBnds = ((0,0),(cF-cI,rF-rI))
    ((rI,cI),(rF,cF)) = bounds m

matrixAddition ::  Num a => Array (Int,Int) a ->  Array (Int,Int) a ->  Array (Int,Int) a
matrixAddition a b = array newBnds [((r,c), (a!(r,c)) + (b!(r,c))) | (r,c) <- range newBnds] where
    newBnds = ((max rIa rIb, max cIa cIb),(min rFa rFb, min rFa rFb))
    ((rIa,cIa),(rFa,cFa)) = bounds a
    ((rIb,cIb),(rFb,cFb)) = bounds b

matrix :: Int -> Int -> (Int -> Int -> a) -> Array (Int,Int) a
matrix rs cs fn = listArray bnds [ fn r c | (r,c) <- range bnds] where
    bnds = ((0,0),(rs-1,cs-1))

showMatrix :: Show a => Array (Int, Int) a -> String
showMatrix m = concat [(show $ m!(r,c)) ++ (if c == cF && r /= rF then "\n" else "  \t") | (r,c) <- range bnds] where
    bnds@((_,_),(rF,cF)) = bounds m

applyUntil :: (a -> a) -> (a -> Bool) -> a -> a
applyUntil = applyUntilN (-1)

applyUntilN ::  Int -> (a -> a) -> (a -> Bool)-> a -> a
applyUntilN n fn ck val
    | ck val                = val
    | n == 0                = trace ("untill finished early") val
    | otherwise             = applyUntilN (n-1) fn ck (fn val)

traceShowV :: Show a => a -> a
traceShowV v = traceShow v v

traceShowVM :: Show a => String -> a -> a
traceShowVM msg v = traceShow (msg ++ (show v)) v

iterateMN :: (Monad m, Integral i) => (a -> m a) -> a -> i -> m [a]
iterateMN fn x n
    | n == 0    = return []
    | otherwise = do
        x' <- fn x
        xs <- iterateMN fn x' (n-1)
        return $ x : xs


