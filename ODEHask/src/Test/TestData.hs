module Test.TestData where

import Test.HUnit
import qualified Data.Vector as V

import System.Random
import Util
import Linear hiding (_w,_l)



testDataVec3Corner :: [Vec3]
testDataVec3Corner = [
        V3    0    0    0,
        V3    1    1    1,
        V3  (-1) (-1) (-1),
        V3    1    0    0,
        V3  (-1)   1    0,
        V3    0  (-1)   1,
        V3    0    0  (-1),
        V3    1    1    0,
        V3    0    1    1,
        V3    1    0    1,
        V3 100000 100000 100000,
        V3  0.001 0.001 0.001
    ]

testDataVec3 :: [Vec3]
testDataVec3 = testDataVec3Corner ++ (take 100 $ randomVec3s 104957134807)

testDataVector6 :: [V.Vector Double]
testDataVector6 = [V.fromList [a,b,c,d,e,f] | (V3 a b c) <- testDataVec3Corner,  (V3 d e f) <- testDataVec3Corner] ++ take 100 (randomVector6s 5098247540252)

randomVec3s seed = toVec3s (randoms (mkStdGen seed)) where
    toVec3s (x:y:z:rest) = (V3 x y z) : (toVec3s rest)

randomVector6s seed = toVector6s (randoms (mkStdGen seed)) where
    toVector6s (a:b:c:d:e:f:rest) = (V.fromList [a,b,c,d,e,f]) : (toVector6s rest)


