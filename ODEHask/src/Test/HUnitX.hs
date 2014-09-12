module Test.HUnitX where


import Data.Vector (Vector)
import qualified Data.Vector as V
import Test.HUnit
import Control.Monad (unless)
import Linear


(~<?) :: (Show a, Ord a) => a -> a -> Test
expectedFloor ~<? actual = TestCase (expectedFloor @<? actual)

(~?<) :: (Show a, Ord a) => a -> a -> Test
actual ~?< expectedCeil = TestCase (actual @?< expectedCeil)

(@<?) :: (Show a, Ord a) => a -> a -> Assertion
expectedFloor @<? actual = assertLeftExpectedOp (<) "<" expectedFloor actual

(@?<) :: (Show a, Ord a) => a -> a -> Assertion
actual @?< expectedCeil = assertRightExpectedOp (<) "<" actual expectedCeil

class (Show a) => AboutEq a where
    (~=) :: a -> a -> Bool
    (!~=) :: a -> a -> Bool
    u !~= v = not $ u ~= v 

    (@?~=:) :: a -> a -> String -> Assertion
    (@?~=:) actual expected lab = unless (actual ~= expected) (assertFailureEq lab actual expected)
    (@~=?:) :: a -> a -> String -> Assertion
    (@~=?:) = flip (@?~=:)
    
    (@?!~=:) :: a -> a -> String -> Assertion
    (@?!~=:) actual expected lab = unless (not $ actual ~= expected) (assertFailureEq lab actual expected)
    (@!~=?:) :: a -> a -> String -> Assertion
    (@!~=?:) = flip (@?!~=:)

    (@?~=) :: a -> a -> Assertion
    a @?~= e = (a @?~=: e) ""
    (@~=?) :: a -> a -> Assertion
    (@~=?) = flip (@?~=)
    
    (@?!~=) :: a -> a -> Assertion
    a @?!~= e = (a @?!~=: e) ""
    (@!~=?) :: a -> a -> Assertion
    (@!~=?) = flip (@?!~=)


(~==) :: (Fractional a, Ord a) => a -> a -> Bool
a ~== b = (abs $ a - b) < (fromRational epsilon)

instance AboutEq Double where
    (~=) = (~==)

instance (AboutEq a) => AboutEq (V3 a) where
    (V3 a b c) ~= (V3 d e f) = [a,b,c] ~= [d,e,f]

instance (AboutEq a, AboutEq b) => AboutEq (a,b) where
    (a1,b1) ~= (a2,b2) = (a1 ~= a2) && (b1 ~= b2)

instance (AboutEq a) => AboutEq [a] where
    (x:xs) ~= (y:ys) = (x ~= y)  &&  (xs ~= ys)
    [] ~= [] = True
    [] ~= _ = False
    _ ~= [] = False
    
instance (AboutEq a) => AboutEq (Vector a) where
    a ~= b = (V.length a == V.length b)  &&  (V.and (V.zipWith (~=) a b))
    a !~= b = (V.length a /= V.length b)  ||  (V.or (V.zipWith (!~=) a b))



epsilon = 0.00000001
    
assertFailureEq lab actual expected = assertFailure msg where
    msg = (if lab == "" then "" else lab ++ "\n") ++ "expected: ≈ " ++ show expected ++ "\n but got:   " ++ show actual

assertLeftExpectedOp op opStr expected actual = unless (expected `op` actual) (assertFailure msg) where
    msg = "expected: " ++ show expected ++ " " ++ opStr ++ " ? \n but got: " ++ show actual
    
assertRightExpectedOp op opStr actual expected = unless (actual `op` expected) (assertFailure msg) where
    msg = "expected: ? " ++ opStr ++ " " ++ show expected ++ "\n but got: " ++ show actual

















