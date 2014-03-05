module Data.TreeF (
    Tree(..),
    TreeF(..),
    toFocus,
    map1,
    view,set,($-),
    parent,child0,rightSib,hasParent,root,
    treeZip,treeZipWith,
    treeMap',treeMap,treeMapM,treeMapM_,treeMapCon_,treeMapCon,
    treeFold,treeFoldNR
    
) where


import Data.Maybe
import Control.Monad


data Tree a   = Tree a [Tree a] deriving Show
data TreeFC a = TreeFC a [Tree a] [Tree a] deriving Show
data TreeF a  = TreeF (Tree a) [TreeFC a] deriving Show


instance Functor Tree where
    fmap fn (Tree el cs) = Tree (fn el) (map (fmap fn) cs) 

instance Functor TreeFC where
    fmap fn (TreeFC p l r) = TreeFC (fn p) (map (fmap fn) l) (map (fmap fn) r) 

instance Functor TreeF where
    fmap fn (TreeF tree tfcs) = TreeF (fmap fn tree) (map (fmap fn) tfcs)

tMap :: (a -> b) -> TreeF a -> TreeF b
tMap fn (TreeF t _) = toFocus $ tMap' t where
    tMap' (Tree x xc) = Tree (fn x) (map tMap' xc)

-- the function is given tree in version a and version b. ensure that only parent elements are accesed in the b
-- version and only child elements are accesed in the a version (else an error will occur) 
treeMap' :: (TreeF a -> b) -> TreeF a -> TreeF b
treeMap' fn tf = snd $ applyToChildren (tf, newRoot) where
        moveUp (af, bf)  = (fromJust $ parent af, fromJust $ parent bf)
        newRoot = toFocus (Tree (fn tf) [])
        applyToChildren (af,bf) = case child0 af of
                Nothing  -> (af,bf)
                Just ac  -> moveUp $ applyToRight (ac, appendChildAndFocus (fn ac) bf)
        applyToRight (af,bf) = case rightSib af' of
                Nothing   -> (af',bf')
                Just raf' -> applyToRight (raf', appendChildAndFocus (fn raf') (fromJust $ parent bf'))
              where
                (af',bf') = applyToChildren (af,bf)
       

treeMap :: (TreeF a -> TreeF a) -> TreeF a -> TreeF a
treeMap fn = treeMapCon_ fn' () where
    fn' _ tF = ((), fn tF)

-- This will return a zipped tree focus (with no parent elements)
-- this will match elements of the same traversal from the root
treeZip :: TreeF a -> TreeF b -> TreeF (a,b)
treeZip (TreeF t1 _) (TreeF t2 _) = toFocus $ treeZip' t1 t2 where
    treeZip' (Tree a ac) (Tree b bc) = Tree (a,b) (zipWith treeZip' ac bc)
    
treeZipWith :: (TreeF a -> TreeF b -> c) -> TreeF a -> TreeF b -> TreeF c
treeZipWith fn af bf = cf where
    abf = treeZip af bf
    cf = treeMap' fn' abf
    fn' abf' = fn af' bf' where
        af' = fmap fst abf'
        bf' = fmap snd abf'
        
    
toFocus :: Tree a -> TreeF a
toFocus r = TreeF r []

parent :: TreeF a -> Maybe (TreeF a)
parent (TreeF _ []) = Nothing 
parent (TreeF s ((TreeFC pj l r):ps)) =  Just $ TreeF (Tree pj (l ++ [s] ++ r)) ps

hasParent :: TreeF a -> Bool
hasParent = isJust . parent

root :: TreeF a -> TreeF a
root j = maybe j root (parent j)

child0 :: TreeF a -> Maybe (TreeF a)
child0 (TreeF (Tree _ []) _) = Nothing 
child0 (TreeF (Tree j (c:cs)) ps) =  Just $ TreeF c (TreeFC j [] cs : ps)

hasChild :: TreeF a -> Bool
hasChild = isJust . child0

appendChild :: a -> TreeF a -> TreeF a
appendChild newC tf = fromJust $ parent $ appendChildAndFocus newC tf

appendChildAndFocus :: a -> TreeF a -> TreeF a
appendChildAndFocus newC (TreeF (Tree j cs) ps) = TreeF (Tree newC []) (TreeFC j cs [] : ps)

rightSib :: TreeF a -> Maybe (TreeF a)
rightSib (TreeF _ []) = Nothing
rightSib (TreeF _ ((TreeFC _ _ []):_)) = Nothing
rightSib (TreeF s ((TreeFC pj l (r:rs)):ps)) =  Just $ TreeF r ((TreeFC pj (l ++ [s]) rs):ps)

hasRightSib :: TreeF a -> Bool
hasRightSib = isJust . rightSib

treeMapCon_ :: (d -> TreeF a -> (d,TreeF a)) -> d -> TreeF a -> TreeF a
treeMapCon_ fn d0 r =  snd $ treeMapCon fn d0 r
                
treeFold :: (d -> TreeF a -> d) -> d -> TreeF a -> d
treeFold fn d0 r =  fst $ treeMapCon fn' d0 r where
        fn' d j = (fn d j, j) 
        
-- like fold but skipping the root
treeFoldNR :: (d -> TreeF a -> d) -> d -> TreeF a -> d
treeFoldNR fn = treeFold fn' where
    fn' d jf
        | hasParent jf  = fn d jf
        | otherwise     = d
             
treeMapCon :: (d -> TreeF a -> (d,TreeF a)) -> d -> TreeF a -> (d, TreeF a)
treeMapCon fnC d f = applyToChildren $ applyToSelf (d,f) where
        fn = uncurry fnC
        moveUp (e, f)  = (e, fromJust $ parent f)
        applyToSelf = fn
        applyToChildren (d,r) = case child0 r of
                Nothing  -> (d,r)
                Just c   -> moveUp $ applyToRight (d,c)
        applyToRight (d,r) = case rightSib r' of
                Nothing  -> dr'
                Just r's  -> applyToRight (d',r's)
              where
                dr'@(d',r') = treeMapCon fnC d r
                
                 
treeMapM :: Monad m => (TreeF a -> m b) -> TreeF a -> m [b]
treeMapM fn j = sequence $ reverse $ treeFold (\ms j' -> (fn j'):ms) [] j

treeMapM_ :: (Functor m, Monad m) => (TreeF a -> m b) -> TreeF a -> m ()
treeMapM_ fn = void . (treeMapM fn)


    
-- Apply a function to the currently focused element
map1 :: (a -> a) -> TreeF a -> TreeF a
map1 fn focus = set (fn $- focus) focus

-- Get the currently focused element
view :: TreeF a -> a
view (TreeF (Tree el _) _) = el

-- Set (replace) the currently focused element
set :: a -> TreeF a -> TreeF a
set newEl (TreeF (Tree _ cs) fcs) = (TreeF (Tree newEl cs) fcs)
    
($-) :: (a -> b) -> TreeF a -> b
fn $- focus = fn $ view focus
