module Data.TreeF (
    Tree,
    TreeF,
    STree(..),
    STreeF(..),
    toFocus,
    map1,
    view,set,($-),
    parent,child0,rightSib,hasParent,root,
    tMap,    
    treeZip,
    treeMap,treeMapM,treeMapM_,treeMapCon_,treeMapCon,treeFold
) where


import Data.Maybe
import Control.Monad


-- types with prefix S represent Trees with "Split" types. These are intermediate trees used when applying a map
-- that changes the type of the tree

-- Tree element childTrees
data STree e c   = Tree e [Tree c] deriving Show
type Tree a   = STree a a
-- TreeFC parentElement leftSiblings rightSiblings
data STreeFCS l p r = TreeFC [TreeFCS p] p [Tree l] [Tree r] deriving Show
type TreeFCS a = STreeFCS a a a
-- TreeF focusedTree crumbs
-- TreeF (elemAndChildren) (ListOf_parentElemAndSibs)
-- TreeF (child tree with focused element as root) (the rest of the tree)
-- STreeF parent leftSibs focusedEl rightSibs children
data STreeF p l f r c  = TreeF (STree f c) (STreeFCS l p r) deriving Show -- used when modifying a tree
type TreeF a  = STreeF a a a a a

{-
instance Functor STree where
    fmap fn (Tree el cs) = Tree (fn el) (map (fmap fn) cs) 

instance Functor TreeFC where
    fmap fn (TreeFC p l r) = TreeFC (fn p) (map (fmap fn) l) (map (fmap fn) r) 

instance Functor STreeF where
    fmap fn (TreeF tree tfcs) = TreeF (fmap fn tree) (map (fmap fn) tfcs)

tMap :: (a -> b) -> TreeF a -> TreeF b
tMap fn (TreeF t _) = toFocus $ tMap' t where
    tMap' (Tree x xc) = Tree (fn x) (map tMap' xc)
    -}

treeMap :: (TreeF a -> TreeF a) -> TreeF a -> TreeF a
treeMap fn = treeMapCon_ fn' () where
    fn' _ tF = ((), fn tF)

-- This will return a zipped tree focus (with no parent elements)
-- this will match elements of the same traversal from the root
treeZip :: TreeF a -> TreeF b -> TreeF (a,b)
treeZip (TreeF t1 _) (TreeF t2 _) = toFocus $ treeZip' t1 t2 where
    treeZip' (Tree a ac) (Tree b bc) = Tree (a,b) (zipWith treeZip' ac bc)

toFocus :: Tree a -> TreeF a
toFocus r = TreeF r []

parent :: TreeF a -> Maybe (TreeF a)
parent (TreeF _ []) = Nothing 
parent (TreeF s (TreeFC ps pj l r)) =  Just $ TreeF (Tree ps pj (l ++ [s] ++ r))

root :: TreeF a -> TreeF a
root j = maybe j root (parent j)

hasParent :: TreeF a -> Bool
hasParent = isJust . parent

child0 :: STreeF a a a a b -> Maybe (STreeF a b b b b)
child0 (TreeF (Tree _ []) _) = Nothing 
child0 (TreeF (Tree j (c:cs)) ps) =  Just $ TreeF c (TreeFC ps j [] cs)

rightSib :: TreeF a -> Maybe (TreeF a)
rightSib (TreeF _ []) = Nothing
rightSib (TreeF _ (TreeFC _ _ _ [])) = Nothing
rightSib (TreeF s (TreeFC ps pj l (r:rs))) =  Just $ TreeF r (TreeFC ps pj (l ++ [s]) rs)

treeMapCon_ :: (d -> TreeF a -> (d,TreeF a)) -> d -> TreeF a -> TreeF a
treeMapCon_ fn d0 r =  snd $ treeMapCon fn d0 r
                
treeFold :: (d -> TreeF a -> d) -> d -> TreeF a -> d
treeFold fn d0 r =  fst $ treeMapCon fn' d0 r where
        fn' d j = (fn d j, j) 
             
treeMapCon :: (d -> STreeF a a a b b -> (d, STreeF  b a)) -> d -> TreeF a -> (d, TreeF b)
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
{-          
-- map that allows changing of the type of tree.
treeMapCon' :: (d -> STreeF b a a -> (d, STreeF b b a)) -> d -> TreeF a -> (d, TreeF b)
treeMapCon' fnC d f = applyToChildren $ applyToSelf (d,f) where
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
 -}              
                 
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
