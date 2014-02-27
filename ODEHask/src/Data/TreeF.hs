module Data.TreeF (
    Tree(..),
    TreeF(..),
    toFocus,
    map1,
    view,set,($-),
    parent,child0,rightSib,hasParent,root,
    treeMap,treeMapM,treeMapM_,treeMapCon_,treeMapCon,treeFold
    
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
    
treeMap :: (TreeF a -> TreeF a) -> TreeF a -> TreeF a
treeMap fn = treeMapCon_ fn' () where
    fn' _ tF = ((), fn tF)

toFocus :: Tree a -> TreeF a
toFocus r = TreeF r []

parent :: TreeF a -> Maybe (TreeF a)
parent (TreeF _ []) = Nothing 
parent (TreeF s ((TreeFC pj l r):ps)) =  Just $ TreeF (Tree pj (l ++ [s] ++ r)) ps

root :: TreeF a -> TreeF a
root j = maybe j root (parent j)

hasParent :: TreeF a -> Bool
hasParent = isJust . parent

child0 :: TreeF a -> Maybe (TreeF a)
child0 (TreeF (Tree _ []) _) = Nothing 
child0 (TreeF (Tree j (c:cs)) ps) =  Just $ TreeF c (TreeFC j [] cs : ps)

rightSib :: TreeF a -> Maybe (TreeF a)
rightSib (TreeF _ []) = Nothing
rightSib (TreeF _ ((TreeFC _ _ []):_)) = Nothing
rightSib (TreeF s ((TreeFC pj l (r:rs)):ps)) =  Just $ TreeF r ((TreeFC pj (l ++ [s]) rs):ps)

treeMapCon_ :: (d -> TreeF a -> (d,TreeF a)) -> d -> TreeF a -> TreeF a
treeMapCon_ fn d0 r =  snd $ treeMapCon fn d0 r
                
treeFold :: (d -> TreeF a -> d) -> d -> TreeF a -> d
treeFold fn d0 r =  fst $ treeMapCon fn' d0 r where
        fn' d j = (fn d j, j) 
             
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
                
                
treeMapM :: Monad m => (TreeF a -> m a) -> TreeF a -> m [a]
treeMapM fn j = sequence $ treeFold (\ms j' -> (fn j'):ms) [] j

treeMapM_ :: (Functor m, Monad m) => (TreeF a -> m a) -> TreeF a -> m ()
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
