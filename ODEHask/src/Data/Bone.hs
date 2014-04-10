module Data.Bone where

import Util

data Bone = 
      Long Vec3 Vec3        -- start and end in global coordinates
    | Box Vec3 Vec3 Quat    -- side lengths, center, rotation