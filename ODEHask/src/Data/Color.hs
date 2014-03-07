module Data.Color (
    Color(..),
    toRGBA
) where

import Constants

data Color =  Red
            | RedA Double
            | Green
            | GreenA Double
            | Blue
            | BlueA Double
            | Yellow
            | YellowA Double
            | White
            | WhiteA Double
            | Black
            | BlackA Double
            | Orange
            | OrangeA Double
            | RGB Double Double Double
            | RGBA Double Double Double Double

toRGBA :: Color -> (Double, Double, Double, Double)
toRGBA (Red)        = (1,0,0,defaultColorAlpha)
toRGBA (RedA a)     = (1,0,0,a)
toRGBA (Green)      = (0,1,0,defaultColorAlpha)
toRGBA (GreenA a)   = (0,1,0,a)
toRGBA (Blue)       = (0,0,1,defaultColorAlpha)
toRGBA (BlueA a)    = (0,0,1,a)
toRGBA (Yellow)     = (1,1,0,defaultColorAlpha)
toRGBA (YellowA a)  = (1,1,0,a)
toRGBA (White)      = (1,1,1,defaultColorAlpha)
toRGBA (WhiteA a)   = (1,1,1,a)
toRGBA (Black)      = (0,0,0,defaultColorAlpha)
toRGBA (BlackA a)   = (0,0,0,a)
toRGBA (Orange)     = (1,0.5,0,defaultColorAlpha)
toRGBA (OrangeA a)  = (1,0.5,0,a)
toRGBA (RGB r g b)  = (r,g,b,defaultColorAlpha)
toRGBA (RGBA r g b a) = (r,g,b,a)