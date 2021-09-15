## The required data by the maneuver MPC to work.

# State 
float64 x
float64 y
float64 psi
float64 u
float64 v
float64 r

# Command
float64 xd
float64 yd
float64 psid
float64 ud
float64 vd
float64 rd

# Feed-Forward acceleration
float64 axd 
float64 ayd
float64 apsid

# Error state
float64 xe
float64 ye
float64 psie
float64 ue
float64 ve
float64 re

