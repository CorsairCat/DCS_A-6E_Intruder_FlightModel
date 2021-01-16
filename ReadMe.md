# A-6E Intruder Mod's EFM for DCS World
> + This is the source code for a-6e mod
> + This is created from ed template

## TO-DO List
> The Query is from 1 very ergent to max later
1. Fix drag profile, current is bit too large;
2. Fix the pitch control is not stable and will shake;
3. Add the SpeedBrake, Flap and Gear Drag and Lift;
4. Add the Engine Temperature output;
5. Check the performance for turning;
6. Add carrier operations;
7. Overide fuel control system currently use
8. Complete Autopilot Control in EFM
9. Adding Bombing/Calculation System to EFM

## Further Development
1. transfer engine operation fully into EFM
2. Test for a more percious Flight Model
3. Move some other cockpit class into EFM

## Flight Model Structure
### NameSpace A6E
1. Interface Class
2. Engine Class
3. Mass Class
4. Gear Class
5. Motion Class
6. Atmosphere Class
7. Aerodynamic Class
8. CarrierOp Class
9. Structure Class
10. Mission System Class

### NameSpace J52Engine
1. mainly some constant data