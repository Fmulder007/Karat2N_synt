(G-CODE GENERATED BY FLATCAM v8.993 - www.flatcam.org - Version Date: 2020/06/05)

(Name: Syntez2.drl_cnc)
(Type: G-code from Excellon)
(Units: MM)

(Created on Thursday, 08 October 2020 at 13:59)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 0.5)
(Tool: 2 -> Dia: 0.8)
(Tool: 3 -> Dia: 3.0)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 70.0)
(Tool: 2 -> Feedrate: 70.0)
(Tool: 3 -> Feedrate: 70.0)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)
(Tool: 2 -> Feedrate Rapids: 1500)
(Tool: 3 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -1.9)
(Tool: 2 -> Z_Cut: -1.9)
(Tool: 3 -> Z_Cut: -1.9)

(Tools Offset: )
(Tool: 1 -> Offset Z: -0.0)
(Tool: 2 -> Offset Z: -0.0)
(Tool: 3 -> Offset Z: -0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 2)
(Tool: 2 -> Z_Move: 2)
(Tool: 3 -> Z_Move: 2)

(Z Toolchange: 5.0 mm)
(X,Y Toolchange: 0.00, 0.00 mm)
(Z Start: None mm)
(Z End: 2.0 mm)
(Steps per circle: 128)
(Preprocessor Excellon: default)

(X range:    3.1000 ...   96.6500  mm)
(Y range:  -32.4000 ...   -2.2800  mm)

(Spindle Speed: None RPM)
G21
G90
G94



G01 F70.00

M5
G00 Z5.0000
T1
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 0.50 ||| Total drills for tool T1 = 21)
M0
G00 Z5.0000

M03
G00 X30.3200 Y-32.0300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X43.5400 Y-28.4200
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X43.5400 Y-25.8800
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X45.7200 Y-15.0000
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X47.8600 Y-15.0000
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X53.8200 Y-20.0400
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X53.8200 Y-25.1200
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X63.1800 Y-32.1500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X96.4000 Y-9.0500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X63.3200 Y-17.4800
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X60.7800 Y-17.4800
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X57.0700 Y-14.8000
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X63.9000 Y-8.3300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X58.2600 Y-5.0700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X58.2600 Y-2.5300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X47.4700 Y-10.1200
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X41.2700 Y-3.4500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X28.4200 Y-8.8100
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X38.5800 Y-20.0400
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X38.5800 Y-22.5800
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X38.5800 Y-25.1200
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G01 F70.00

M5
G00 Z5.0000
T2
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 0.80 ||| Total drills for tool T2 = 20)
M0
G00 Z5.0000

M03
G00 X13.7700 Y-11.4700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X16.3100 Y-11.4700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X18.8500 Y-11.4700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X23.6600 Y-8.2900
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X28.0000 Y-6.0700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X30.5400 Y-6.0700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X33.0800 Y-6.0700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X72.4000 Y-2.8000
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X75.5300 Y-3.3300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X75.5300 Y-5.8700
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X75.5300 Y-8.4100
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X75.5300 Y-10.9500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X72.4000 Y-10.4200
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X65.4900 Y-11.6500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X62.9500 Y-11.6500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X33.0800 Y-3.5300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X30.5400 Y-3.5300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X28.0000 Y-3.5300
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X23.6600 Y-3.2100
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X23.6600 Y-5.7500
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G01 F70.00

M5
G00 Z5.0000
T3
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 3.00 ||| Total drills for tool T3 = 2)
M0
G00 Z5.0000

M03
G00 X4.6000 Y-4.9600
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X82.7100 Y-4.9600
G01 Z-1.9000
G01 Z0
G00 Z2.0000
M05
G00 Z2.00

