(G-CODE GENERATED BY FLATCAM v8.993 - www.flatcam.org - Version Date: 2020/06/05)

(Name: 133u02.drlbottom_cnc)
(Type: G-code from Excellon)
(Units: MM)

(Created on Friday, 13 November 2020 at 16:22)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 0.5)
(Tool: 2 -> Dia: 0.6)
(Tool: 3 -> Dia: 0.8)
(Tool: 4 -> Dia: 0.9)
(Tool: 5 -> Dia: 3.0)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 70.0)
(Tool: 2 -> Feedrate: 70.0)
(Tool: 3 -> Feedrate: 70.0)
(Tool: 4 -> Feedrate: 70.0)
(Tool: 5 -> Feedrate: 70.0)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)
(Tool: 2 -> Feedrate Rapids: 1500)
(Tool: 3 -> Feedrate Rapids: 1500)
(Tool: 4 -> Feedrate Rapids: 1500)
(Tool: 5 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -1.8)
(Tool: 2 -> Z_Cut: -1.8)
(Tool: 3 -> Z_Cut: -1.8)
(Tool: 4 -> Z_Cut: -1.8)
(Tool: 5 -> Z_Cut: -1.8)

(Tools Offset: )
(Tool: 5 -> Offset Z: -0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 2)
(Tool: 2 -> Z_Move: 2)
(Tool: 3 -> Z_Move: 2)
(Tool: 4 -> Z_Move: 2)
(Tool: 5 -> Z_Move: 2)

(Z Toolchange: 5.0 mm)
(X,Y Toolchange: 0.00, 0.00 mm)
(Z Start: None mm)
(Z End: 2.0 mm)
(Steps per circle: 128)
(Preprocessor Excellon: default)

(X range:  -89.3600 ...   -3.1000  mm)
(Y range:  -30.8200 ...   -2.2800  mm)

(Spindle Speed: 10000.0 RPM)
G21
G90
G94



G01 F70.00

M5
G00 Z5.0000
T5
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 3.00 ||| Total drills for tool T5 = 2)
M0
G00 Z5.0000

M03 S10000
G00 X-4.6000 Y-4.9600
G01 Z-1.8000
G01 Z0
G00 Z2.0000
G00 X-82.7100 Y-4.9600
G01 Z-1.8000
G01 Z0
G00 Z2.0000
M05
G00 Z2.00
