(G-CODE GENERATED BY FLATCAM v8.993 - www.flatcam.org - Version Date: 2020/06/05)

(Name: driver_�1_����.gbr_cutout_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Saturday, 31 October 2020 at 23:26)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)

(TOOL DIAMETER: 0.8 mm)
(Feedrate_XY: 120.0 mm/min)
(Feedrate_Z: 70.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -1.7 mm)
(DepthPerCut: 0.1 mm <=>17 passes)
(Z_Move: 2.0 mm)
(Z Start: None mm)
(Z End: 25.0 mm)
(Steps per circle: 128)
(Preprocessor Geometry: default)

(X range:   -0.5000 ...   15.5000  mm)
(Y range:  -10.5125 ...    0.4875  mm)

(Spindle Speed: 10000 RPM)
G21
G90
G94



G01 F120.00
G00 Z2.0000

M03 S10000
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.1000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.2000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.3000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.4000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.5000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.6000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.7000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.8000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-0.9000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.0000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.1000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.2000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.3000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.4000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.5000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.6000
G01 F120.00
G01 X15.0490 Y-10.5101
G01 X15.0975 Y-10.5029
G01 X15.1451 Y-10.4910
G01 X15.1913 Y-10.4744
G01 X15.2357 Y-10.4535
G01 X15.2778 Y-10.4282
G01 X15.3172 Y-10.3990
G01 X15.3536 Y-10.3661
G01 X15.3865 Y-10.3297
G01 X15.4157 Y-10.2903
G01 X15.4410 Y-10.2482
G01 X15.4619 Y-10.2038
G01 X15.4785 Y-10.1576
G01 X15.4904 Y-10.1100
G01 X15.4976 Y-10.0615
G01 X15.5000 Y-10.0125
G01 X15.5000 Y-0.0125
G01 X15.4976 Y0.0365
G01 X15.4904 Y0.0850
G01 X15.4785 Y0.1326
G01 X15.4619 Y0.1788
G01 X15.4410 Y0.2232
G01 X15.4157 Y0.2653
G01 X15.3865 Y0.3047
G01 X15.3536 Y0.3411
G01 X15.3172 Y0.3740
G01 X15.2778 Y0.4032
G01 X15.2357 Y0.4285
G01 X15.1913 Y0.4494
G01 X15.1451 Y0.4660
G01 X15.0975 Y0.4779
G01 X15.0490 Y0.4851
G01 X15.0000 Y0.4875
G01 X0.0000 Y0.4875
G01 X-0.0490 Y0.4851
G01 X-0.0975 Y0.4779
G01 X-0.1451 Y0.4660
G01 X-0.1913 Y0.4494
G01 X-0.2357 Y0.4285
G01 X-0.2778 Y0.4032
G01 X-0.3172 Y0.3740
G01 X-0.3536 Y0.3411
G01 X-0.3865 Y0.3047
G01 X-0.4157 Y0.2653
G01 X-0.4410 Y0.2232
G01 X-0.4619 Y0.1788
G01 X-0.4785 Y0.1326
G01 X-0.4904 Y0.0850
G01 X-0.4976 Y0.0365
G01 X-0.5000 Y-0.0125
G01 X-0.5000 Y-10.0125
G01 X-0.4976 Y-10.0615
G01 X-0.4904 Y-10.1100
G01 X-0.4785 Y-10.1576
G01 X-0.4619 Y-10.2038
G01 X-0.4410 Y-10.2482
G01 X-0.4157 Y-10.2903
G01 X-0.3865 Y-10.3297
G01 X-0.3536 Y-10.3661
G01 X-0.3172 Y-10.3990
G01 X-0.2778 Y-10.4282
G01 X-0.2357 Y-10.4535
G01 X-0.1913 Y-10.4744
G01 X-0.1451 Y-10.4910
G01 X-0.0975 Y-10.5029
G01 X-0.0490 Y-10.5101
G01 X0.0000 Y-10.5125
G01 X15.0000 Y-10.5125
G00 X15.0000 Y-10.5125
G01 F70.00
G01 Z-1.7000
G01 F120.00
G01 X0.0000 Y-10.5125
G01 X-0.0490 Y-10.5101
G01 X-0.0975 Y-10.5029
G01 X-0.1451 Y-10.4910
G01 X-0.1913 Y-10.4744
G01 X-0.2357 Y-10.4535
G01 X-0.2778 Y-10.4282
G01 X-0.3172 Y-10.3990
G01 X-0.3536 Y-10.3661
G01 X-0.3865 Y-10.3297
G01 X-0.4157 Y-10.2903
G01 X-0.4410 Y-10.2482
G01 X-0.4619 Y-10.2038
G01 X-0.4785 Y-10.1576
G01 X-0.4904 Y-10.1100
G01 X-0.4976 Y-10.0615
G01 X-0.5000 Y-10.0125
G01 X-0.5000 Y-0.0125
G01 X-0.4976 Y0.0365
G01 X-0.4904 Y0.0850
G01 X-0.4785 Y0.1326
G01 X-0.4619 Y0.1788
G01 X-0.4410 Y0.2232
G01 X-0.4157 Y0.2653
G01 X-0.3865 Y0.3047
G01 X-0.3536 Y0.3411
G01 X-0.3172 Y0.3740
G01 X-0.2778 Y0.4032
G01 X-0.2357 Y0.4285
G01 X-0.1913 Y0.4494
G01 X-0.1451 Y0.4660
G01 X-0.0975 Y0.4779
G01 X-0.0490 Y0.4851
G01 X0.0000 Y0.4875
G01 X15.0000 Y0.4875
G01 X15.0490 Y0.4851
G01 X15.0975 Y0.4779
G01 X15.1451 Y0.4660
G01 X15.1913 Y0.4494
G01 X15.2357 Y0.4285
G01 X15.2778 Y0.4032
G01 X15.3172 Y0.3740
G01 X15.3536 Y0.3411
G01 X15.3865 Y0.3047
G01 X15.4157 Y0.2653
G01 X15.4410 Y0.2232
G01 X15.4619 Y0.1788
G01 X15.4785 Y0.1326
G01 X15.4904 Y0.0850
G01 X15.4976 Y0.0365
G01 X15.5000 Y-0.0125
G01 X15.5000 Y-10.0125
G01 X15.4976 Y-10.0615
G01 X15.4904 Y-10.1100
G01 X15.4785 Y-10.1576
G01 X15.4619 Y-10.2038
G01 X15.4410 Y-10.2482
G01 X15.4157 Y-10.2903
G01 X15.3865 Y-10.3297
G01 X15.3536 Y-10.3661
G01 X15.3172 Y-10.3990
G01 X15.2778 Y-10.4282
G01 X15.2357 Y-10.4535
G01 X15.1913 Y-10.4744
G01 X15.1451 Y-10.4910
G01 X15.0975 Y-10.5029
G01 X15.0490 Y-10.5101
G01 X15.0000 Y-10.5125
G00 Z2.0000
M05
G00 Z2.0000
G00 Z25.00

