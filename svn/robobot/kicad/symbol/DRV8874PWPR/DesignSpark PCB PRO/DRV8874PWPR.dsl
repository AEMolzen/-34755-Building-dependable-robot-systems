SamacSys ECAD Model
2320806/455769/2.50/17/3/Integrated Circuit

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r147.5_45"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.45) (shapeHeight 1.475))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "r355_246"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 2.46) (shapeHeight 3.55))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "SOP65P640X120-17N" (originalName "SOP65P640X120-17N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r147.5_45) (pt -2.938, 2.275) (rotation 90))
			(pad (padNum 2) (padStyleRef r147.5_45) (pt -2.938, 1.625) (rotation 90))
			(pad (padNum 3) (padStyleRef r147.5_45) (pt -2.938, 0.975) (rotation 90))
			(pad (padNum 4) (padStyleRef r147.5_45) (pt -2.938, 0.325) (rotation 90))
			(pad (padNum 5) (padStyleRef r147.5_45) (pt -2.938, -0.325) (rotation 90))
			(pad (padNum 6) (padStyleRef r147.5_45) (pt -2.938, -0.975) (rotation 90))
			(pad (padNum 7) (padStyleRef r147.5_45) (pt -2.938, -1.625) (rotation 90))
			(pad (padNum 8) (padStyleRef r147.5_45) (pt -2.938, -2.275) (rotation 90))
			(pad (padNum 9) (padStyleRef r147.5_45) (pt 2.938, -2.275) (rotation 90))
			(pad (padNum 10) (padStyleRef r147.5_45) (pt 2.938, -1.625) (rotation 90))
			(pad (padNum 11) (padStyleRef r147.5_45) (pt 2.938, -0.975) (rotation 90))
			(pad (padNum 12) (padStyleRef r147.5_45) (pt 2.938, -0.325) (rotation 90))
			(pad (padNum 13) (padStyleRef r147.5_45) (pt 2.938, 0.325) (rotation 90))
			(pad (padNum 14) (padStyleRef r147.5_45) (pt 2.938, 0.975) (rotation 90))
			(pad (padNum 15) (padStyleRef r147.5_45) (pt 2.938, 1.625) (rotation 90))
			(pad (padNum 16) (padStyleRef r147.5_45) (pt 2.938, 2.275) (rotation 90))
			(pad (padNum 17) (padStyleRef r355_246) (pt 0, 0) (rotation 0))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.925 2.8) (pt 3.925 2.8) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 3.925 2.8) (pt 3.925 -2.8) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 3.925 -2.8) (pt -3.925 -2.8) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.925 -2.8) (pt -3.925 2.8) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.2 2.5) (pt 2.2 2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.2 2.5) (pt 2.2 -2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.2 -2.5) (pt -2.2 -2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.2 -2.5) (pt -2.2 2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.2 1.85) (pt -1.55 2.5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.675 2.85) (pt -2.2 2.85) (width 0.2))
		)
	)
	(symbolDef "DRV8874PWPR" (originalName "DRV8874PWPR")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 7) (pt 0 mils -600 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -625 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 8) (pt 0 mils -700 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -725 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 9) (pt 700 mils -1200 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 725 mils -970 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 10) (pt 1400 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 11) (pt 1400 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 12) (pt 1400 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 13) (pt 1400 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 14) (pt 1400 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 15) (pt 1400 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 16) (pt 1400 mils -600 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -625 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 17) (pt 1400 mils -700 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -725 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 100 mils) (pt 1200 mils 100 mils) (width 6 mils))
		(line (pt 1200 mils 100 mils) (pt 1200 mils -1000 mils) (width 6 mils))
		(line (pt 1200 mils -1000 mils) (pt 200 mils -1000 mils) (width 6 mils))
		(line (pt 200 mils -1000 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1250 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "DRV8874PWPR" (originalName "DRV8874PWPR") (compHeader (numPins 17) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "EN/IN1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "PH/IN2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "NSLEEP") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "NFAULT") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "5" (pinName "VREF") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "6" (pinName "IPROPI") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "7" (pinName "IMODE") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "8" (pinName "OUT1") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "17" (pinName "EP") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "16" (pinName "PMODE") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "15" (pinName "GND") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "14" (pinName "CPL") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "13" (pinName "CPH") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "12" (pinName "VCP") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "11" (pinName "VM") (partNum 1) (symPinNum 15) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "10" (pinName "OUT2") (partNum 1) (symPinNum 16) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "9" (pinName "PGND") (partNum 1) (symPinNum 17) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "DRV8874PWPR"))
		(attachedPattern (patternNum 1) (patternName "SOP65P640X120-17N")
			(numPads 17)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
				(padNum 13) (compPinRef "13")
				(padNum 14) (compPinRef "14")
				(padNum 15) (compPinRef "15")
				(padNum 16) (compPinRef "16")
				(padNum 17) (compPinRef "17")
			)
		)
		(attr "Mouser Part Number" "595-DRV8874PWPR")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/DRV8874PWPR?qs=mAH9sUMRCtvh%2FSU15ywmDA%3D%3D")
		(attr "Manufacturer_Name" "Texas Instruments")
		(attr "Manufacturer_Part_Number" "DRV8874PWPR")
		(attr "Description" "H-Bridge Motor Driver With Integrated Current Sense and Regulation")
		(attr "Datasheet Link" "https://www.ti.com/lit/gpn/DRV8874")
		(attr "Height" "1.2 mm")
	)

)
