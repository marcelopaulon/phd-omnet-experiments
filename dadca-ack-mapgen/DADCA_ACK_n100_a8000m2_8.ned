package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_8
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=312.09,7959.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=408.52,7972.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=649.84,7972.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=873.52,7975.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=97.41,7925.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=214.34,7925.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=359.49,7919.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=498.42,7953.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=585.43,7955.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=743.21,7934.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=803.91,7953.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=983.48,7934.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=291.42,7875.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=471.23,7883.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=547.11,7871.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=627.75,7884.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=688.59,7892.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=778.22,7881.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=845.14,7870.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=926.62,7893.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=47.58,7855.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=239.93,7831.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=392.34,7838.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=20.61,7791.73");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=136.54,7812.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=324.02,7791.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=605.25,7814.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=773.61,7815.07");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=926.08,7823.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=262.19,7774.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=489.34,7766.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=567.08,7750.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=698.29,7757.45");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=879.22,7781.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=951.36,7766.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=155.94,7730.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=337.49,7713.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=437.49,7724.76");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=625.74,7725.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=775.04,7713.83");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=882.81,7719.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=987.75,7714.51");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=78.73,7697.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=161.16,7656.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=270.20,7694.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=387.59,7676.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=800.41,7657.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=877.76,7656.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=20.60,7639.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=267.55,7615.02");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=484.39,7649.45");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=581.93,7616.42");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=680.45,7637.25");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=925.87,7615.66");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=987.69,7639.60");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=97.28,7593.10");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=328.50,7592.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=392.53,7584.57");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=739.66,7580.62");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=801.16,7575.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=22.58,7553.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=210.12,7552.64");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=349.11,7534.66");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=449.13,7535.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=513.44,7557.25");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=652.89,7532.99");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=850.16,7534.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=105.71,7524.75");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=264.07,7493.74");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=501.84,7486.68");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=695.87,7485.01");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=974.38,7504.99");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=20.44,7469.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=205.86,7453.47");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=365.64,7451.33");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=422.55,7478.77");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=614.74,7447.20");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=770.40,7480.57");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=826.58,7445.00");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=80.80,7438.92");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=512.13,7421.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=720.13,7402.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=785.09,7397.48");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=887.52,7435.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=72.71,7365.78");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=166.15,7391.20");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=255.24,7380.39");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=351.70,7382.15");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=434.85,7385.46");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=591.46,7389.57");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=665.86,7362.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=856.63,7365.64");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=932.10,7363.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=309.02,7328.70");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=414.03,7326.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=490.93,7332.33");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=749.92,7334.94");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=988.83,7336.39");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=24.19,7273.61");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=139.00,7271.60");
    }

}

