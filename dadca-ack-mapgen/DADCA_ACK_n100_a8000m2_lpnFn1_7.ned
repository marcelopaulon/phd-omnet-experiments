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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=180.58,65.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=267.52,111.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=318.74,159.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=374.58,205.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=288.87,226.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=239.56,294.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=213.72,359.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=153.30,322.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=142.54,399.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=149.70,482.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=63.45,524.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=135.10,588.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=202.11,653.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=143.79,713.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=68.31,663.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=9.05,565.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=29.74,762.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=195.04,778.35");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=147.65,822.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=106.42,895.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=204.35,881.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=242.37,935.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=185.14,982.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=246.97,1026.88");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=327.46,1033.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=390.48,1079.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=311.31,1134.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=408.90,1142.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=412.27,1215.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=435.61,985.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=429.55,895.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=494.41,871.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=582.62,881.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=551.01,992.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=594.37,1061.96");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=538.28,1097.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=650.00,1008.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=645.30,943.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=555.40,819.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=465.91,801.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=586.80,721.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=632.95,653.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=541.15,677.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=492.47,605.70");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=456.52,543.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=398.08,600.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=431.21,668.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=345.16,734.49");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=247.08,719.14");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=272.21,594.36");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=329.29,543.92");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=401.88,485.34");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=435.33,393.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=365.16,330.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=435.39,277.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=528.38,309.47");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=453.44,193.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=523.43,91.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=541.13,154.17");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=635.30,180.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=706.26,229.02");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=784.33,274.65");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=836.66,320.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=791.83,360.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=873.57,378.43");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=963.09,390.00");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1060.95,387.37");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1087.72,469.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1171.08,445.05");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1145.45,551.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1133.27,394.74");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1215.96,402.71");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1069.93,661.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1074.44,597.44");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1039.07,506.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1008.71,564.42");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1007.65,632.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=992.75,722.78");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=960.45,784.25");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=995.40,858.45");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=939.89,881.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=876.50,900.93");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=792.62,901.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=803.04,802.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=879.24,768.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=744.36,831.66");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=758.35,688.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=746.31,759.94");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=685.86,708.17");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=733.49,628.16");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=729.69,528.55");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=692.23,473.68");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=651.18,398.75");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=591.05,451.38");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=501.37,418.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=464.55,470.40");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=533.54,524.63");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=200.08,575.80");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=257.95,515.91");
    }

}

