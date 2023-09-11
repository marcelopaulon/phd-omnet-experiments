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

network DADCA_ACK_n100_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=128.75,191.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=212.90,150.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=196.75,86.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=191.52,23.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=120.15,25.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=55.45,35.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=2.77,107.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=44.13,157.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=158.86,288.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=313.72,12.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=392.56,2.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=473.13,86.17");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=353.75,113.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=289.66,90.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=85.07,243.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=73.02,332.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=4.29,330.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=2.80,439.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=223.22,319.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=297.71,379.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=245.59,460.35");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=218.74,540.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=144.27,586.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=74.06,529.77");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=114.70,472.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=69.67,409.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=149.57,422.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=105.54,632.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=168.99,651.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=231.28,615.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=39.26,603.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=10.83,691.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=60.99,735.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=165.09,509.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=13.94,500.02");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=105.03,779.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=67.78,827.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=74.64,893.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=8.41,846.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=135.96,711.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=232.12,396.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=342.35,587.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=295.92,654.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=290.78,551.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=342.68,309.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=379.51,401.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=387.00,475.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=427.80,545.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=460.18,610.89");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=147.62,359.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=155.89,939.10");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=183.49,1005.79");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=236.83,950.96");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=241.11,1032.75");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=222.25,1124.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=298.31,1060.51");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=157.69,1162.61");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=68.79,1139.14");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=86.17,1059.73");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=144.32,1081.29");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=195.00,1238.89");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=129.16,1242.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=64.85,1289.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=0.78,1316.04");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1.36,1218.87");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=154.62,1322.04");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=325.04,963.97");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=272.08,891.58");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=332.04,860.99");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=379.73,809.62");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=373.63,729.23");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=434.11,670.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=520.06,635.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=492.11,555.64");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=523.36,464.47");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=606.13,691.70");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=629.94,768.62");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=545.53,784.87");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=532.00,880.41");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=551.25,537.01");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=639.92,403.85");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=677.56,462.49");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=602.93,483.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=701.71,556.11");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=669.45,648.13");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=631.99,597.21");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=726.02,611.67");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=634.11,844.63");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=719.91,802.86");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=784.14,780.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=723.70,706.04");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=773.30,658.57");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=851.38,599.66");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=945.67,574.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1016.35,650.92");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1077.20,668.40");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1168.17,689.40");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1118.97,740.94");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1029.15,739.23");
    }

}

