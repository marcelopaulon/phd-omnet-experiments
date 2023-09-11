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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=64.64,184.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=145.08,161.45");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=93.45,10.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=28.50,63.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=124.79,218.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=100.60,279.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=199.26,315.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=205.36,391.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=226.05,466.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=143.57,465.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=290.22,467.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=338.57,386.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=419.48,348.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=498.43,397.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=521.86,452.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=592.10,429.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=523.43,570.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=474.25,535.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=624.46,561.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=594.56,644.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=542.27,681.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=583.74,738.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=498.72,760.68");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=440.05,711.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=473.59,658.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=648.30,758.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=528.80,853.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=403.38,765.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=670.08,658.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=735.99,730.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=756.09,825.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=738.73,909.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=662.97,934.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=625.93,869.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=615.39,1001.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=687.26,865.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=750.63,1015.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=792.81,972.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=580.66,909.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=506.17,937.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=836.79,1023.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=853.34,947.01");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=923.19,969.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=718.23,1075.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=950.61,1088.86");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1017.27,1020.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=974.54,1176.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=836.16,1144.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=794.20,1232.57");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=829.07,1282.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=703.92,1178.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=633.51,1142.09");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=588.18,1194.81");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=586.76,1272.45");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=503.18,1291.16");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=445.11,1255.96");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=519.51,1198.23");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=591.50,1085.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=656.07,1065.68");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=889.43,1231.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=940.43,1305.48");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=962.60,1235.95");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1072.96,997.13");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1116.10,1050.88");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1120.45,898.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1162.52,975.86");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1212.41,1090.53");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1227.78,1012.84");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1226.80,924.99");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1302.37,1016.04");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1324.89,934.28");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1364.70,989.59");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1414.50,946.61");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1429.12,1022.78");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1347.70,1058.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1506.53,1069.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1481.45,979.56");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1615.89,1019.85");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1653.85,1068.92");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1555.37,997.83");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1733.61,1155.24");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1689.43,996.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1761.30,984.68");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1729.29,927.10");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1805.54,906.16");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1810.68,846.03");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1871.25,834.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1920.70,895.00");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1852.03,952.13");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1907.08,1004.25");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1988.26,1042.26");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=2080.55,1066.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=2010.49,1116.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=2090.89,1136.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=2158.92,1140.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=2013.54,1182.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1929.33,1246.70");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1979.32,1280.67");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1927.19,1110.21");
    }

}

