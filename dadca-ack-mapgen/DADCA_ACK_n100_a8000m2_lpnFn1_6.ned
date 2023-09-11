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

network DADCA_ACK_n100_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=31.65,122.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=96.45,176.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=13.09,57.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=111.28,21.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=158.48,202.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=206.75,165.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=286.03,159.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=201.56,98.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=24.16,261.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=70.74,311.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=115.46,384.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1.04,414.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=151.99,324.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=134.82,447.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=85.28,498.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=203.20,511.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=116.20,553.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=157.15,618.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=99.75,700.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=55.44,658.38");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=67.79,595.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=147.38,779.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=224.47,831.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=263.97,769.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=106.91,875.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=68.67,928.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=14.18,967.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=10.22,894.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=47.03,837.31");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=221.46,918.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=278.19,952.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=280.30,863.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=349.44,840.84");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=294.38,1034.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=194.94,1022.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=126.74,987.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=74.67,1029.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=229.16,1087.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=310.78,1104.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=396.94,1093.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=359.23,1156.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=501.16,1138.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=580.53,1081.07");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=527.26,1028.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=484.52,950.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=463.64,1073.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=535.79,1241.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=418.29,1304.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=362.84,1269.64");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=404.10,1223.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=446.04,1176.03");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=474.67,1247.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=621.83,1129.99");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=641.46,1046.49");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=609.79,1199.65");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=679.11,1187.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=652.81,1271.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=734.85,1131.97");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=707.37,1303.37");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=752.23,1368.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=770.30,1285.80");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=823.01,1326.50");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=871.31,1242.77");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=911.75,1356.93");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=984.33,1353.91");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=939.18,1295.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=957.89,1203.49");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1021.12,1239.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1049.34,1156.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1008.26,1092.16");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=980.44,971.07");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1070.19,1007.10");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1108.58,935.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1116.68,845.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1091.77,1088.66");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1136.54,1158.59");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1139.14,1254.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1200.46,1289.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1214.40,1125.60");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1264.70,1187.45");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1339.11,1195.05");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1380.96,1240.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1307.51,1261.93");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1460.88,1232.16");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1484.84,1320.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1568.19,1352.60");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1639.34,1343.66");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1652.77,1406.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1713.31,1271.11");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1742.05,1179.11");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1804.87,1143.04");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1772.54,1059.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1714.08,1045.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1772.94,968.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1831.94,1007.87");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1749.20,860.70");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1656.51,841.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1718.70,805.36");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1654.89,935.46");
    }

}

