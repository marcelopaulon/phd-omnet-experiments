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

network DADCA_ACK_n100_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor;p=5.02,112.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=16.96,207.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=9.81,286.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=51.15,330.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=80.39,274.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=122.58,223.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=204.15,170.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=244.89,254.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=34.44,8.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=155.03,133.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=299.02,345.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=263.21,429.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=235.07,519.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=161.25,400.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=197.29,464.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=235.34,372.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=410.62,391.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=456.88,432.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=524.29,439.90");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=580.01,510.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=642.98,510.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=680.20,449.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=619.52,442.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=738.91,434.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=532.97,563.63");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=355.02,534.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=350.16,438.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=442.70,493.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=751.37,570.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=698.13,633.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=625.72,661.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=710.58,693.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=804.18,702.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=793.37,778.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=882.33,743.97");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=921.14,825.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=990.27,846.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1019.45,757.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=948.82,920.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1016.67,931.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1084.09,941.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1145.83,883.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1229.78,928.49");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1154.34,964.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1124.84,1028.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=985.78,990.47");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=901.97,958.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=908.68,1025.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=828.26,1026.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=759.11,997.98");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=679.12,1001.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=687.14,1082.95");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=725.67,941.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=847.37,873.80");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=885.01,624.50");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=961.29,566.25");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=992.94,653.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1205.45,1005.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1290.52,1049.21");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1346.55,1075.61");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1409.51,1085.82");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1424.31,1016.92");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1478.83,964.89");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1534.13,1038.84");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1350.10,1138.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1103.95,1088.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1054.28,1149.76");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1072.03,1216.14");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1011.99,1194.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=934.59,1235.88");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=867.08,1296.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=853.96,1220.93");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=792.29,1156.87");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=855.32,1090.65");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=605.82,1074.29");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=676.60,1162.66");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=712.14,1224.02");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=614.17,1227.67");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=558.91,1171.98");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=589.00,1342.03");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=543.39,1265.49");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=711.06,1322.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=500.82,1204.66");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=519.75,1375.83");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=416.38,1444.90");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=370.18,1511.74");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=379.68,1345.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=419.74,1283.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=373.91,1197.95");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=505.98,1312.91");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=572.37,1493.33");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=479.91,1456.97");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=647.16,1515.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=735.72,1518.69");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=703.73,1459.44");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=575.34,1405.62");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=338.83,1296.78");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=421.34,1632.22");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=437.70,1721.90");
    }

}

