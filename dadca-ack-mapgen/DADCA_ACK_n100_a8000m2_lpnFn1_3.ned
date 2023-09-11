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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=152.02,34.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=213.78,80.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=37.29,10.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=146.64,166.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=65.44,186.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=7.16,81.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=5.90,153.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=229.59,169.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=73.34,254.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=155.13,267.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=243.30,239.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=178.13,323.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=100.54,361.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=260.97,303.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=187.08,393.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=296.28,422.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=268.01,476.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=340.94,491.29");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=361.65,432.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=354.38,573.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=352.49,633.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=332.08,696.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=413.60,742.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=225.05,692.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=253.88,631.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=423.03,613.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=230.19,802.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=161.49,800.43");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=87.87,816.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=22.38,768.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=74.49,692.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=46.38,915.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=178.59,912.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=369.14,822.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=311.37,773.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=240.66,895.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=251.13,964.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=345.34,937.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=353.99,1022.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=300.07,1069.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=371.38,1159.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=424.49,1231.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=461.91,1141.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=495.71,1239.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=427.50,1298.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=367.32,1288.44");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=452.48,1364.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=335.92,1353.41");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=262.40,1305.51");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=189.26,1238.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=162.73,1172.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=239.54,1123.17");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=286.06,1246.90");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=194.50,1384.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=144.92,1330.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=293.16,1172.24");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=71.15,1252.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=36.34,1322.60");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=8.24,1240.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=153.07,1020.69");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=68.13,1016.93");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=18.47,860.53");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=135.20,866.48");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=224.60,1056.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=103.69,1088.83");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=83.54,1169.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=34.23,1094.22");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=225.30,1189.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=243.90,1450.82");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=176.83,1514.78");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=185.90,1607.60");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=256.29,1562.20");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=299.67,1634.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=338.09,1722.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=426.13,1739.94");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=240.78,1722.03");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=142.80,1722.49");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=155.39,1662.39");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=69.14,1699.90");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=68.68,1633.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=377.28,1542.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=374.17,1460.31");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=302.39,1410.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=395.41,1399.19");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=157.76,1439.43");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=85.97,1519.62");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=44.53,1424.83");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=84.34,1364.33");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=428.42,1066.60");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=463.34,1005.32");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=530.70,986.76");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=468.55,932.69");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=423.06,849.20");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=511.68,855.57");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=576.88,907.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=662.86,946.90");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=633.03,881.53");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=668.10,809.49");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=552.54,808.28");
    }

}

