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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=61.86,44.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=178.96,13.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=219.03,90.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=289.12,45.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=361.17,23.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=278.76,114.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=11.57,84.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=97.33,189.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=27.67,168.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=39.79,271.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=216.56,182.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=294.69,192.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=324.26,260.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=397.62,91.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=390.48,188.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=469.19,245.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=512.57,316.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=599.53,340.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=641.64,292.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=364.89,341.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=349.65,422.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=304.41,510.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=348.79,565.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=390.12,637.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=451.34,655.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=563.78,729.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=508.40,786.48");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=510.55,691.28");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=525.95,616.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=413.70,782.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=411.21,878.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=405.49,974.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=424.79,1031.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=481.96,1082.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=548.14,1112.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=514.37,1188.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=491.25,1266.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=405.83,1268.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=437.71,1359.00");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=348.53,1342.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=285.16,1313.38");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=262.87,1390.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=179.76,1430.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=103.67,1460.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=166.38,1490.75");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=184.38,1551.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=111.25,1567.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=47.31,1605.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=18.02,1667.94");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=100.51,1714.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=177.45,1659.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=67.31,1517.17");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=54.83,1343.18");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=48.18,1420.23");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=115.16,1305.91");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=267.49,1243.28");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=183.33,1248.16");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=6.70,1491.10");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=189.18,1356.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=164.18,1185.45");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=151.90,1092.71");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=199.59,1012.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=129.43,988.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=169.29,902.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=112.50,859.38");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=29.33,851.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=115.57,767.59");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=166.47,824.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=63.76,1181.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=6.67,1150.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=22.87,1241.30");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=65.71,1038.95");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=49.07,980.23");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=210.63,1071.69");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=253.08,1151.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=112.91,1396.15");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=364.55,1443.10");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=306.93,1510.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=302.62,1194.42");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=399.41,1162.31");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=452.49,1219.98");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=501.44,1354.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=496.32,1429.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=552.75,1394.35");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=582.81,1459.45");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=519.46,1532.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=458.21,1544.60");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=343.95,1616.43");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=406.09,1574.98");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=279.70,1601.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=383.85,1716.61");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=300.86,1721.82");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=258.02,1679.80");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=190.32,1721.91");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=96.05,1643.89");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=51.05,1767.86");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=59.31,1841.63");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=59.35,1903.40");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=76.29,1974.93");
    }

}

