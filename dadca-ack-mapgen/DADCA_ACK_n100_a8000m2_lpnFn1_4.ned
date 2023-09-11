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
        @display("i=misc/sensor,p=62.20,44.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=32.80,114.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=185.41,18.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=269.75,8.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=276.99,81.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=209.96,142.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=341.45,82.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=397.19,120.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=446.02,177.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=481.81,121.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=334.73,150.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=282.36,226.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=310.68,312.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=239.84,368.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=169.70,434.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=306.95,398.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=220.52,289.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=136.67,521.35");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=70.26,558.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=218.43,604.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=125.76,583.79");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=140.83,701.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=90.63,738.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=71.16,663.95");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=72.81,803.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=95.12,870.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=103.46,961.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=124.47,1043.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=64.06,1097.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=198.84,1064.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=226.45,1142.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=159.34,1184.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=244.83,1219.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=288.45,1170.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=318.73,1232.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=356.90,1300.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=297.91,1334.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=310.83,1395.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=276.49,1476.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=358.12,1499.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=389.09,1589.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=404.67,1659.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=344.35,1674.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=403.86,1735.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=456.73,1694.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=509.68,1767.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=524.74,1845.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=600.45,1882.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=641.04,1964.71");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=710.72,1890.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=652.53,1810.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=635.76,1713.12");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=595.96,1656.37");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=717.73,1676.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=724.12,1737.42");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=749.87,1832.72");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=793.75,1726.94");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=838.45,1667.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=906.95,1595.77");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=836.57,1607.26");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=860.21,1536.13");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=895.33,1467.89");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=854.54,1413.47");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=860.66,1329.17");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=796.89,1321.64");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=734.05,1308.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=665.56,1250.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=690.16,1410.63");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=619.91,1387.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=546.68,1372.48");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=467.90,1388.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=204.62,1518.06");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=246.75,1589.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=159.24,1628.78");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=114.07,1669.62");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=131.04,1747.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=189.64,1685.10");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=284.25,1671.25");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=144.76,1505.69");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=61.39,1521.75");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=97.56,1585.04");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=15.00,1570.75");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=23.38,1687.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=30.16,1766.53");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=186.28,1787.67");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=90.70,1447.67");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=391.71,1410.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=429.00,1311.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=464.66,1258.98");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=496.39,1196.89");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=584.46,1147.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=556.49,1053.87");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=616.79,1006.04");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=686.89,1066.93");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=689.39,1150.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=597.71,1207.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=815.05,1005.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=823.76,1075.46");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=755.11,1124.73");
    }

}

