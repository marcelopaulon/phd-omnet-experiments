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
        @display("i=misc/sensor,p=79.13,171.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=153.70,202.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=175.77,126.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=54.67,51.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=14.92,212.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=9.47,292.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=24.27,365.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=121.39,414.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=93.55,284.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=214.85,350.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=188.29,284.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=153.66,351.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=158.26,517.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=99.08,538.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=16.81,485.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=77.49,481.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=214.97,481.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=252.83,422.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=162.12,604.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=128.51,662.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=100.82,734.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=155.32,779.60");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=233.98,732.81");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=286.04,682.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=217.43,654.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=280.56,606.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=268.17,532.30");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=218.92,577.84");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=201.89,845.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=87.00,831.14");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=61.93,908.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=64.73,1001.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=156.70,994.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=161.85,1060.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=90.19,1084.78");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=24.29,1070.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=177.82,1166.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=34.59,1174.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=5.54,1265.74");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=81.48,1222.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=49.71,1343.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=31.38,1430.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=208.49,1111.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=136.72,934.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=264.08,1006.09");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=299.56,935.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=250.22,891.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=311.71,873.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=372.13,874.86");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1.57,967.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=134.60,1251.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=163.75,1362.00");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=109.22,1392.05");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=106.91,1462.19");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=55.57,1494.94");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=69.30,1577.08");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=134.30,1599.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=84.60,1653.53");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=55.24,1720.41");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=14.93,1674.93");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=256.51,1615.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=353.68,1604.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=339.94,1539.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=423.63,1665.49");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=406.15,1734.56");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=480.28,1731.76");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=561.57,1727.35");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=525.13,1791.41");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=613.58,1822.87");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=549.60,1650.90");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=638.27,1639.91");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=584.14,1594.88");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=499.05,1615.21");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=514.18,1539.94");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=453.32,1569.67");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=396.16,1512.80");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=346.49,1461.51");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=346.09,1389.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=277.03,1442.83");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=253.05,1376.70");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=259.29,1313.95");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=311.72,1194.47");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=366.51,1148.70");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=222.18,1215.69");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=95.97,1158.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=176.40,1295.56");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=162.19,1509.29");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=12.23,1598.58");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=158.13,1660.69");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=198.33,1738.44");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=248.04,1695.35");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=276.08,1774.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=332.62,1820.93");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=400.74,1839.94");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=381.47,1930.56");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=445.21,1979.63");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=354.32,2013.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=309.84,2076.73");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=220.74,2046.91");
    }

}

