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
        @display("i=misc/sensor;p=145.10,33.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=62.13,25.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=158.37,163.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=238.86,119.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=309.05,92.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=191.25,82.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=298.58,166.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=249.52,206.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=308.82,228.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=391.25,234.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=427.46,165.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=498.06,231.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=525.44,148.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=587.98,164.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=444.17,44.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=512.62,61.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=556.47,12.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=585.99,66.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=356.70,145.12");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=481.83,291.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=434.37,367.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=577.62,325.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=420.31,436.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=452.25,500.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=357.93,490.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=250.65,440.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=394.61,560.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=480.10,563.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=303.39,519.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=236.64,499.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=355.55,425.33");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=282.70,340.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=208.31,314.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=151.23,491.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=180.53,421.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=348.08,353.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=553.88,262.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=643.18,109.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=707.76,140.37");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=647.26,214.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=702.45,287.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=763.11,249.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=742.03,337.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=627.18,397.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=530.41,417.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=588.56,509.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=609.09,593.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=714.89,474.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=790.72,525.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=713.96,558.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=654.70,548.46");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=701.47,626.49");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=764.36,635.72");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=850.73,617.22");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=834.93,697.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=982.79,657.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1029.08,617.37");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=955.68,739.10");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=915.51,687.69");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=904.40,778.29");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=923.28,865.37");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1031.19,796.59");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1035.82,887.13");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1173.84,925.16");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1181.93,997.09");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=980.54,916.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=909.41,944.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1103.73,961.83");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1033.04,990.64");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1030.01,1072.12");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1062.97,1123.11");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1051.18,1216.15");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1048.25,1310.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=979.42,1274.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1018.32,1401.58");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=991.91,1457.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1087.80,1431.82");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1103.80,1523.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1168.00,1575.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1184.47,1501.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1233.34,1541.75");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1304.89,1557.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1366.43,1500.35");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1476.44,1552.57");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1537.61,1618.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1591.94,1670.44");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1608.24,1600.27");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1648.60,1554.74");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1713.12,1617.01");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1795.96,1586.21");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1829.52,1657.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1893.05,1627.64");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1907.93,1539.01");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1850.37,1468.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1984.80,1426.12");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2046.90,1473.69");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2076.92,1568.46");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2031.48,1640.81");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1964.80,1616.11");
    }

}

