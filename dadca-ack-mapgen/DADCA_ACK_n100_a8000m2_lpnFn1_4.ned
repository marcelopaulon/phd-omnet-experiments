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
        @display("i=misc/sensor,p=158.31,133.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=152.14,39.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=200.01,80.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=224.56,167.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=263.73,217.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=194.90,242.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=170.55,308.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=143.82,374.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=130.19,458.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=71.12,328.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=46.82,260.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=11.98,181.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=125.22,212.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=230.48,364.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=355.42,302.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=396.64,363.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=310.18,351.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=379.43,421.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=373.33,215.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=451.72,177.18");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=516.62,234.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=600.82,244.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=675.36,304.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=749.37,332.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=825.11,309.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=822.88,426.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=885.96,455.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=912.76,375.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=977.63,324.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=939.26,268.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=948.55,428.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1007.38,396.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1078.07,421.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1064.58,517.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=998.54,501.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1078.66,293.59");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1098.84,360.40");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1179.13,390.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1171.50,292.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1199.95,214.98");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1139.62,233.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1303.80,235.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1283.06,161.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1347.02,141.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1363.19,212.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1413.99,178.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1466.97,119.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1563.60,130.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1597.97,200.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1688.25,168.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1719.77,91.98");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1652.02,77.25");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1749.83,246.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1761.03,149.12");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1775.74,52.78");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1833.57,99.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1824.42,253.52");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1899.12,227.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1857.66,182.58");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1621.31,286.60");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1615.34,373.96");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1542.09,403.57");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1533.21,252.74");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1495.49,183.92");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1968.59,86.06");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=2041.57,40.98");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1929.85,171.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1907.42,31.15");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1967.39,14.56");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1897.83,119.12");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1547.90,322.22");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1672.37,227.69");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1690.10,318.91");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1760.39,312.21");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1498.19,16.67");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1557.66,40.02");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1622.83,1.90");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1853.28,330.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1826.45,413.88");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1865.86,465.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1800.58,516.21");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1743.89,464.03");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1735.93,540.48");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1677.04,454.97");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1459.09,322.68");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=2015.90,141.45");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=2029.88,217.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=2015.73,282.18");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=2095.93,228.60");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=2119.28,309.71");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=2196.97,350.93");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=2246.17,311.47");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=2169.71,427.98");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=2238.54,398.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=2262.70,493.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=2199.42,522.89");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=2353.22,498.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=2337.47,576.18");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=2269.76,553.45");
    }

}

