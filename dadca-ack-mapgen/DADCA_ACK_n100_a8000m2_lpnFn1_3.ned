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
        @display("i=misc/sensor,p=175.93,135.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=263.49,155.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=243.74,256.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=180.11,293.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=142.29,356.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=214.53,384.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=283.35,383.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=302.76,460.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=240.48,470.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=205.37,558.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=156.00,476.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=341.92,245.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=361.27,312.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=447.59,363.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=516.19,314.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=409.70,238.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=450.27,153.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=324.96,187.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=329.50,65.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=263.36,77.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=523.40,382.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=578.24,226.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=669.88,235.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=701.67,327.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=756.05,364.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=795.47,295.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=856.76,286.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=935.57,258.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=943.52,391.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=957.37,486.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=873.93,444.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1005.56,443.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1070.47,388.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1118.56,427.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1125.63,354.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1187.74,353.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1062.18,239.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1120.05,206.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1164.78,155.97");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1152.11,86.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1111.53,7.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1078.73,76.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1259.17,137.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1247.76,44.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1323.86,34.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1411.07,44.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1489.96,82.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1469.05,165.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1560.26,13.53");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1644.55,12.65");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1654.17,79.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1610.90,168.36");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1554.77,190.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1640.50,226.84");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1750.28,135.95");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1580.33,256.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1680.36,309.52");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1736.22,253.74");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1728.50,359.64");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1788.39,410.91");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1807.11,482.19");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1744.67,501.92");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1686.90,547.80");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1783.58,558.96");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1782.64,307.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1817.31,220.06");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1887.70,178.48");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1923.68,126.21");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1984.19,113.31");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1817.04,57.31");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1825.28,121.39");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1882.70,355.95");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1871.66,416.74");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1860.27,526.82");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1607.79,530.69");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1557.05,580.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1495.95,540.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1445.70,606.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1443.06,669.45");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1387.29,626.99");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1504.59,467.30");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1507.90,625.84");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1517.06,721.55");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1382.76,699.39");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1529.99,349.06");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1431.12,445.03");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1593.00,443.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1673.56,610.29");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1684.54,684.54");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1668.84,752.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1586.92,795.74");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1588.22,710.71");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1455.21,770.48");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1432.99,526.04");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1540.28,845.03");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1458.69,896.83");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1417.75,964.28");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1341.74,942.96");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1280.30,901.07");
    }

}

