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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=165.19,37.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=239.43,6.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=256.60,80.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=178.63,119.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=63.96,34.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=29.98,89.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=78.79,168.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=247.82,163.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=155.32,269.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=66.70,311.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=84.88,410.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=79.47,495.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=196.22,531.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=231.09,586.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=220.35,679.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=326.25,579.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=287.46,510.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=319.81,453.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=336.75,391.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=218.00,457.27");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=438.52,540.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=401.43,458.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=463.41,388.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=538.76,365.36");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=608.91,295.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=688.85,284.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=664.53,369.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=780.74,270.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=529.19,277.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=593.36,203.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=505.31,210.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=718.35,436.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=746.81,494.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=800.37,530.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=867.84,585.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=935.02,528.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=983.83,613.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1050.08,585.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1027.88,676.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=857.88,502.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=817.00,429.35");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=931.35,368.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=874.79,347.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=915.42,435.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=752.12,328.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=809.58,197.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=758.38,143.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=704.88,218.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=665.50,151.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=709.06,98.34");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=624.71,92.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=734.32,0.84");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=795.99,17.82");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=788.49,86.40");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=917.27,2.32");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=943.94,60.40");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1007.78,117.82");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=962.58,162.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1003.89,208.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1063.65,283.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1145.60,313.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1181.82,377.81");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1252.73,405.33");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1286.47,466.88");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1370.71,428.37");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1461.41,436.04");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1522.11,411.92");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1525.91,497.09");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1559.16,563.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1438.31,500.20");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1392.60,596.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1493.73,617.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1530.96,675.29");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1529.10,746.52");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1631.21,770.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1577.41,815.43");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1574.77,904.19");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1507.59,896.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1549.93,966.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1496.13,1079.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1592.25,1106.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1643.92,1058.35");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1706.91,1085.49");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1764.72,1121.13");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1832.63,1088.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1863.35,1154.58");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1894.24,1069.57");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1820.24,1227.07");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1752.80,1215.16");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1685.83,1195.52");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1608.78,1257.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1655.18,1339.58");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1773.14,1300.18");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1695.06,1255.88");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1814.22,1029.17");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1858.48,943.64");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1929.41,989.94");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1950.26,925.69");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=2005.37,987.46");
    }

}

