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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=28.96,71.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=41.73,11.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=137.15,11.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=203.06,61.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=225.01,3.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=326.23,54.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=266.06,65.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=405.46,68.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=382.86,152.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=318.78,186.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=398.53,243.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=473.91,164.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=124.01,194.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=34.93,151.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=86.47,253.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=74.07,318.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=3.20,357.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=27.16,426.29");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=145.48,394.76");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=148.66,458.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=89.25,470.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=37.24,523.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=36.33,619.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=104.13,682.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=186.13,738.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=195.76,653.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=167.86,577.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=247.70,570.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=105.05,779.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=180.76,806.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=51.03,833.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=15.91,888.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=70.81,945.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=117.57,1007.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=195.38,966.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=217.44,1061.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=99.48,1082.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=34.03,1044.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=30.44,1141.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=70.21,1191.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=97.39,1244.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=146.83,1310.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=212.80,1279.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=265.30,1326.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=153.98,1221.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=111.15,1360.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=68.70,1424.90");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=91.54,1559.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=115.38,1646.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=199.00,1478.77");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=246.00,1422.32");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=338.91,1390.57");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=273.79,1226.25");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=273.12,1157.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=196.97,1175.22");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=325.52,1283.06");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=343.86,1204.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=393.05,1263.33");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=388.94,1153.61");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=290.67,1480.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=341.55,1516.97");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=418.53,1477.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=400.89,1546.28");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=146.02,1513.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=234.57,1608.05");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=319.22,1580.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=350.95,1660.73");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=407.89,1371.88");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=473.87,1409.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=554.94,1411.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=546.41,1350.86");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=608.56,1328.57");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=636.19,1469.74");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=480.93,1549.45");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=505.66,1630.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=591.79,1599.80");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=668.23,1537.40");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=743.88,1580.56");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=736.30,1513.54");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=808.21,1547.40");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=798.20,1481.95");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=744.66,1710.34");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=789.13,1772.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=858.76,1764.62");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=871.36,1705.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=785.82,1661.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=668.01,1632.53");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=645.34,1704.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=719.61,1796.86");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=847.32,1838.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=972.65,1760.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=595.65,1781.40");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=515.86,1739.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=450.45,1759.24");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=372.17,1777.42");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=396.08,1873.75");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=423.06,1948.57");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=481.67,2002.73");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=462.01,1901.83");
    }

}

