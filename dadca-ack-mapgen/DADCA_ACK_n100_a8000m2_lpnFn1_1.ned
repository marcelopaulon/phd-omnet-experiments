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

network DADCA_ACK_n100_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=45.08,28.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=155.55,61.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=224.79,9.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=244.07,73.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=300.77,15.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=312.01,102.67");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=377.29,173.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=473.78,197.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=424.82,128.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=395.28,294.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=392.05,231.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=312.71,193.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=309.33,264.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=277.88,317.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=331.49,390.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=415.19,441.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=417.62,536.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=462.53,579.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=365.27,586.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=311.09,669.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=323.97,747.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=266.22,823.38");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=195.07,845.96");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=170.59,782.04");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=79.33,790.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=128.67,854.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=132.34,654.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=114.47,736.03");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=34.43,745.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=54.87,683.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=15.54,838.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=218.50,718.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=164.14,907.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=118.12,986.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=142.66,1064.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=67.35,1086.57");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=239.39,1090.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=187.05,1022.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=328.64,1013.77");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=301.06,1069.08");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=322.99,1148.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=374.41,1228.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=392.63,1138.98");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=469.56,1128.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=360.79,1344.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=437.92,1348.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=332.92,1281.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=499.77,1222.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=463.52,1288.67");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=271.98,1328.22");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=543.62,1276.87");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=626.58,1314.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=654.58,1376.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=725.95,1410.15");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=692.19,1482.33");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=588.26,1482.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=634.58,1544.28");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=672.62,1605.66");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=736.92,1614.54");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=605.59,1604.72");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=520.34,1633.05");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=483.48,1709.22");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=423.52,1696.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=459.27,1785.16");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=456.17,1504.38");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=443.13,1434.24");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=507.94,1440.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=519.07,1536.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=813.72,1500.92");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=471.96,1575.78");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=371.55,1417.79");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=403.79,1552.21");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=358.02,1635.59");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=386.41,1494.23");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=423.48,1619.55");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=332.40,1558.60");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=266.79,1483.96");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=237.70,1587.38");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=335.80,1727.52");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=275.61,1687.12");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=232.58,1735.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=303.02,1789.83");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=118.55,1729.94");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=29.24,1772.38");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=71.34,1827.29");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=131.44,1802.69");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=59.74,1690.33");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=143.07,1865.02");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=63.35,1894.32");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=208.80,1855.11");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=221.97,1796.00");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=613.84,1694.99");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=686.40,1745.19");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=658.81,1809.76");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=802.74,1778.14");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=881.51,1804.61");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=777.90,1840.91");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=825.24,1903.05");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=747.49,1898.48");
    }

}

