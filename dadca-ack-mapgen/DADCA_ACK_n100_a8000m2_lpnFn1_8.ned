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

network DADCA_ACK_n100_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=168.19,120.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=129.32,41.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=220.01,55.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=263.44,100.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=273.09,193.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=209.36,210.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=112.46,220.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=96.80,161.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=322.73,245.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=289.82,330.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=285.83,393.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=150.34,360.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=159.35,284.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=271.81,15.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=224.42,290.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=33.17,396.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=30.95,482.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=91.42,426.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=103.21,490.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=161.52,444.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=219.69,477.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=222.74,403.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=438.61,259.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=359.55,310.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=392.48,380.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=560.61,288.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=646.77,320.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=709.80,383.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=779.99,432.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=823.01,495.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=748.36,523.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=781.10,603.50");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=817.33,658.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=870.84,685.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=866.25,762.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=927.49,770.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=752.92,756.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=750.24,668.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=901.35,591.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=962.50,520.18");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1006.98,594.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1038.77,678.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1027.46,752.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=967.93,678.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1088.88,775.25");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1177.99,805.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=951.39,831.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=817.10,831.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=964.16,896.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1034.25,908.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1027.86,984.60");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1038.16,1048.51");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=951.33,999.91");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=856.64,978.83");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=803.17,907.54");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=854.70,550.82");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1066.41,605.59");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1144.57,643.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1165.31,564.04");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1233.71,601.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1213.32,492.85");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1270.16,419.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1330.20,391.93");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1297.07,307.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1293.73,242.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1255.19,185.02");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1176.29,183.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1192.89,264.83");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1169.32,359.92");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1232.16,322.44");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1109.49,451.99");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1062.87,519.65");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1385.21,173.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1382.37,271.62");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1422.95,362.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1341.98,115.82");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1415.09,92.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1449.29,183.47");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1452.11,22.77");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1383.88,21.94");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1504.18,115.07");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1573.92,95.17");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1639.18,142.28");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1599.55,188.33");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1728.83,212.13");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1713.95,296.44");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1759.31,352.33");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1753.10,430.46");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1749.93,511.04");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1692.22,373.20");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1823.30,249.21");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1843.12,180.27");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1775.71,156.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1871.27,301.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1652.99,259.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1641.41,415.85");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1571.89,454.73");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1551.84,359.85");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1507.67,404.09");
    }

}

