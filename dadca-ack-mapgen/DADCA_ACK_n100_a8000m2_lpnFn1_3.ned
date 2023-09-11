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
        @display("i=misc/sensor,p=137.01,174.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=39.92,170.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=201.37,256.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=162.94,325.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=104.25,298.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=89.45,368.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=0.74,396.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=23.23,308.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=155.22,388.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=253.34,391.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=321.73,456.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=261.16,460.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=157.77,510.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=114.51,594.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=151.06,668.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=190.64,749.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=245.91,774.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=300.38,804.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=309.37,864.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=422.71,806.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=80.86,687.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=9.25,730.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=43.98,791.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=42.55,491.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=228.26,331.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=277.82,539.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=445.94,512.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=378.46,554.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=324.66,593.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=217.73,654.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=281.36,664.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=240.04,587.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=176.35,582.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=86.94,531.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=358.65,716.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=292.50,327.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=10.77,581.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=115.88,812.07");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=133.85,728.25");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=255.04,908.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=214.71,851.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=154.05,922.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=76.61,874.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=16.55,864.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=89.16,963.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=204.83,1040.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=245.09,970.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=152.15,1072.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=198.35,1139.23");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=258.54,1213.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=319.87,1175.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=316.13,1334.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=238.13,1331.77");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=162.57,1305.54");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=168.23,1239.56");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=110.11,1212.69");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=291.86,1022.24");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=380.11,1018.43");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=343.16,1076.85");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=421.63,1176.67");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=368.94,1223.20");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=34.56,1077.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=57.57,1150.29");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=138.20,1136.27");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=278.06,1101.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=406.25,1117.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=468.54,1057.22");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=391.40,1307.49");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=366.94,1392.18");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=306.20,1432.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=248.84,1479.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=158.73,1370.23");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=164.51,1430.52");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=249.58,1391.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=92.45,1539.92");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=22.04,1607.73");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=81.58,1629.42");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=38.08,1682.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=137.02,1687.82");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=145.89,1604.33");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=202.18,1556.60");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=29.48,1543.39");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=0.24,1471.96");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=116.66,1745.90");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=67.87,1852.36");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=103.62,1465.03");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=231.29,1621.35");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=314.13,1599.62");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=389.33,1652.21");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=416.95,1712.94");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=434.92,1803.80");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=312.98,1745.47");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=377.12,1763.12");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=304.40,1824.70");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=325.49,1901.42");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=368.40,1945.05");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=319.27,2026.72");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=247.30,1965.46");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=203.81,2048.17");
    }

}

