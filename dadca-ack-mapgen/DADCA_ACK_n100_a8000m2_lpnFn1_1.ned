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
        @display("i=misc/sensor,p=115.61,168.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=166.99,216.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=236.92,161.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=243.81,243.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=345.51,93.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=373.78,173.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=263.34,74.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=162.93,361.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=106.22,428.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=224.85,421.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=223.28,347.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=81.79,485.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=177.17,475.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=309.76,456.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=323.25,254.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=388.07,318.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=177.10,291.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=103.37,279.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=99.77,571.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=26.75,609.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=34.87,384.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=157.58,544.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=204.95,606.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=202.36,697.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=136.18,715.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=194.38,776.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=233.50,862.92");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=276.00,918.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=199.56,915.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=129.19,959.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=51.26,1006.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=56.37,1072.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=111.82,1120.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=181.73,1168.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=18.00,1192.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=83.89,1229.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=2.86,1128.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1.78,1045.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=253.19,1022.17");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=250.65,1121.72");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=344.30,1124.42");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=395.47,1177.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=439.29,1232.56");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=389.74,1302.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=329.80,1289.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=333.23,1211.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=362.79,1367.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=313.79,1416.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=289.07,1477.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=377.11,1512.43");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=301.39,1346.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=245.35,1411.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=400.82,1421.36");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=153.39,1526.65");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=77.85,1504.14");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=26.19,1467.71");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=42.69,1372.03");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=131.12,1410.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=181.14,1451.90");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=128.36,1647.81");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=213.85,1602.21");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=287.91,1607.25");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=355.67,1610.37");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=521.92,1371.53");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=613.43,1355.54");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=590.49,1431.07");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=503.69,1477.84");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=436.21,1480.02");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=435.52,1562.34");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=462.37,1622.96");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=418.44,1678.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=463.38,1720.32");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=381.65,1727.59");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=342.22,1794.94");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=520.46,1741.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=561.78,1666.45");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=638.91,1629.05");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=702.80,1674.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=608.12,1577.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=571.95,1510.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=481.66,1312.64");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=242.01,1314.15");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=302.46,985.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=192.21,1063.14");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=129.14,1060.06");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=115.88,900.37");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=15.47,904.60");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=58.81,818.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=293.16,854.89");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=383.29,876.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=350.74,789.23");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=251.27,747.12");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=132.18,776.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=485.64,906.78");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=564.96,896.00");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=652.72,909.20");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=697.82,839.50");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=748.59,774.12");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=677.24,718.07");
    }

}

